use clap::Parser;
use foxglove_ws::{Channel, FoxgloveWebSocket};
use mcap::records::system_time_to_nanos;
use prost::Message;
use prost_reflect::ReflectMessage;
use std::{net::SocketAddr, sync::Arc, time::SystemTime};
use tokio::signal;
use tracing::{error, info};
use zenoh::{config::Config, prelude::r#async::*, subscriber::FlumeSubscriber};

use rplidar_zenoh_driver::{foxglove, setup_tracing, ErrorWrapper};

#[derive(Parser, Debug)]
#[command()]
struct Args {
    /// lidar prefix
    ///
    /// Prefix for all topics
    #[clap(long, default_value = "rplidar")]
    prefix: String,

    /// publish topic
    #[clap(long, default_value = "laser_scan")]
    scan_topic: String,

    /// publish topic
    #[clap(long, default_value = "point_cloud")]
    cloud_topic: String,

    /// Endpoints to connect to.
    #[clap(short = 'e', long)]
    connect: Vec<zenoh_config::EndPoint>,

    /// Endpoints to listen on.
    #[clap(long)]
    listen: Vec<zenoh_config::EndPoint>,

    /// foxglove bind address
    #[clap(long, default_value = "0.0.0.0:8765")]
    host: SocketAddr,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();
    setup_tracing()?;

    // start foxglove server
    let server = foxglove_ws::FoxgloveWebSocket::default();
    tokio::spawn({
        let server = server.clone();
        async move { server.serve(args.host).await }
    });

    // configure zenoh
    let mut zenoh_config = Config::default();
    if !args.listen.is_empty() {
        zenoh_config.listen.endpoints.clone_from(&args.listen);
        info!(listen_endpoints= ?zenoh_config.listen.endpoints, "Configured listening endpoints");
    }
    if !args.connect.is_empty() {
        zenoh_config.connect.endpoints.clone_from(&args.connect);
        info!(connect_endpoints= ?zenoh_config.connect.endpoints, "Configured connect endpoints");
    }

    let zenoh_session = zenoh::open(zenoh_config)
        .res()
        .await
        .map_err(ErrorWrapper::ZenohError)?;
    let zenoh_session = zenoh_session.into_arc();
    info!("Started zenoh session");

    let scan_topic = format!("{}/{}", args.prefix, args.scan_topic)
        .trim_matches('/')
        .to_owned();
    start_proto_subscriber(
        &scan_topic,
        zenoh_session.clone(),
        &server,
        &foxglove::LaserScan::default(),
    )
    .await?;

    let cloud_topic = format!("{}/{}", args.prefix, args.cloud_topic)
        .trim_matches('/')
        .to_owned();
    start_proto_subscriber(
        &cloud_topic,
        zenoh_session.clone(),
        &server,
        &foxglove::PointCloud::default(),
    )
    .await?;

    signal::ctrl_c().await?;
    info!("ctrl-c received, exiting");

    Ok(())
}

async fn start_proto_subscriber(
    topic: &str,
    zenoh_session: Arc<Session>,
    foxglove_server: &FoxgloveWebSocket,
    protobuf: &dyn ReflectMessage,
) -> anyhow::Result<()> {
    info!(topic, "Starting proto subscriber");
    let zenoh_subscriber = zenoh_session
        .declare_subscriber(topic)
        .res()
        .await
        .map_err(ErrorWrapper::ZenohError)?;

    let foxglove_channel = create_publisher_for_protobuf(protobuf, foxglove_server, topic).await?;

    tokio::spawn({
        let topic = topic.to_owned();
        async move {
            loop {
                if let Err(err) =
                    zenoh_listener_loop(&topic, &zenoh_subscriber, &foxglove_channel).await
                {
                    error!(?topic, ?err, "Zenoh listener failed");
                }
            }
        }
    });
    Ok(())
}

async fn zenoh_listener_loop(
    topic: &str,
    zenoh_subscriber: &FlumeSubscriber<'_>,
    foxglove_channel: &Channel,
) -> anyhow::Result<()> {
    let mut message_counter = 0;
    loop {
        let sample = zenoh_subscriber.recv_async().await?;
        message_counter += 1;
        let now = SystemTime::now();
        let time_nanos = system_time_to_nanos(&now);
        let payload = if let Ok(blob) = TryInto::<Vec<u8>>::try_into(&sample.value) {
            blob
        } else if let Ok(text) = TryInto::<String>::try_into(&sample.value) {
            text.encode_to_vec()
        } else {
            anyhow::bail!("Failed to convert message type");
        };
        foxglove_channel.send(time_nanos, &payload).await?;

        if message_counter % 20 == 0 {
            info!(
                topic,
                message_counter, "{} sent {} messages", topic, message_counter
            );
        }
    }
}

const PROTOBUF_ENCODING: &str = "protobuf";

async fn create_publisher_for_protobuf(
    protobuf: &dyn ReflectMessage,
    foxglove_server: &FoxgloveWebSocket,
    topic: &str,
) -> anyhow::Result<Channel> {
    let protobuf_schema_data = protobuf.descriptor().parent_pool().encode_to_vec();
    foxglove_server
        .create_publisher(
            topic,
            PROTOBUF_ENCODING,
            protobuf.descriptor().full_name(),
            protobuf_schema_data,
            Some(PROTOBUF_ENCODING),
            false,
        )
        .await
}

#[allow(dead_code)]
const JSON_ENCODING: &str = "json";

#[allow(dead_code)]
async fn start_json_subscriber(
    topic: &str,
    zenoh_session: Arc<Session>,
    foxglove_server: &FoxgloveWebSocket,
    type_name: &str,
    json_schema: &str,
    latched: bool,
) -> anyhow::Result<()> {
    info!(topic, "Starting json subscriber");
    let zenoh_subscriber = zenoh_session
        .declare_subscriber(topic)
        .res()
        .await
        .map_err(ErrorWrapper::ZenohError)?;
    let foxglove_channel = foxglove_server
        .create_publisher(
            topic,
            JSON_ENCODING,
            type_name,
            json_schema,
            Some("jsonschema"),
            latched,
        )
        .await?;

    tokio::spawn({
        let topic = topic.to_owned();
        async move {
            let mut message_counter = 0;
            loop {
                let sample = zenoh_subscriber.recv_async().await.unwrap();
                message_counter += 1;
                let now = SystemTime::now();
                let time_nanos = system_time_to_nanos(&now);
                let payload: Vec<u8> = sample.value.try_into().unwrap();
                foxglove_channel.send(time_nanos, &payload).await.unwrap();

                if message_counter % 20 == 0 {
                    info!(
                        topic,
                        message_counter, "{} sent {} messages", topic, message_counter
                    );
                }
            }
        }
    });
    Ok(())
}

#[allow(dead_code)]
const GENERIC_JSON_SCHEMA: &str = r#"
{
"title": "GenericJsonSchema",
"description": "Generic JSON Schema",
"type": "object",
"properties": {}
}
"#;
