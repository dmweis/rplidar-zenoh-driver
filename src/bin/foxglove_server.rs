use base64::{engine::general_purpose, Engine as _};
use clap::Parser;
use mcap::records::system_time_to_nanos;
use once_cell::sync::Lazy;
use prost_reflect::{DescriptorPool, ReflectMessage};
use std::{net::SocketAddr, time::SystemTime};
use tokio::{select, signal};
use tracing::info;
use zenoh::{config::Config, prelude::r#async::*};

use rplidar_zenoh_driver::setup_tracing;

static FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/file_descriptor_set.bin"));

static DESCRIPTOR_POOL: Lazy<DescriptorPool> = Lazy::new(|| {
    DescriptorPool::decode(FILE_DESCRIPTOR_SET).expect("Failed to load file descriptor set")
});

/// protobuf
pub mod foxglove {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

#[derive(Parser, Debug)]
#[command()]
struct Args {
    /// publish topic
    #[clap(long, default_value = "laser_scan")]
    scan_topic: String,

    /// publish topic
    #[clap(long, default_value = "point_cloud")]
    cloud_topic: String,

    /// listen on
    #[clap(long)]
    listen: Vec<String>,

    /// connect to
    #[clap(long)]
    connect: Vec<String>,

    /// foxglove bind address
    #[clap(long, default_value = "127.0.0.1:8765")]
    host: SocketAddr,
}

const PROTOBUF_ENCODING: &str = "protobuf";

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();
    setup_tracing()?;

    let server = foxglove_ws::FoxgloveWebSocket::new();
    tokio::spawn({
        let server = server.clone();
        async move { server.serve(args.host).await }
    });

    let mut zenoh_config = Config::default();
    if !args.listen.is_empty() {
        zenoh_config.listen.endpoints = args
            .listen
            .iter()
            .map(|endpoint| endpoint.parse().unwrap())
            .collect();
        info!(listen_endpoints= ?zenoh_config.listen.endpoints, "Configured listening endpoints");
    }

    if !args.connect.is_empty() {
        zenoh_config.connect.endpoints = args
            .connect
            .iter()
            .map(|endpoint| endpoint.parse().unwrap())
            .collect();
        info!(connect_endpoints= ?zenoh_config.connect.endpoints, "Configured connect endpoints");
    }

    let zenoh_session = zenoh::open(zenoh_config).res().await.unwrap();
    info!("Started zenoh session");

    let laser_scan_subscriber = zenoh_session
        .declare_subscriber(&args.scan_topic)
        .res()
        .await
        .unwrap();

    let point_cloud_subscriber = zenoh_session
        .declare_subscriber(&args.cloud_topic)
        .res()
        .await
        .unwrap();

    let laser_scan_message = foxglove::LaserScan::default();
    let laser_scan_schema_data = laser_scan_message
        .descriptor()
        .parent_pool()
        .encode_to_vec();
    let laser_scan_schema_encoded: String =
        general_purpose::STANDARD_NO_PAD.encode(laser_scan_schema_data);
    let laser_scan_channel = server
        .publish(
            args.scan_topic.clone(),
            PROTOBUF_ENCODING.to_string(),
            laser_scan_message.descriptor().full_name().to_owned(),
            laser_scan_schema_encoded,
            PROTOBUF_ENCODING.to_string(),
            false,
        )
        .await?;

    let point_cloud_message = foxglove::PointCloud::default();
    let point_cloud_schema_data = point_cloud_message
        .descriptor()
        .parent_pool()
        .encode_to_vec();
    let point_cloud_schema_encoded: String =
        general_purpose::STANDARD_NO_PAD.encode(point_cloud_schema_data);
    let point_cloud_channel = server
        .publish(
            args.cloud_topic.clone(),
            PROTOBUF_ENCODING.to_string(),
            point_cloud_message.descriptor().full_name().to_owned(),
            point_cloud_schema_encoded,
            PROTOBUF_ENCODING.to_string(),
            false,
        )
        .await?;

    let mut laser_scan_counter = 0;
    let mut point_cloud_counter = 0;
    loop {
        select!(
            sample = laser_scan_subscriber.recv_async() => {
                let sample = sample.unwrap();
                laser_scan_counter+= 1;
                let now = SystemTime::now();
                let time_nanos = system_time_to_nanos(&now);
                let payload: Vec<u8> = sample.value.try_into()?;
                laser_scan_channel.send(time_nanos, &payload).await?;

                if laser_scan_counter % 20 == 0 {
                    info!("laser_scan_counter: {}", laser_scan_counter);
                }
            },

            sample = point_cloud_subscriber.recv_async() => {
                let sample = sample.unwrap();
                point_cloud_counter+= 1;
                let now = SystemTime::now();
                let time_nanos = system_time_to_nanos(&now);
                let payload: Vec<u8> = sample.value.try_into()?;
                point_cloud_channel.send(time_nanos, &payload).await?;

                if point_cloud_counter % 20 == 0 {
                    info!("point_cloud_counter: {}", point_cloud_counter);
                }
            },
            _ = signal::ctrl_c() => {
                info!("ctrl-c received, exiting");
                break;
            }
        );
    }

    Ok(())
}
