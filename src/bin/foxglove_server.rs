use clap::Parser;
use foxglove_ws::{Channel, FoxgloveWebSocket};
use mcap::records::system_time_to_nanos;
use once_cell::sync::Lazy;
use prost_reflect::{DescriptorPool, ReflectMessage};
use std::{net::SocketAddr, sync::Arc, time::SystemTime};
use tokio::signal;
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

    /// Endpoints to connect to.
    #[clap(short = 'e', long)]
    connect: Vec<zenoh_config::EndPoint>,

    /// Endpoints to listen on.
    #[clap(long)]
    listen: Vec<zenoh_config::EndPoint>,

    /// foxglove bind address
    #[clap(long, default_value = "127.0.0.1:8765")]
    host: SocketAddr,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();
    setup_tracing()?;

    // start foxglove server
    let server = foxglove_ws::FoxgloveWebSocket::new();
    tokio::spawn({
        let server = server.clone();
        async move { server.serve(args.host).await }
    });

    // configure zenoh
    let mut zenoh_config = Config::default();
    if !args.listen.is_empty() {
        zenoh_config.listen.endpoints = args.listen.clone();
        info!(listen_endpoints= ?zenoh_config.listen.endpoints, "Configured listening endpoints");
    }
    if !args.connect.is_empty() {
        zenoh_config.connect.endpoints = args.connect.clone();
        info!(connect_endpoints= ?zenoh_config.connect.endpoints, "Configured connect endpoints");
    }

    let zenoh_session = zenoh::open(zenoh_config).res().await.unwrap();
    let zenoh_session = zenoh_session.into_arc();
    info!("Started zenoh session");

    start_proto_subscriber(
        &args.scan_topic,
        zenoh_session.clone(),
        &server,
        &foxglove::LaserScan::default(),
    )
    .await?;

    start_proto_subscriber(
        &args.cloud_topic,
        zenoh_session.clone(),
        &server,
        &foxglove::PointCloud::default(),
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/ikea_dimmer",
        zenoh_session.clone(),
        &server,
        "IkeaDimmer",
        IKEA_DIMMER_JSON_SCHEMA,
        false,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/motion/one",
        zenoh_session.clone(),
        &server,
        "MotionSensor",
        MOTION_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/motion/two",
        zenoh_session.clone(),
        &server,
        "MotionSensor",
        MOTION_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/motion/three",
        zenoh_session.clone(),
        &server,
        "MotionSensor",
        MOTION_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/contact/fridge",
        zenoh_session.clone(),
        &server,
        "ContactSensor",
        CONTACT_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/contact/main_door",
        zenoh_session.clone(),
        &server,
        "ContactSensor",
        CONTACT_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/contact/lock",
        zenoh_session.clone(),
        &server,
        "ContactSensor",
        CONTACT_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    start_json_subscriber(
        "zigbee2mqtt/climate_sensor/one",
        zenoh_session.clone(),
        &server,
        "ClimateSensor",
        CLIMATE_SENSOR_JSON_SCHEMA,
        true,
    )
    .await?;

    signal::ctrl_c().await.unwrap();
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
    let zenoh_subscriber = zenoh_session.declare_subscriber(topic).res().await.unwrap();

    let foxglove_channel = create_publisher_for_protobuf(protobuf, foxglove_server, topic).await?;

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

const JSON_ENCODING: &str = "json";

async fn start_json_subscriber(
    topic: &str,
    zenoh_session: Arc<Session>,
    foxglove_server: &FoxgloveWebSocket,
    type_name: &str,
    json_shcema: &str,
    latched: bool,
) -> anyhow::Result<()> {
    info!(topic, "Starting json subscriber");
    let zenoh_subscriber = zenoh_session.declare_subscriber(topic).res().await.unwrap();
    let foxglove_channel = foxglove_server
        .create_publisher(
            topic,
            JSON_ENCODING,
            type_name,
            json_shcema,
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

const IKEA_DIMMER_JSON_SCHEMA: &str = r#"
{
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "object",
    "properties": {
      "action": {
        "type": "string"
      },
      "battery": {
        "type": "integer"
      },
      "brightness": {
        "type": "integer"
      },
      "linkquality": {
        "type": "integer"
      }
    },
    "required": [
      "action",
      "battery",
      "brightness",
      "linkquality"
    ]
}
"#;

const MOTION_SENSOR_JSON_SCHEMA: &str = r#"
{
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "object",
    "properties": {
      "battery": {
        "type": "integer"
      },
      "battery_low": {
        "type": "boolean"
      },
      "linkquality": {
        "type": "integer"
      },
      "occupancy": {
        "type": "boolean"
      },
      "tamper": {
        "type": "boolean"
      },
      "voltage": {
        "type": "integer"
      }
    },
    "required": [
      "battery",
      "battery_low",
      "linkquality",
      "occupancy",
      "tamper",
      "voltage"
    ]
  }
"#;

const CONTACT_SENSOR_JSON_SCHEMA: &str = r#"
{
"$schema": "http://json-schema.org/draft-04/schema#",
"type": "object",
"properties": {
    "battery": {
    "type": "integer"
    },
    "battery_low": {
    "type": "boolean"
    },
    "contact": {
    "type": "boolean"
    },
    "linkquality": {
    "type": "integer"
    },
    "tamper": {
    "type": "boolean"
    },
    "voltage": {
    "type": "integer"
    }
},
"required": [
    "battery",
    "battery_low",
    "contact",
    "linkquality",
    "tamper",
    "voltage"
]
}
"#;

const CLIMATE_SENSOR_JSON_SCHEMA: &str = r#"
{
"$schema": "http://json-schema.org/draft-04/schema#",
"type": "object",
"properties": {
    "battery": {
    "type": "integer"
    },
    "humidity": {
    "type": "number"
    },
    "linkquality": {
    "type": "integer"
    },
    "temperature": {
    "type": "number"
    },
    "voltage": {
    "type": "integer"
    }
},
"required": [
    "battery",
    "humidity",
    "linkquality",
    "temperature",
    "voltage"
]
}
"#;
