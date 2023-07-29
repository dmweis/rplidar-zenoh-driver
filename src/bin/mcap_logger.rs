use clap::Parser;
use mcap::{
    records::{system_time_to_nanos, MessageHeader},
    Channel, Schema, Writer,
};
use once_cell::sync::Lazy;
use prost_reflect::{DescriptorPool, ReflectMessage};
use std::{borrow::Cow, collections::BTreeMap, fs, io::BufWriter, sync::Arc, time::SystemTime};
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
    #![allow(non_snake_case)]
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

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

    /// output file
    #[clap(long, default_value = "out.mcap")]
    output: String,

    /// listen on
    #[clap(long)]
    listen: Vec<String>,

    /// connect to
    #[clap(long)]
    connect: Vec<String>,
}

const PROTOBUF_ENCODING: &str = "protobuf";

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();
    setup_tracing()?;

    info!(file = ?args.output, "Creating mcap output file");
    let mut out = Writer::new(BufWriter::new(fs::File::create(&args.output)?))?;

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

    let scan_topic = format!("{}/{}", args.prefix, args.scan_topic);
    let laser_scan_subscriber = zenoh_session
        .declare_subscriber(&scan_topic)
        .res()
        .await
        .unwrap();

    let point_cloud_topic = format!("{}/{}", args.prefix, args.cloud_topic);
    let point_cloud_subscriber = zenoh_session
        .declare_subscriber(&point_cloud_topic)
        .res()
        .await
        .unwrap();

    let laser_scan_message = foxglove::LaserScan::default();
    let laser_scan_channel_id =
        register_mcap_topic_for_protobuf(&laser_scan_message, &mut out, &scan_topic)?;

    let point_cloud_message = foxglove::PointCloud::default();
    let point_cloud_channel_id =
        register_mcap_topic_for_protobuf(&point_cloud_message, &mut out, &point_cloud_topic)?;

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
                out.write_to_known_channel(
                    &MessageHeader {
                        channel_id: laser_scan_channel_id,
                        sequence: laser_scan_counter,
                        log_time: time_nanos,
                        publish_time: time_nanos,
                    },
                    &payload,
                )?;
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
                out.write_to_known_channel(
                    &MessageHeader {
                        channel_id: point_cloud_channel_id,
                        sequence: point_cloud_counter,
                        log_time: time_nanos,
                        publish_time: time_nanos,
                    },
                    &payload,
                )?;
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

    out.finish()?;
    info!("mcap file closed");

    Ok(())
}

fn register_mcap_topic_for_protobuf(
    protobuf: &dyn ReflectMessage,
    mcap_writer: &mut Writer<BufWriter<fs::File>>,
    topic: &str,
) -> anyhow::Result<u16> {
    let schema = Some(Arc::new(Schema {
        name: protobuf.descriptor().full_name().to_owned(),
        encoding: PROTOBUF_ENCODING.to_owned(),
        // this includes all files
        // filter to only include relevant
        // https://mcap.dev/guides/cpp/protobuf#register-schema
        data: Cow::from(protobuf.descriptor().parent_pool().encode_to_vec()),
    }));

    let my_channel = Channel {
        topic: String::from(topic),
        schema,
        message_encoding: PROTOBUF_ENCODING.to_owned(),
        metadata: BTreeMap::default(),
    };

    Ok(mcap_writer.add_channel(&my_channel)?)
}
