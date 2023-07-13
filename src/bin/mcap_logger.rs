use clap::Parser;
use mcap::{
    records::{system_time_to_nanos, MessageHeader},
    Channel, Schema, Writer,
};
use once_cell::sync::Lazy;
use prost_reflect::DescriptorPool;
use prost_reflect::ReflectMessage;
use prost_types::Timestamp;
use std::{
    borrow::Cow,
    collections::BTreeMap,
    fs,
    io::BufWriter,
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};
use tokio::select;
use tokio::signal;
use zenoh::config::Config;
use zenoh::prelude::r#async::*;

static FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/file_descriptor_set.bin"));

static DESCRIPTOR_POOL: Lazy<DescriptorPool> = Lazy::new(|| {
    DescriptorPool::decode(FILE_DESCRIPTOR_SET).expect("Failed to load file descriptor set")
});

/// protobuf
pub mod foxglove {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

#[allow(dead_code)]
fn system_time_to_proto_time(time: &SystemTime) -> Timestamp {
    let duration = time.duration_since(UNIX_EPOCH).unwrap();
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}

#[derive(Parser, Debug)]
#[command()]
struct Args {
    /// publish topic
    #[clap(long, default_value = "laser_scan")]
    topic: String,

    /// listen on
    #[clap(long)]
    listen: Vec<String>,

    /// connect to
    #[clap(long)]
    connect: Vec<String>,
}

fn create_topic_for_message(
    message: &dyn ReflectMessage,
    out: &mut Writer<BufWriter<fs::File>>,
    topic: &str,
) -> anyhow::Result<u16> {
    let schema = Some(Arc::new(Schema {
        name: message.descriptor().full_name().to_owned(),
        encoding: String::from("protobuf"),
        // this includes all files
        // filter to only include relevant
        // https://mcap.dev/guides/cpp/protobuf#register-schema
        data: Cow::from(message.descriptor().parent_pool().encode_to_vec()),
    }));

    let my_channel = Channel {
        topic: String::from(topic),
        schema,
        message_encoding: String::from("protobuf"),
        metadata: BTreeMap::default(),
    };

    Ok(out.add_channel(&my_channel)?)
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();

    let mut out = Writer::new(BufWriter::new(fs::File::create("out.mcap")?))?;

    let mut zenoh_config = Config::default();
    if !args.listen.is_empty() {
        zenoh_config.listen.endpoints = args
            .listen
            .iter()
            .map(|endpoint| endpoint.parse().unwrap())
            .collect();
    }

    if !args.connect.is_empty() {
        zenoh_config.connect.endpoints = args
            .connect
            .iter()
            .map(|endpoint| endpoint.parse().unwrap())
            .collect();
    }

    let zenoh_session = zenoh::open(zenoh_config).res().await.unwrap();

    let laser_scan_subscriber = zenoh_session
        .declare_subscriber(&args.topic)
        .res()
        .await
        .unwrap();

    let point_cloud_subscriber = zenoh_session
        .declare_subscriber("point_cloud")
        .res()
        .await
        .unwrap();

    let laser_scan_message = foxglove::LaserScan::default();
    let laser_scan_channel_id =
        create_topic_for_message(&laser_scan_message, &mut out, "laser_scan")?;

    let point_cloud_message = foxglove::LaserScan::default();
    let point_cloud_channel_id =
        create_topic_for_message(&point_cloud_message, &mut out, "point_cloud")?;

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
            },

            _ = signal::ctrl_c() => {
                break;
            }
        );
    }

    out.finish()?;

    Ok(())
}
