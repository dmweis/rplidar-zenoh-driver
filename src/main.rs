use clap::Parser;
use once_cell::sync::Lazy;
use prost::Message;
use prost_reflect::DescriptorPool;
use prost_types::Timestamp;
use rplidar_driver::{utils::sort_scan, RplidarDevice, RposError, ScanOptions};
use std::time::{SystemTime, UNIX_EPOCH};
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

#[derive(Parser, Debug)]
#[command()]
struct Args {
    /// Turn on lidar
    #[clap(long)]
    lidar_on: bool,

    /// serial port for lidar
    #[clap(long)]
    port: String,

    /// publish topic
    #[clap(long, default_value = "String::from(\"laser_scan\")")]
    topic: String,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();

    let mut lidar = RplidarDevice::open_port(&args.port)?;

    if !args.lidar_on {
        lidar.stop_motor()?;
        println!("Lidar paused.");
        wait_for_enter()?;
        return Ok(());
    }

    let zenoh_session = zenoh::open(Config::default()).res().await.unwrap();

    let publisher = zenoh_session
        .declare_publisher(args.topic)
        .res()
        .await
        .unwrap();

    let scan_options = ScanOptions::with_mode(2);
    let _ = lidar.start_scan_with_options(&scan_options)?;
    loop {
        match lidar.grab_scan() {
            Ok(mut scan) => {
                sort_scan(&mut scan)?;

                let _projected_scan = scan
                    .iter()
                    .filter(|scan| scan.is_valid())
                    .map(|scan_point| {
                        let x = scan_point.distance() * (-scan_point.angle()).cos();
                        let y = scan_point.distance() * (-scan_point.angle()).sin();
                        (x, y)
                    })
                    .collect::<Vec<_>>();

                let now = SystemTime::now();

                let laser_scan = foxglove::LaserScan {
                    timestamp: Some(system_time_to_proto_time(&now)),
                    frame_id: "lidar".to_string(),
                    pose: Some(foxglove::Pose {
                        position: Some(foxglove::Vector3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        }),
                        orientation: Some(foxglove::Quaternion {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                            w: 0.0,
                        }),
                    }),
                    start_angle: 0.0,
                    end_angle: std::f64::consts::PI * 2.0,
                    ranges: scan.iter().map(|point| point.distance() as f64).collect(),
                    intensities: vec![],
                };

                publisher
                    .put(laser_scan.encode_to_vec())
                    .res()
                    .await
                    .unwrap();
            }
            Err(err) => match err {
                RposError::OperationTimeout => continue,
                _ => println!("Error: {:?}", err),
            },
        }
    }
}

fn wait_for_enter() -> anyhow::Result<()> {
    use std::io::Read;
    println!("Press Enter to continue...");
    let _ = std::io::stdin().read(&mut [0])?;
    Ok(())
}

fn system_time_to_proto_time(time: &SystemTime) -> Timestamp {
    let duration = time
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards");
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}
