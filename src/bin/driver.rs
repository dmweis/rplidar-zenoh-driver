use clap::Parser;
use once_cell::sync::Lazy;
use prost::Message;
use prost_reflect::DescriptorPool;
use prost_types::Timestamp;
use rplidar_driver::{utils::sort_scan, RplidarDevice, RposError, ScanOptions, ScanPoint};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{SystemTime, UNIX_EPOCH},
};
use tokio::sync::mpsc::{channel, Receiver};
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
    /// Turn of lidar
    #[clap(long)]
    lidar_off: bool,

    /// serial port for lidar
    #[clap(long)]
    serial_port: String,

    /// publish topic
    #[clap(long, default_value = "laser_scan")]
    scan_topic: String,

    /// publish topic
    #[clap(long, default_value = "point_cloud")]
    cloud_topic: String,

    /// frame_id
    #[clap(long, default_value = "lidar")]
    frame_id: String,

    /// listen on
    #[clap(long)]
    listen: Vec<String>,

    /// connect to
    #[clap(long)]
    connect: Vec<String>,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Args = Args::parse();
    setup_tracing()?;

    let (mut scan_receiver, should_lidar_run) =
        start_lidar_driver(&args.serial_port, !args.lidar_off)?;

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

    let zenoh_session = zenoh::open(zenoh_config).res().await.unwrap().into_arc();

    let subscriber = zenoh_session
        .declare_subscriber("lidar_state")
        .res()
        .await
        .unwrap();

    let laser_scan_publisher = zenoh_session
        .declare_publisher(args.scan_topic)
        .res()
        .await
        .unwrap();

    let point_cloud_publisher = zenoh_session
        .declare_publisher(args.cloud_topic)
        .res()
        .await
        .unwrap();

    let pose = foxglove::Pose {
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
    };

    let point_stride = 4 + 4 + 1;
    let point_cloud_fields = vec![
        foxglove::PackedElementField {
            name: "x".to_string(),
            offset: 0,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "y".to_string(),
            offset: 4,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "quality".to_string(),
            offset: 8,
            r#type: foxglove::packed_element_field::NumericType::Uint8 as i32,
        },
    ];

    tokio::spawn(async move {
        loop {
            if let Ok(sample) = subscriber.recv_async().await {
                info!("Received message: {}", sample);
                let message: String = sample.value.try_into().unwrap();
                info!("Message: {}", message);
                let lidar_command_on = message.to_lowercase().ends_with("on");
                if lidar_command_on {
                    info!("Starting scan");
                    should_lidar_run.store(true, Ordering::Relaxed);
                } else {
                    info!("Stopping scan");
                    should_lidar_run.store(false, Ordering::Relaxed);
                }
            }
        }
    });

    let mut scan_counter = 0;
    while let Some(mut scan) = scan_receiver.recv().await {
        let capture_time = SystemTime::now();
        scan_counter += 1;

        if scan_counter % 80 == 0 {
            info!("Scan counter: {}", scan_counter);
        }

        sort_scan(&mut scan)?;

        let start_angle = scan.get(0).map(|point| point.angle()).unwrap_or_default();
        let end_angle = scan
            .iter()
            .last()
            .map(|point| point.angle())
            .unwrap_or_default();

        // laser scan
        let laser_scan = foxglove::LaserScan {
            timestamp: Some(system_time_to_proto_time(&capture_time)),
            frame_id: args.frame_id.clone(),
            pose: Some(pose.clone()),
            start_angle: start_angle as f64,
            end_angle: end_angle as f64,
            ranges: scan.iter().map(|point| point.distance() as f64).collect(),
            intensities: scan.iter().map(|point| point.quality as f64).collect(),
        };

        laser_scan_publisher
            .put(laser_scan.encode_to_vec())
            .res()
            .await
            .unwrap();

        // point cloud
        let projected_scan = scan
            .iter()
            .filter(|scan| scan.is_valid())
            .map(|scan_point| {
                let x = scan_point.distance() * (-scan_point.angle()).cos();
                let y = scan_point.distance() * (-scan_point.angle()).sin();
                let quality = scan_point.quality;
                (x, y, quality)
            })
            .collect::<Vec<_>>();

        let point_cloud = foxglove::PointCloud {
            timestamp: Some(system_time_to_proto_time(&capture_time)),
            frame_id: args.frame_id.clone(),
            pose: Some(pose.clone()),
            point_stride,
            fields: point_cloud_fields.clone(),
            data: projected_scan
                .iter()
                .flat_map(|(x, y, quality)| {
                    // foxglove data is low endian
                    let mut data = x.to_le_bytes().to_vec();
                    data.extend(y.to_le_bytes());
                    data.extend(quality.to_le_bytes());
                    data
                })
                .collect(),
        };

        point_cloud_publisher
            .put(point_cloud.encode_to_vec())
            .res()
            .await
            .unwrap();
    }

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

fn start_lidar_driver(
    port: &str,
    start_with_lidar_running: bool,
) -> anyhow::Result<(Receiver<Vec<ScanPoint>>, Arc<AtomicBool>)> {
    let (scan_sender, scan_receiver) = channel(10);
    let should_lidar_run = Arc::new(AtomicBool::new(start_with_lidar_running));

    thread::spawn({
        let port = port.to_owned();
        let should_lidar_run = Arc::clone(&should_lidar_run);
        move || {
            let mut lidar = RplidarDevice::open_port(&port).unwrap();

            let mut lidar_running = false;

            loop {
                match should_lidar_run.load(Ordering::Relaxed) {
                    true => {
                        if !lidar_running {
                            let scan_options = ScanOptions::with_mode(2);
                            let _ = lidar.start_scan_with_options(&scan_options).unwrap();
                            lidar_running = true;
                        }
                        match lidar.grab_scan() {
                            Ok(scan) => {
                                scan_sender.blocking_send(scan).unwrap();
                            }
                            Err(err) => match err {
                                RposError::OperationTimeout => continue,
                                _ => info!("Error: {:?}", err),
                            },
                        }
                    }
                    false => match lidar_running {
                        true => {
                            info!("Stopping lidar");
                            lidar.stop_motor().unwrap();
                            lidar_running = false;
                        }
                        false => thread::sleep(std::time::Duration::from_millis(500)),
                    },
                }
            }
        }
    });

    Ok((scan_receiver, should_lidar_run))
}
