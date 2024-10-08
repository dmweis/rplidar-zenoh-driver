use clap::Parser;
use prost::Message;
use rplidar_driver::{utils::sort_scan, RplidarDevice, RposError, ScanOptions, ScanPoint};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, SystemTime},
};
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tracing::{error, info, log::warn};
use zenoh::{config::Config, prelude::r#async::*};

use rplidar_zenoh_driver::{
    foxglove, rp_lidar_projected_points_to_foxglove_point_cloud, setup_tracing,
    system_time_to_proto_time, RpLidarProjectedPoint,
};

#[derive(Parser, Debug)]
#[command()]
struct Args {
    /// Turn of lidar
    #[clap(long)]
    lidar_off: bool,

    /// serial port for lidar
    #[clap(long)]
    serial_port: String,

    /// zenoh prefix
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

    let state_topic = format!("{}/state", args.prefix)
        .trim_matches('/')
        .to_owned();
    let subscriber = zenoh_session
        .declare_subscriber(&state_topic)
        .res()
        .await
        .unwrap();

    let laser_scan_topic = format!("{}/{}", args.prefix, args.scan_topic)
        .trim_matches('/')
        .to_owned();
    let laser_scan_publisher = zenoh_session
        .declare_publisher(laser_scan_topic)
        .res()
        .await
        .unwrap();

    let point_cloud_topic = format!("{}/{}", args.prefix, args.cloud_topic)
        .trim_matches('/')
        .to_owned();
    let point_cloud_publisher = zenoh_session
        .declare_publisher(point_cloud_topic)
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

    tokio::spawn(async move {
        loop {
            if let Ok(sample) = subscriber.recv_async().await {
                info!("Received message: {}", sample);
                if let Ok(message) = TryInto::<String>::try_into(&sample.value) {
                    info!("Message: {}", message);
                    let lidar_command_on = message.to_lowercase().ends_with("on");
                    if lidar_command_on {
                        info!("Starting scan");
                        should_lidar_run.store(true, Ordering::Relaxed);
                    } else {
                        info!("Stopping scan");
                        should_lidar_run.store(false, Ordering::Relaxed);
                    }
                } else {
                    warn!("Failed to parse message: {:?}", sample.value);
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

        let start_angle = scan.first().map(|point| point.angle()).unwrap_or_default();
        let end_angle = scan
            .iter()
            .last()
            .map(|point| point.angle())
            .unwrap_or_default();

        // laser scan
        let laser_scan = foxglove::LaserScan {
            timestamp: Some(system_time_to_proto_time(&capture_time)),
            frame_id: args.frame_id.clone(),
            pose: Some(pose),
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

                RpLidarProjectedPoint::new(x, y, scan_point.distance(), scan_point.angle(), quality)
            })
            .collect::<Vec<_>>();

        let point_cloud = rp_lidar_projected_points_to_foxglove_point_cloud(
            &capture_time,
            &args.frame_id,
            &pose,
            &projected_scan,
        );

        point_cloud_publisher
            .put(point_cloud.encode_to_vec())
            .res()
            .await
            .unwrap();
    }

    Ok(())
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
        move || loop {
            if let Err(err) = lidar_loop(&port, scan_sender.clone(), should_lidar_run.clone()) {
                error!("Lidar loop error: {}", err);
                thread::sleep(Duration::from_secs(1));
            }
        }
    });

    Ok((scan_receiver, should_lidar_run))
}

fn lidar_loop(
    port: &str,
    scan_sender: Sender<Vec<ScanPoint>>,
    should_lidar_run: Arc<AtomicBool>,
) -> anyhow::Result<()> {
    let mut lidar = RplidarDevice::open_port(port)?;
    // start with this flag opposite of desired so that we set the lidar to correct start
    let mut lidar_running = !should_lidar_run.load(Ordering::Relaxed);
    loop {
        match should_lidar_run.load(Ordering::Relaxed) {
            true => {
                if !lidar_running {
                    lidar.start_motor()?;
                    let scan_options = ScanOptions::with_mode(2);
                    let _ = lidar.start_scan_with_options(&scan_options)?;
                    lidar_running = true;
                }
                match lidar.grab_scan() {
                    Ok(scan) => {
                        scan_sender.blocking_send(scan)?;
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
                    lidar.stop_motor()?;
                    lidar_running = false;
                }
                false => thread::sleep(std::time::Duration::from_millis(500)),
            },
        }
    }
}
