use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use once_cell::sync::Lazy;
use prost_reflect::DescriptorPool;
use prost_types::Timestamp;
use tracing::{dispatcher, Dispatch};
use tracing_subscriber::{prelude::__tracing_subscriber_SubscriberExt, EnvFilter, Registry};

pub fn setup_tracing() -> anyhow::Result<()> {
    let filter = EnvFilter::builder()
        .with_default_directive(tracing_subscriber::filter::LevelFilter::INFO.into())
        .parse("")?;

    let subscriber = Registry::default()
        .with(filter)
        .with(tracing_logfmt::layer());
    dispatcher::set_global_default(Dispatch::new(subscriber))
        .context("Global logger has already been set!")?;
    Ok(())
}

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

#[derive(thiserror::Error, Debug)]
pub enum ErrorWrapper {
    #[error("Zenoh error {0:?}")]
    ZenohError(#[from] zenoh::Error),
}

pub struct RpLidarProjectedPoint {
    pub x: f32,
    pub y: f32,
    pub distance: f32,
    pub angle: f32,
    pub quality: u8,
}

impl RpLidarProjectedPoint {
    pub fn new(x: f32, y: f32, distance: f32, angle: f32, quality: u8) -> Self {
        Self {
            x,
            y,
            distance,
            angle,
            quality,
        }
    }

    pub fn to_foxglove_blob(&self) -> [u8; 17] {
        // The total size is 4 + 4 + 4 + 4 + 1 = 17
        let mut result = [0u8; 17];
        // foxglove data is in little endian
        result[0..4].copy_from_slice(&self.x.to_le_bytes());
        result[4..8].copy_from_slice(&self.y.to_le_bytes());
        result[8..12].copy_from_slice(&self.distance.to_le_bytes());
        result[12..16].copy_from_slice(&self.angle.to_le_bytes());
        result[16] = self.quality;
        result
    }

    pub fn from_foxglove_point_cloud(point_cloud: &foxglove::PointCloud) -> Result<Vec<Self>> {
        let (point_stride, point_cloud_fields) = rp_lidar_projected_point_descriptor();

        assert_eq!(point_cloud.fields, point_cloud_fields);

        let data = &point_cloud.data;

        let expected_len = data.len() as u32 / point_stride;

        let mut parsed_point_cloud = Vec::with_capacity(expected_len as usize);

        for chunk in data.chunks_exact(point_stride as usize) {
            if chunk.len() != point_stride as usize {
                anyhow::bail!("chunk size doesn't equal point_stride");
            }
            let x = f32::from_le_bytes(chunk[0..4].try_into()?);
            let y = f32::from_le_bytes(chunk[4..8].try_into()?);
            let distance = f32::from_le_bytes(chunk[8..12].try_into()?);
            let angle = f32::from_le_bytes(chunk[12..16].try_into()?);
            let quality = chunk[16];

            let point = RpLidarProjectedPoint::new(x, y, distance, angle, quality);
            parsed_point_cloud.push(point);
        }

        Ok(parsed_point_cloud)
    }
}

pub fn rp_lidar_projected_point_descriptor() -> (u32, Vec<foxglove::PackedElementField>) {
    //                      x   y   dis ang quality
    let point_stride = 4 + 4 + 4 + 4 + 1;
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
            name: "distance".to_string(),
            offset: 8,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "angle".to_string(),
            offset: 12,
            r#type: foxglove::packed_element_field::NumericType::Float32 as i32,
        },
        foxglove::PackedElementField {
            name: "quality".to_string(),
            offset: 16,
            r#type: foxglove::packed_element_field::NumericType::Uint8 as i32,
        },
    ];

    (point_stride, point_cloud_fields)
}

pub fn rp_lidar_projected_points_to_foxglove_point_cloud(
    timestamp: &SystemTime,
    frame_id: &str,
    pose: &foxglove::Pose,
    points: &[RpLidarProjectedPoint],
) -> foxglove::PointCloud {
    let (point_stride, point_cloud_fields) = rp_lidar_projected_point_descriptor();

    foxglove::PointCloud {
        timestamp: Some(system_time_to_proto_time(timestamp)),
        frame_id: frame_id.to_owned(),
        pose: Some(pose.clone()),
        point_stride,
        fields: point_cloud_fields.clone(),
        data: points
            .iter()
            .flat_map(|point| point.to_foxglove_blob())
            .collect(),
    }
}

pub fn system_time_to_proto_time(time: &SystemTime) -> Timestamp {
    let duration = time
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards");
    Timestamp {
        seconds: duration.as_secs() as i64,
        nanos: duration.subsec_nanos() as i32,
    }
}
