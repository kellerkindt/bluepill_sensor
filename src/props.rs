use crate::platform::Platform;
use sensor_common::props::{ComponentRoot, CpuComponent, DeviceComponent};
use sensor_common::{Error, Read, Type, Write};

pub struct Property<T> {
    pub id: &'static [u8],
    pub name: Option<&'static str>,
    pub ty: Type,
    pub read: Option<fn(&T, &mut dyn Write) -> Result<usize, Error>>,
    pub write: Option<fn(&mut T, &mut dyn Read) -> Result<usize, Error>>,
}

macro_rules! read_fn {
    (|$platform:ident, $write:ident| $body:expr) => {{
        fn read_fn($platform: &Platform, $write: &mut dyn Write) -> Result<usize, Error> {
            $body
        }
        Some(read_fn)
    }};
}

pub const PROPERTIES: &'static [Property<Platform>] = &[
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::Id as u8,
        ],
        name: Some("cpu-id"),
        ty: Type::U32,
        read: read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_id().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::Implementer as u8,
        ],
        name: Some("cpu-implementer"),
        ty: Type::U8,
        read: read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_implementer().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::Variant as u8,
        ],
        name: Some("cpu-variant"),
        ty: Type::U8,
        read: read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_variant().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::PartNumber as u8,
        ],
        name: Some("cpu-partnumber"),
        ty: Type::U16,
        read: read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_partnumber().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::Revision as u8,
        ],
        name: Some("cpu-revision"),
        ty: Type::U8,
        read: read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_revision().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[ComponentRoot::Device as u8, DeviceComponent::Uptime as u8],
        name: Some("Device uptime"),
        ty: Type::U64,
        read: read_fn! {
            |platform, write| write.write_all(&platform.system.info.uptime_ms().to_be_bytes())
        },
        write: None,
    },
];
