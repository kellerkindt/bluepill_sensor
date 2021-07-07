use crate::platform::Platform;
use sensor_common::props::{ComponentRoot, CpuComponent, DeviceComponent};
use sensor_common::{Error, Read, Type, Write};

#[macro_export]
macro_rules! property_read_fn {
    (|$platform:ident, $module:ident: &mut $moduleTy:ty, $write:ident| $body:expr) => {{
        fn read_fn(
            $platform: &mut Platform,
            $module: &mut $moduleTy,
            $write: &mut dyn Write,
        ) -> Result<usize, sensor_common::Error> {
            $body
        }
        Some(read_fn)
    }};
    (|$platform:ident, $write:ident| $body:expr) => {
        property_read_fn! { |$platform, _t: &mut (), $write| $body }
    };
}

pub struct Property<T> {
    pub id: &'static [u8],
    pub name: Option<&'static str>,
    pub ty: Type,
    pub read: Option<fn(&mut Platform, &mut T, &mut dyn Write) -> Result<usize, Error>>,
    pub write: Option<fn(&mut Platform, &mut T, &mut dyn Read) -> Result<usize, Error>>,
}

pub const PROPERTIES: &'static [Property<()>] = &[
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::Id as u8,
        ],
        name: Some("cpu-id"),
        ty: Type::U32,
        read: property_read_fn! {
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
        read: property_read_fn! {
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
        read: property_read_fn! {
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
        read: property_read_fn! {
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
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_revision().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[ComponentRoot::Device as u8, DeviceComponent::Uptime as u8],
        name: Some("Device uptime"),
        ty: Type::U64,
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.system.info.uptime_ms().to_be_bytes())
        },
        write: None,
    },
];
