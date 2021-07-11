use crate::platform::Platform;
use sensor_common::props::{
    ComponentRoot, CpuComponent, DeviceComponent, Property, QueryComplexity,
};
use sensor_common::{Type, Write};

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

pub const PROPERTIES: &'static [Property<Platform, ()>] = &[
    Property {
        id: &[
            ComponentRoot::Device as u8,
            DeviceComponent::Cpu as u8,
            CpuComponent::Id as u8,
        ],
        type_hint: Some(Type::U32),
        complexity: QueryComplexity::low(),
        description: Some("cpu-id"),
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
        type_hint: Some(Type::U8),
        complexity: QueryComplexity::low(),
        description: Some("cpu-implementer"),
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
        type_hint: Some(Type::U8),
        complexity: QueryComplexity::low(),
        description: Some("cpu-variant"),
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
        type_hint: Some(Type::U16),
        complexity: QueryComplexity::low(),
        description: Some("cpu-partnumber"),
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
        type_hint: Some(Type::U8),
        complexity: QueryComplexity::low(),
        description: Some("cpu-revision"),
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.system.info.cpu_revision().to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[ComponentRoot::Device as u8, DeviceComponent::Uptime as u8],
        type_hint: Some(Type::U64),
        complexity: QueryComplexity::low(),
        description: Some("device-uptime"),
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.system.info.uptime_ms().to_be_bytes())
        },
        write: None,
    },
];
