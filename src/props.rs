use crate::platform::Platform;
#[cfg(feature = "board-rev-3-0")]
use sensor_common::props::TemperatureComponent;
use sensor_common::props::{
    ComponentRoot, CpuComponent, DeviceComponent, EeePromComponent, MetaInformation,
    NetworkComponent, PlatformComponent, Property, QueryComplexity, SntpComponent,
};
use sensor_common::Type;

#[macro_export]
macro_rules! property_read_fn {
    (|$platform:ident, $module:ident: &mut $moduleTy:ty, $write:ident| $body:expr) => {{
        fn read_fn(
            $platform: &mut Platform,
            $module: &mut $moduleTy,
            $write: &mut dyn sensor_common::Write,
        ) -> Result<usize, sensor_common::Error> {
            {
                let _ = &($platform);
                let _ = &($module);
                let _ = &($write);
            };
            $body
        }
        Some(read_fn)
    }};
    (|$platform:ident, $write:ident| $body:expr) => {
        property_read_fn! { |$platform, _t: &mut (), $write| $body }
    };
}

#[macro_export]
macro_rules! property_write_fn {
    (|$platform:ident, $module:ident: &mut $moduleTy:ty, $read:ident| $body:expr) => {{
        fn write_fn(
            $platform: &mut Platform,
            $module: &mut $moduleTy,
            $read: &mut dyn sensor_common::Read,
        ) -> Result<usize, sensor_common::Error> {
            {
                let _ = &($platform);
                let _ = &($module);
                let _ = &($read);
            };
            $body
        }
        Some(write_fn)
    }};
    (|$platform:ident, $read:ident| $body:expr) => {
        property_write_fn! { |$platform, _t: &mut (), $read| $body }
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
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Meta as u8,
            MetaInformation::Version as u8,
        ],
        type_hint: Some(Type::U64),
        complexity: QueryComplexity::low(),
        description: Some("platform-version"),
        read: property_read_fn! {
            |platform, write| {
                let version: &'static [u8] = env!("CARGO_PKG_VERSION").as_bytes();
                let len = version.len().min(u8::MAX as usize);
                write.write_u8(len as u8)?;
                write.write_all(&version[..usize::from(len)])
            }
        },
        write: None,
    },
    // Property {
    //     id: &[
    //         ComponentRoot::Platform as u8,
    //         PlatformComponent::Meta as u8,
    //         MetaInformation::Module as u8,
    //     ],
    //     type_hint: Some(Type::U64),
    //     complexity: QueryComplexity::low(),
    //     description: Some("platform-module"),
    //     read: property_read_fn! {
    //         |platform, write| {
    //             let name = core::any::type_name::<M>().as_bytes();
    //             let len = name.len().min(u8::MAX as usize);
    //             write.write_u8(len as u8)?;
    //             write.write_all(&name[..usize::from(len)])
    //         }
    //     },
    //     write: None,
    // },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::EeeProm as u8,
            EeePromComponent::MagicCrcStart as u8,
        ],
        type_hint: Some(Type::U8),
        complexity: QueryComplexity::low(),
        description: Some("eeprom-magic-crc"),
        read: property_read_fn! {
            |platform, write| write.write_all(&crate::cnf::MAGIC_EEPROM_CRC_START.to_be_bytes())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Network as u8,
            NetworkComponent::Mac as u8,
        ],
        type_hint: Some(Type::Bytes(6)),
        complexity: QueryComplexity::low(),
        description: Some("network-mac"),
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.network_config.mac.octets())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Network as u8,
            NetworkComponent::Ip as u8,
        ],
        type_hint: Some(Type::Bytes(4)),
        complexity: QueryComplexity::low(),
        description: Some("network-ip"),
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.network_config.ip.octets())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Network as u8,
            NetworkComponent::Subnet as u8,
        ],
        type_hint: Some(Type::Bytes(4)),
        complexity: QueryComplexity::low(),
        description: Some("network-subnet"),
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.network_config.subnet.octets())
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Network as u8,
            NetworkComponent::Gateway as u8,
        ],
        type_hint: Some(Type::Bytes(4)),
        complexity: QueryComplexity::low(),
        description: Some("network-gateway"),
        read: property_read_fn! {
            |platform, write| write.write_all(&platform.network_config.gateway.octets())
        },
        write: None,
    },
    #[cfg(feature = "board-rev-3-0")]
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Temperature as u8,
            TemperatureComponent::Value as u8,
        ],
        type_hint: Some(Type::F32),
        complexity: QueryComplexity::low(),
        description: Some("temp-value"),
        read: property_read_fn! {
            |platform, write| {
                let value = platform
                    .subsystem_i2c
                    .read_temperature_blocking()
                    .unwrap_or(f32::NAN)
                    .to_be_bytes();
                write.write_all(&value)
            }
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Sntp as u8,
            SntpComponent::CurrentTimeMillis as u8,
        ],
        type_hint: Some(Type::U64),
        complexity: QueryComplexity::low(),
        description: Some("sntp-time-millis"),
        read: property_read_fn! {
            |platform, write| {
                let value = platform
                    .sntp
                    .current_time_millis(&platform.system.info)
                    .unwrap_or_default()
                    .to_be_bytes();
                write.write_all(&value)
            }
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Sntp as u8,
            SntpComponent::LastOffsetMillis as u8,
        ],
        type_hint: Some(Type::U64),
        complexity: QueryComplexity::low(),
        description: Some("sntp-offset-millis"),
        read: property_read_fn! {
            |platform, write| {
                let value = platform
                    .sntp
                    .last_offset_millis()
                    .unwrap_or_default()
                    .to_be_bytes();
                write.write_all(&value)
            }
        },
        write: None,
    },
    Property {
        id: &[
            ComponentRoot::Platform as u8,
            PlatformComponent::Sntp as u8,
            SntpComponent::LastUpdateMillis as u8,
        ],
        type_hint: Some(Type::U64),
        complexity: QueryComplexity::low(),
        description: Some("sntp-update-millis"),
        read: property_read_fn! {
            |platform, write| {
                let value = platform
                    .sntp
                    .last_update_millis()
                    .unwrap_or_default()
                    .to_be_bytes();
                write.write_all(&value)
            }
        },
        write: None,
    },
];
