use crate::module::{
    Module, ModuleBuilder, ModulePeripherals, PlatformConstraints, RequestHandler,
};
use crate::platform::{Action, DeviceInformation, HandleError, Platform};
use crate::props::Property;
use byteorder::{ByteOrder, NetworkEndian};
use sensor_common::props::ModuleId;
use sensor_common::{Bus, Format, Read, Request, Response, Type, Write};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::serial::{Config, Rx, Serial, Tx};
use stm32f1xx_hal::time::U32Ext;

macro_rules! block_while {
    ($c:expr, $e:expr) => {
        loop {
            #[allow(unreachable_patterns)]
            match $e {
                Err(nb::Error::Other(e)) =>
                {
                    #[allow(unreachable_code)]
                    break Err(nb::Error::Other(e))
                }
                Err(nb::Error::WouldBlock) => {
                    if !$c {
                        break Err(nb::Error::WouldBlock);
                    }
                }
                Ok(x) => break Ok(x),
            }
        }
    };
}

pub struct EnvironmentalModuleBuilder;

impl ModuleBuilder<EnvironmentalModule> for EnvironmentalModuleBuilder {
    fn build(
        _platform: &mut Platform,
        constraints: &mut PlatformConstraints,
        peripherals: ModulePeripherals,
    ) -> EnvironmentalModule {
        let tx = peripherals
            .pin_30
            .into_alternate_push_pull(&mut constraints.gpioa_crh);
        let rx = peripherals.pin_31;
        let serial = Serial::usart1(
            peripherals.usart1,
            (tx, rx),
            &mut constraints.afio.mapr,
            Config::default().baudrate(9600.bps()),
            constraints.clocks,
            &mut constraints.apb2,
        );
        let (tx, rx) = serial.split();
        EnvironmentalModule {
            serial_tx: tx,
            serial_rx: rx,
        }
    }
}

pub struct EnvironmentalModule {
    serial_tx: Tx<USART1>,
    serial_rx: Rx<USART1>,
}

impl EnvironmentalModule {
    pub fn read_co2(&mut self, info: &DeviceInformation) -> Result<f32, ()> {
        use embedded_hal::prelude::*;
        const START_BYTE: u8 = 0xFF;
        const COMMAND: u8 = 0x86;

        let tx = &mut self.serial_tx;
        let rx = &mut self.serial_rx;

        let time = || info.uptime_ms();
        let timeout = time() + 500;
        let in_time = || timeout > time();

        block_while!(in_time(), tx.write(START_BYTE))
            .and(block_while!(in_time(), tx.write(0x01)))
            .and(block_while!(in_time(), tx.write(COMMAND)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x79)))
            .map_err(drop)
            .and_then(|_| {
                Ok([
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                ])
            })
            .and_then(|value| {
                // 0xFF - summed value of 1..=7 + 1
                let checksum: u8 = 0xFF_u8 + (!(&value[1..=7]).iter().sum::<u8>() + 1_u8) + 1_u8;

                if checksum == value[8] && value[0] == START_BYTE && value[1] == COMMAND {
                    let value = NetworkEndian::read_u16(&value[2..4]) as f32;
                    Ok(value)
                } else {
                    Err(())
                }
            })
    }
}

impl Module for EnvironmentalModule {
    type Builder = EnvironmentalModuleBuilder;

    const PROPERTIES: &'static [Property<Self>] = &[Property {
        id: &[0x01],
        name: Some("co2-ppm"),
        ty: Type::F32,
        read: property_read_fn! {
            |platform, env: &mut EnvironmentalModule, write| {
                match env.read_co2(&platform.system.info) {
                    Ok(co2_value) => {
                        write.write_all(&co2_value.to_be_bytes()[..])
                    }
                    Err(_) => {
                        Err(sensor_common::Error::UnexpectedEOF)
                    }
                }
            }
        },
        write: None,
    }];

    fn module_id(&self) -> ModuleId {
        ModuleId {
            group: 0,
            id: 0,
            ext: 0,
        }
    }

    fn update(&mut self, _platform: &mut Platform) {}
}

impl RequestHandler for EnvironmentalModule {
    fn try_handle_request(
        &mut self,
        platform: &mut Platform,
        request: Request,
        _request_payload: &mut impl Read,
        response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        if let Request::ReadSpecified(id, Bus::Custom(130)) = request {
            match self.read_co2(&platform.system.info) {
                Ok(co2_value) => {
                    Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                    response_writer.write_all(&co2_value.to_be_bytes()[..])?;
                }
                Err(_) => {
                    Response::NotAvailable(id).write(response_writer)?;
                }
            }
            Ok(Action::SendResponse)
        } else {
            Ok(Action::HandleRequest(request))
        }
    }
}
