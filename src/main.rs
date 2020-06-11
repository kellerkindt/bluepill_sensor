#![no_std]
#![no_main]

extern crate cortex_m;
//#[macro_use(entry, exception)]
#[macro_use(entry)]
extern crate cortex_m_rt;
//extern crate panic_abort;
extern crate panic_halt;

extern crate embedded_hal;
extern crate stm32f1xx_hal;

extern crate byteorder;
extern crate void;

//#[macro_use(block_while)]
#[macro_use(block)]
extern crate nb;

extern crate ads1x1x;
extern crate onewire;
// extern crate pcd8544;
extern crate sensor_common;
extern crate w5500;

// mod am2302;
mod ds93c46;
mod io_utils;
mod module;
mod platform;
mod system;
// mod sht1x;

use crate::io_utils::*;
use crate::module::ecm::{ElectricCounterModule, LongTimeFreqMeasurement};
use crate::module::RequestHandler;
use crate::system::System;
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use core::convert::Infallible;
use core::result::*;
use ds93c46::*;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use embedded_hal::spi::Mode;
use embedded_hal::spi::Phase;
use embedded_hal::spi::Polarity;
use nb::Error as NbError;
use onewire::OneWire;
use onewire::{compute_partial_crc8 as crc8, Device};
use platform::*;
use sensor_common::*;
use stm32f1xx_hal::gpio::gpioa::PA5;
use stm32f1xx_hal::gpio::gpioa::PA6;
use stm32f1xx_hal::gpio::gpioa::PA7;
use stm32f1xx_hal::gpio::Alternate;
use stm32f1xx_hal::gpio::Floating;
use stm32f1xx_hal::gpio::Input;
use stm32f1xx_hal::gpio::PushPull;
use stm32f1xx_hal::i2c::Error as I2cError;
use stm32f1xx_hal::pac::SPI1;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::spi;
use stm32f1xx_hal::spi::Spi;
use stm32f1xx_hal::spi::Spi1NoRemap;
use w5500::*;

/*
#[macro_export]
macro_rules! block_while {
    ($c:expr, $e:expr) => {
        loop {
            #[allow(unreachable_patterns)]
            match $e {
                Err(nb::Error::Other(e)) => {
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
*/

#[entry]
fn main() -> ! {
    let mut cp: cortex_m::Peripherals = cortex_m::Peripherals::take().unwrap();
    let peripherals = stm32f1xx_hal::pac::Peripherals::take().unwrap();

    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f1xx_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    /*
    let mut pcd_gnd = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_light = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_vcc = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_clk = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_din = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    let mut pcd_dc = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
    let mut pcd_ce = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);
    let mut pcd_rst = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);

    pcd_gnd.set_low();
    pcd_light.set_high();
    pcd_vcc.set_high();
    */

    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

    // SPI1
    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // DWT does not count without a debugger connected and without
    // this workaround, see https://github.com/japaric/stm32f103xx-hal/issues/76
    // unsafe { *(0xE000EDFC as *mut u32) |= 0x01000000; }
    // unsafe { cp.DCB.demcr.modify(|w| w | (0x01 << 24)); }

    // let countdown = ::stm32f1xx_hal::timer::Timer::syst(cp.SYST, 1.hz(), clocks);

    // TODO
    // let mut onewire_2 = gpiob.pb5.into_open_drain_output(&mut gpiob.crl);
    // let mut onewire_2 = OneWire::new(&mut onewire_2, false);

    // let mut am2302_00 = gpioa.pa12.into_open_drain_output(&mut gpioa.crh);
    // let mut am2302_01 = gpioa.pa11.into_open_drain_output(&mut gpioa.crh);
    // let mut am2302_02 = gpioa.pa10.into_open_drain_output(&mut gpioa.crh);
    // let mut am2302_03 = gpioa.pa9.into_open_drain_output(&mut gpioa.crh);
    /*
    let mut am2302_04 = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    let mut am2302_05 = gpiob.pb15.into_open_drain_output(&mut gpiob.crh);
    let mut am2302_06 = gpiob.pb14.into_open_drain_output(&mut gpiob.crh);
    let mut am2302_07 = gpiob.pb13.into_open_drain_output(&mut gpiob.crh);
    let mut am2302_08 = gpiob.pb12.into_open_drain_output(&mut gpiob.crh);
    */

    /*
    let (mut usart_tx, mut usart_rx) = Serial::usart1(
        peripherals.USART1,
        (
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
            gpioa.pa10.into_floating_input(&mut gpioa.crh),
        ),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    )
    .split();
    */

    /*
    let i2c: BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)> =
        BlockingI2c::i2c1(
            peripherals.I2C1,
            (
                gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl),
                gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl),
            ),
            &mut afio.mapr,
            i2c::Mode::Standard {
                frequency: 10.khz().into(),
            },
            clocks,
            &mut rcc.apb1,
            1_000,
            2,
            1_000,
            10_000,
        );
    */

    let mut spi: Spi<
        SPI1,
        Spi1NoRemap,
        (
            PA5<Alternate<PushPull>>,
            PA6<Input<Floating>>,
            PA7<Alternate<PushPull>>,
        ),
    > = Spi::spi1(
        peripherals.SPI1,
        (sclk, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1.mhz(), // upt to 8mhz for w5500 module, 2mhz is max for eeprom in 3.3V
        clocks,
        &mut rcc.apb2,
    );

    let mut w5500_reset = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let mut platform = Platform {
        board_reset: {
            let mut self_reset = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
            self_reset.set_high_infallible();
            self_reset
        },

        w5500_intr: gpiob.pb10.into_floating_input(&mut gpiob.crh),
        w5500: {
            let mut w5500_cs = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

            w5500_cs.set_low_infallible(); // deselect

            w5500_reset.set_high_infallible(); // request reset
            delay.delay_ms(250_u16);
            w5500_reset.set_low_infallible(); // allow boot

            W5500::with_initialisation(
                w5500_cs,
                &mut spi,
                OnWakeOnLan::Ignore,
                OnPingRequest::Respond,
                ConnectionType::Ethernet,
                ArpResponses::Cache,
            )
            .unwrap()
        },

        spi,

        ds93c46: {
            let mut ds93c46_cs = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
            ds93c46_cs.set_low_infallible(); // deselect
            DS93C46::new(ds93c46_cs)
        },

        w5500_reset,

        led_red: gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
        led_yellow: gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
        led_blue: gpioa.pa3.into_push_pull_output(&mut gpioa.crl),

        factory_reset: gpioa.pa0.into_open_drain_output(&mut gpioa.crl),

        onewire: OneWire::new(gpioc.pc15.into_open_drain_output(&mut gpioc.crh), false),

        network_config: NetworkConfiguration::default(),
        network_udp: None,

        system: {
            let timer = {
                cp.DCB.enable_trace();
                ::stm32f1xx_hal::time::MonoTimer::new(cp.DWT, clocks)
            };

            System {
                info: DeviceInformation::new(&timer, cp.CPUID.base.read()),
                delay,
                timer,
                led_status: gpioc.pc13.into_push_pull_output(&mut gpioc.crh),
            }
        },
    };

    // TODO error handling
    if platform.load_network_configuration().is_err() {
        for _ in 0..4 {
            platform.system.delay.delay_ms(1000_u16);
            if platform.system.led_status.is_set_low().unwrap() {
                platform.system.led_status.set_high_infallible();
            } else {
                platform.system.led_status.set_low_infallible();
            }
        }
    }

    platform.w5500_reset.set_low_infallible();
    platform.system.delay.delay_ms(250_u16);
    platform.w5500_reset.set_high_infallible();
    platform.system.delay.delay_ms(25_u16); // the network chip needs some time to boot!

    let _ = platform.init_network();
    let _ = platform.init_onewire();

    let mut tick = 0_u64;

    let mut ecc = ElectricCounterModule {
        pa8: gpioa.pa8.into_pull_up_input(&mut gpioa.crh),
        pa12: gpioa.pa12.into_floating_input(&mut gpioa.crh),
        pa15: pa15.into_floating_input(&mut gpioa.crh),
        pb3: pb3.into_floating_input(&mut gpiob.crl),
        pb4: pb4.into_floating_input(&mut gpiob.crl),

        garage_open_since: None,
        ltfm1: LongTimeFreqMeasurement::new(),
        ltfm2: LongTimeFreqMeasurement::new(),
        ltfm3: LongTimeFreqMeasurement::new(),
        ltfm4: LongTimeFreqMeasurement::new(),
    };

    loop {
        if tick % 100 == 0 {
            if platform.system.led_status.is_set_low().unwrap_or(false) {
                platform.system.led_status.set_high_infallible();
            } else {
                platform.system.led_status.set_low_infallible();
            }
        }

        match handle_udp_requests(
            &mut platform,
            &mut [0u8; 2048],
            Some(&mut ecc), // Option::<&mut Infallible>::None,
        ) {
            Err(_e) => {
                // writeln!(display, "Error:");
                // writeln!(display, "{:?}", e);
                platform.led_yellow.set_high_infallible();
                platform.system.delay.delay_ms(1_000_u16);
            }
            Ok(_address) => {}
        };

        {
            let probe_start = platform.system.timer.now();
            while platform.factory_reset.is_high().unwrap_or(false) {
                // pressed for longer than 3s?
                if (probe_start.elapsed() / platform.system.timer.frequency().0) > 3 {
                    platform.network_config = NetworkConfiguration::default();
                    let _ = platform.save_network_configuration();
                    while platform.factory_reset.is_high().unwrap_or(false) {
                        platform.led_blue.set_high_infallible();
                        platform.led_yellow.set_high_infallible();
                        platform.led_red.set_high_infallible();
                    }
                    platform.reset();
                }
            }
        }

        ecc.update(platform.system.info.uptime_us());

        // delay.delay_ms(100_u16);
        tick += 1;
        platform.system.info.update_uptime_offset();
    }
}

fn handle_udp_requests(
    platform: &mut Platform,
    buffer: &mut [u8],
    module: Option<&mut impl RequestHandler>,
) -> Result<(), HandleError> {
    if let Some((ip, port, size)) = platform.receive_udp(buffer)? {
        platform.led_red.set_high_infallible();
        let (whole_request_buffer, response_buffer) = buffer.split_at_mut(size);
        let writer = &mut &mut *response_buffer;
        let available = writer.available();

        let (response_size, action, request_id) = {
            let (request, request_length) = {
                let reader = &mut &*whole_request_buffer;
                let available = reader.available();
                (Request::read(reader)?, available - reader.available())
            };

            let id = request.id();
            let (_request_header_buffer, request_content_buffer) =
                whole_request_buffer.split_at_mut(request_length);

            platform.led_blue.set_low_infallible();
            platform.led_yellow.set_low_infallible();

            let mut action =
                platform.try_handle_request(request, &mut &*request_content_buffer, writer);

            if let Some(module) = module {
                if let Ok(Action::HandleRequest(request)) = action {
                    action = module.try_handle_request(
                        platform,
                        request,
                        &mut &*request_content_buffer,
                        writer,
                    );
                }
            }

            (available - writer.available(), action, id)
        };

        let reset = match action {
            Ok(Action::SendResponse) => {
                platform.send_udp(&ip, port, &response_buffer[..response_size])?;
                platform.led_yellow.set_high_infallible();
                false
            }
            Ok(Action::SendResponseAndReset) => {
                platform.send_udp(&ip, port, &response_buffer[..response_size])?;
                true
            }
            Ok(Action::HandleRequest(request)) => {
                Response::NotImplemented(request.id()).write(writer)?;
                let response_size = available - writer.available();
                platform.send_udp(&ip, port, &response_buffer[..response_size])?;
                false
            }
            Err(e) => {
                let to_skip = available - writer.available();
                Response::NotAvailable(request_id).write(writer)?;
                let response_size = available - writer.available();
                platform.send_udp(&ip, port, &response_buffer[to_skip..response_size])?;
                return Err(e)?;
            }
        };

        if reset {
            // increase possibility that packet is out
            platform.system.delay.delay_ms(100_u16);
            let _ = platform.load_network_configuration();
            platform.reset();
        }
    }
    Ok(())
}

fn prepare_requested_on_one_wire(
    platform: &mut Platform,
    reader: &mut impl sensor_common::Read,
    writer: &mut impl sensor_common::Write,
) -> Result<u16, HandleError> {
    let mut ms_to_sleep = 0_u16;
    while reader.available() >= onewire::ADDRESS_BYTES as usize && writer.available() > 12 {
        let device = Device {
            address: [
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
            ],
        };

        ms_to_sleep = platform
            .onewire_prepare_read(&device)
            .unwrap_or(0)
            .max(ms_to_sleep);
    }
    Ok(ms_to_sleep)
}

fn transmit_requested_on_one_wire(
    platform: &mut Platform,
    reader: &mut impl sensor_common::Read,
    writer: &mut impl sensor_common::Write,
) -> Result<(), HandleError> {
    while reader.available() >= onewire::ADDRESS_BYTES as usize && writer.available() > 12 {
        let device = Device {
            address: [
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
                reader.read_u8()?,
            ],
        };

        let value = platform
            .onewire_read(&device, 1)
            .unwrap_or(::core::f32::NAN);

        let mut buffer = [0u8; 4];
        NetworkEndian::write_f32(&mut buffer[..], value);

        writer.write_all(&device.address)?;
        writer.write_all(&buffer)?;
    }
    Ok(())
}

#[derive(Debug)]
pub enum HandleError {
    Unknown,
    Spi(spi::Error),
    Parsing(sensor_common::Error),
    OneWire(onewire::Error<Infallible>),
    Unavailable,
    NotMagicCrcAtStart,
    CrcError,
    I2c(NbError<I2cError>),
    NetworkError(TransferError<spi::Error, Infallible>),
}

impl From<TransferError<spi::Error, Infallible>> for HandleError {
    fn from(e: TransferError<spi::Error, Infallible>) -> Self {
        HandleError::NetworkError(e)
    }
}

impl From<spi::Error> for HandleError {
    fn from(e: spi::Error) -> Self {
        HandleError::Spi(e)
    }
}

impl From<sensor_common::Error> for HandleError {
    fn from(e: sensor_common::Error) -> Self {
        HandleError::Parsing(e)
    }
}

impl From<onewire::Error<Infallible>> for HandleError {
    fn from(e: onewire::Error<Infallible>) -> Self {
        HandleError::OneWire(e)
    }
}

impl From<()> for HandleError {
    fn from(_: ()) -> Self {
        HandleError::Unknown
    }
}

impl From<NbError<I2cError>> for HandleError {
    fn from(e: NbError<I2cError>) -> Self {
        HandleError::I2c(e)
    }
}

const MAGIC_EEPROM_CRC_START: u8 = 0x42;

pub struct NetworkConfiguration {
    mac: MacAddress,
    ip: IpAddress,
    subnet: IpAddress,
    gateway: IpAddress,
}

impl NetworkConfiguration {
    pub fn load(
        &mut self,
        eeprom: &mut DS93C46<impl OutputPin<Error = Infallible>>,
        spi: &mut impl FullDuplex<u8, Error = spi::Error>,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), HandleError> {
        let mut buf = [0u8; 6 + 3 * 4 + 2];
        eeprom.read(spi, delay, 0x00, &mut buf)?;
        if buf[0] != MAGIC_EEPROM_CRC_START {
            Err(HandleError::NotMagicCrcAtStart)
        } else {
            let crc = crc8(MAGIC_EEPROM_CRC_START, &buf[1..19]);
            if crc != buf[19] {
                Err(HandleError::CrcError)
            } else {
                self.mac.address.copy_from_slice(&buf[1..7]);
                self.ip.address.copy_from_slice(&buf[7..11]);
                self.subnet.address.copy_from_slice(&buf[11..15]);
                self.gateway.address.copy_from_slice(&buf[15..19]);
                Ok(())
            }
        }
    }

    fn write(
        &self,
        eeprom: &mut DS93C46<impl OutputPin<Error = Infallible>>,
        spi: &mut impl FullDuplex<u8, Error = spi::Error>,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), HandleError> {
        let mut buf = [0u8; 6 + 3 * 4 + 2];
        buf[0] = MAGIC_EEPROM_CRC_START;
        buf[1..7].copy_from_slice(&self.mac.address);
        buf[7..11].copy_from_slice(&self.ip.address);
        buf[11..15].copy_from_slice(&self.subnet.address);
        buf[15..19].copy_from_slice(&self.gateway.address);
        let crc = onewire::compute_partial_crc8(MAGIC_EEPROM_CRC_START, &buf[1..19]);
        buf[19] = crc;
        eeprom.enable_write(spi, delay)?;
        let result = eeprom.write(spi, delay, 0x00, &buf);
        eeprom.disable_write(spi, delay)?;
        Ok(result?)
    }
}

impl Default for NetworkConfiguration {
    fn default() -> Self {
        NetworkConfiguration {
            mac: MacAddress::new(0x02, 0x00, 0x00, 0x00, 0x01, 0x00),
            ip: IpAddress::new(192, 168, 3, 224),
            subnet: IpAddress::new(255, 255, 255, 0),
            gateway: IpAddress::new(192, 168, 3, 1),
        }
    }
}
