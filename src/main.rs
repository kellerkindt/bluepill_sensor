//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---
#![feature(core_intrinsics)]
#![feature(lang_items)]
#![feature(used)]
#![no_std]


extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;

extern crate stm32f103xx;
extern crate stm32f103xx_hal;
extern crate embedded_hal;

extern crate byteorder;

#[macro_use(block)]
extern crate nb;

extern crate onewire;
extern crate pcd8544;
extern crate w5500;
extern crate sensor_common;

mod platform;
mod ds93c46;

use stm32f103xx_hal::prelude::*;
use stm32f103xx_hal::gpio::Output;
use stm32f103xx_hal::gpio::OpenDrain;
use stm32f103xx_hal::prelude::_embedded_hal_digital_OutputPin as OutputPin;
use stm32f103xx_hal::prelude::_embedded_hal_digital_InputPin  as InputPin;
use stm32f103xx_hal::gpio::gpioc::PCx;
use stm32f103xx_hal::delay::Delay;

use stm32f103xx_hal::spi;
use stm32f103xx_hal::spi::Spi;

use embedded_hal::spi::Mode;
use embedded_hal::spi::Phase;
use embedded_hal::spi::Polarity;
use embedded_hal::spi::FullDuplex;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::delay::DelayMs;


use stm32f103xx::GPIOC;

use core::fmt::Write;
use core::result::*;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;

use onewire::*;
use onewire::compute_partial_crc8 as crc8;
use pcd8544::*;
use w5500::*;
use sensor_common::*;

use byteorder::ByteOrder;
use byteorder::NetworkEndian;

use ds93c46::*;
use platform::*;

#[cfg(debug_assertions)]
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(f: F) {
    /*
    static mut STDOUT : Option<hio::HStdout> = None;
    unsafe {
        if STDOUT.is_none() {
            STDOUT = match hio::hstdout() {
                Ok(stdout) => Some(stdout),
                Err(_) => None
            };
        }
        if let Some(ref mut stdout) = STDOUT {
            f(stdout);
        }
    }
    */
}

#[cfg(not(debug_assertions))]
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(_f: F) {
    // do nothing
}

fn main() {

    stdout(|out| writeln!(out, "Hello World!"));

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut peripherals = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();


    let mut speed = 500_u16;

    // rcc.cfgr = rcc.cfgr.sysclk(stm32f103xx_hal::time::Hertz(7_200_000_u32));
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f103xx_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();
    let mut one : PCx<Output<OpenDrain>> = gpioc.pc15.into_open_drain_output(&mut gpioc.crh).downgrade();//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();


    let mut pcd_gnd   = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_light = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_vcc   = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_clk   = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_din   = gpioa.pa8 .into_push_pull_output(&mut gpioa.crh);
    let mut pcd_dc    = gpioa.pa9 .into_push_pull_output(&mut gpioa.crh);
    let mut pcd_ce    = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);
    let mut pcd_rst   = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);


    pcd_gnd  .set_low();
    pcd_light.set_high();
    pcd_vcc  .set_high();

    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

    // SPI1
    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let mut cs_eeprom = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    let mut cs_w5500 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
    let mut rs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let mut self_reset = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
    self_reset.set_high();

    rs.set_low();
    cs_w5500.set_high(); // low active
    cs_eeprom.set_low(); // high active
    delay.delay_ms(250_u16);
    rs.set_high();
    delay.delay_ms(250_u16);


    let mut spi = Spi::spi1(
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



    let mut w5500 = W5500::new(&mut spi, &mut cs_w5500);
    let mut ds93c46 = DS93C46::new(&mut cs_eeprom);

    /*

    let mut configuration = match NetworkConfiguration::from(&mut ds93c46, &mut spi, &mut delay) {
        Ok(configuration) => {
            /*
            loop {
                delay.delay_ms(500_u16);
                if led.is_low() {
                    led.set_high();
                } else {
                    led.set_low();
                }
            }*/
            configuration
        },
        Err(e) => {
            let conf = NetworkConfiguration::default();
            let result = conf.write(&mut ds93c46, &mut spi, &mut delay);
            if result.is_err() {
                loop {
                    delay.delay_ms(1500_u16);
                    if led.is_low() {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                }
            }
            if let HandleError::CrcError = e {
                loop {
                    delay.delay_ms(2000_u16);
                    if led.is_low() {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                }
            }
            conf
        }
    };



*/

    let mut wire = OneWire::new(&mut one, false);

    let mut platform = Platform {
        delay:   &mut delay,
        onewire: &mut wire,
        spi:     &mut spi,

        network:        &mut w5500,
        network_reset:  &mut rs,
        network_config: NetworkConfiguration::default(),

        eeprom:        &mut ds93c46,

        reset: &mut self_reset,
    };

    // TODO error handling
    if platform.load_network_configuration().is_err() {
        for _ in 0..4 {
            platform.delay.delay_ms(1000_u16);
            if led.is_low() {
                led.set_high();
            } else {
                led.set_low();
            }
        }
    }
    platform.delay.delay_ms(25_u16);
    let _ = platform.init_network();
    let _ = platform.init_onewire();


    let mut tick = 0_u64;

    loop {
        if tick % 100 == 0 {
            if led.is_low() {
                led.set_high();
            } else {
                led.set_low();
            }
        }

        match handle_udp_requests(&mut platform, &mut [0u8; 2048]) {
            Err(e) => {
                // writeln!(display, "Error:");
                // writeln!(display, "{:?}", e);
                platform.delay.delay_ms(2_000_u16);
            },
            Ok(address) => if let Some((ip, port)) = address {
                // writeln!(display, "Fine:");
                // writeln!(display, "{}:{}", ip, port);
            }
        };

        // delay.delay_ms(100_u16);
        tick += 1;
    }
}
fn handle_udp_requests(platform: &mut Platform, buffer: &mut [u8]) -> Result<Option<(IpAddress, u16)>, HandleError> {
    if let Some((ip, port, size)) = platform.receive_udp(buffer)? {
        let (whole_request_buffer, response_buffer) = buffer.split_at_mut(size);

        let mut reset = false;
        let response_size = {

            let writer = &mut &mut *response_buffer as &mut ::sensor_common::Write;

            let available = writer.available();

            let (request, request_length) = {
                let reader = &mut &*whole_request_buffer as &mut Read;
                let available = reader.available();
                (Request::read(reader)?, available - reader.available())
            };

            let id = request.id();
            let (_request_header_buffer, request_content_buffer) = whole_request_buffer.split_at_mut(request_length);

            reset = handle_udp_requests_legacy(
                id,
                request,
                &mut platform.network_config,
                platform.reset,
                platform.eeprom,
                platform.onewire,
                platform.delay,
                platform.spi,
                request_content_buffer,
                writer
            )?;

            available - writer.available()
        };

        platform.send_udp(
            &ip,
            port,
            &response_buffer[..response_size]
        )?;

        if reset {
            // increase possibility that packet is out
            platform.delay.delay_ms(100_u16);
            let _ = platform.load_network_configuration();
            // platform.reset();
        }
        Ok(Some((ip, port)))
    } else {
        Ok(None)
    }
}

fn handle_udp_requests_legacy(
    id: u8,
    request: Request, net_conf: &mut NetworkConfiguration, self_reset: &mut OutputPin,
    eeprom: &mut DS93C46, wire: &mut OneWire, delay: &mut Delay,
    spi: &mut FullDuplex<u8, Error=spi::Error>,
    request_content_buffer: &[u8], writer: &mut ::sensor_common::Write) -> Result<bool, HandleError> {

    let mut reset = false;

    match request {
         Request::ReadAll(id) | Request::ReadAllOnBus(id, Bus::OneWire) => {
             Response::Ok(id, Format::AddressValuePairs(Type::Bytes(8), Type::F32)).write(writer)?;
             transmit_all_on_one_wire(wire, delay, writer)?;
         },
         Request::DiscoverAll(id) | Request::DiscoverAllOnBus(id, Bus::OneWire) => {
             Response::Ok(id, Format::AddressOnly(Type::Bytes(8))).write(writer)?;
             discover_all_on_one_wire(wire, delay, writer)?;
         },
         Request::ReadSpecified(id, Bus::OneWire) => {
             Response::Ok(id, Format::AddressValuePairs(Type::Bytes(8), Type::F32)).write(writer)?;
             let ms_till_ready = prepare_requested_on_one_wire(wire, delay, &mut &*request_content_buffer, writer)?;
             delay.delay_ms(ms_till_ready);
             transmit_requested_on_one_wire(wire, delay, &mut &*request_content_buffer, writer)?;
         },

         Request::SetNetworkMac(id, mac) => {
             net_conf.mac.address.copy_from_slice(&mac);
             net_conf.write(eeprom, spi, delay)?;
             Response::Ok(id, Format::Empty).write(writer)?;
             reset = true;
         },
         Request::SetNetworkIpSubnetGateway(id, ip, subnet, gateway) => {
             net_conf.ip.address.copy_from_slice(&ip);
             net_conf.subnet.address.copy_from_slice(&subnet);
             net_conf.gateway.address.copy_from_slice(&gateway);
             net_conf.write(eeprom, spi, delay)?;
             Response::Ok(id, Format::Empty).write(writer)?;
             reset = true;
         },

         Request::RetrieveNetworkConfiguration(id) => {
             Response::Ok(id, Format::ValueOnly(Type::Bytes(6 + 3*4))).write(writer)?;
             writer.write_all(&net_conf.mac.address)?;
             writer.write_all(&net_conf.ip.address)?;
             writer.write_all(&net_conf.subnet.address)?;
             writer.write_all(&net_conf.gateway.address)?;
         },

         Request::RetrieveVersionInformation(id) => {
             let version : &'static [u8] = env!("CARGO_PKG_VERSION").as_bytes();
             let len = version.len() as u8;
             Response::Ok(id, Format::ValueOnly(Type::String(len))).write(writer)?;
             writer.write_all(&version[..len as usize])?;
         },

         _ => {
             Response::NotImplemented(id).write(writer)?;
         }
    };
    Ok(reset)
}

fn transmit_all_on_one_wire(wire: &mut OneWire, delay: &mut Delay, writer: &mut sensor_common::Write) -> Result<(), HandleError> {
    let mut search = DeviceSearch::new();
    while writer.available() >= 12 {
        if let Some(device) = wire.search_next(&mut search, delay)? {

            let value = measure_retry_once_on_crc_error(wire, &device, delay);

            let mut buffer = [0u8; 4];
            NetworkEndian::write_f32(&mut buffer[..], value);

            writer.write_all(&device.address)?;
            writer.write_all(&buffer)?;

        } else {
            return Ok(());
        }
    }
    Ok(())
}

fn discover_all_on_one_wire(wire: &mut OneWire, delay: &mut Delay, writer: &mut sensor_common::Write) -> Result<(), HandleError> {
    let mut search = DeviceSearch::new();
    while writer.available() >= ADDRESS_BYTES as usize {
        if let Some(device) = wire.search_next(&mut search, delay)? {
            writer.write_all(&device.address)?;

        } else {
            return Ok(());
        }
    }
    Ok(())
}

fn prepare_requested_on_one_wire(wire: &mut OneWire, delay: &mut Delay, reader: &mut sensor_common::Read, writer: &mut sensor_common::Write) -> Result<u16, HandleError> {
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
            ]
        };

        ms_to_sleep = prepare_measurement(wire, &device, delay)?.max(ms_to_sleep);
    }
    Ok(ms_to_sleep)
}

fn transmit_requested_on_one_wire(wire: &mut OneWire, delay: &mut Delay, reader: &mut sensor_common::Read, writer: &mut sensor_common::Write) -> Result<(), HandleError> {
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
            ]
        };

        let value = measure_retry_once_on_crc_error(wire, &device, delay);

        let mut buffer = [0u8; 4];
        NetworkEndian::write_f32(&mut buffer[..], value);

        writer.write_all(&device.address)?;
        writer.write_all(&buffer)?;
    }
    Ok(())
}

fn measure_retry_once_on_crc_error(wire: &mut OneWire, device: &Device, delay: &mut Delay) -> f32 {
    match measure(wire, device, delay) {
        Ok(value) => value,
        Err(HandleError::OneWire(onewire::Error::CrcMismatch(_, _))) => {
            match measure(wire, &device, delay) {
                Ok(value) => value,
                _ => core::f32::NAN,
            }
        },
        _ => core::f32::NAN,
    }
}

fn prepare_measurement(wire: &mut OneWire, device: &Device, delay: &mut Delay) -> Result<u16, HandleError> {
    match device.family_code() {
        ds18b20::FAMILY_CODE => {
            Ok(DS18B20::new(device.clone())?.start_measurement(wire, delay)?)
        },
        _ => Err(HandleError::Unavailable)
    }
}

fn measure(wire: &mut OneWire, device: &Device, delay: &mut Delay) -> Result<f32, HandleError> {
    match device.family_code() {
        ds18b20::FAMILY_CODE => {
            Ok(DS18B20::new(device.clone())?.read_measurement(wire, delay)?)
        },
        _ => Err(HandleError::Unavailable)
    }
}

#[derive(Debug)]
pub enum HandleError {
    Spi(spi::Error),
    Parsing(sensor_common::Error),
    OneWire(onewire::Error),
    Unavailable,
    NotMagicCrcAtStart,
    CrcError
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

impl From<onewire::Error> for HandleError {
    fn from(e: onewire::Error) -> Self {
        HandleError::OneWire(e)
    }
}


// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    // asm::bkpt();
}

const MAGIC_EEPROM_CRC_START : u8 = 0x48;

pub struct NetworkConfiguration {
    mac: MacAddress,
    ip: IpAddress,
    subnet: IpAddress,
    gateway: IpAddress,
}

impl NetworkConfiguration {
    fn from<S: FullDuplex<u8, Error=spi::Error>>(eeprom: &mut DS93C46, spi: &mut S, delay: &mut DelayMs<u16>) -> Result<NetworkConfiguration, HandleError> {
        let mut configuration = NetworkConfiguration::default();
        configuration.load(eeprom, spi, delay)?;
        Ok(configuration)
    }

    pub fn load(&mut self, eeprom: &mut DS93C46, spi: &mut FullDuplex<u8, Error=spi::Error>, delay: &mut DelayMs<u16>) -> Result<(), HandleError> {
        let mut buf = [0u8; 6 + 3*4 + 2];
        eeprom.read(spi, delay, 0x00, &mut buf)?;
        if buf[0] != MAGIC_EEPROM_CRC_START {
            Err(HandleError::NotMagicCrcAtStart)
        } else {
            let crc = crc8(MAGIC_EEPROM_CRC_START, &buf[1..19]);
            if crc != buf[19] {
                Err(HandleError::CrcError)
            } else {
                self.mac    .address.copy_from_slice(&buf[ 1..7]);
                self.ip     .address.copy_from_slice(&buf[ 7..11]);
                self.subnet .address.copy_from_slice(&buf[11..15]);
                self.gateway.address.copy_from_slice(&buf[15..19]);
                Ok(())
            }
        }
    }

    fn write(&self, eeprom: &mut DS93C46, spi: &mut FullDuplex<u8, Error=spi::Error>, delay: &mut DelayMs<u16>) -> Result<(), HandleError> {
        let mut buf = [0u8; 6 + 3*4 + 2];
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