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

extern crate onewire;
extern crate pcd8544;
extern crate w5500;
extern crate sensor_common;

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


use stm32f103xx::GPIOC;

use core::fmt::Write;
use core::result::*;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;

use onewire::*;
use pcd8544::*;
use w5500::*;
use sensor_common::*;

use byteorder::ByteOrder;
use byteorder::NetworkEndian;

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
    delay.delay_ms(100_u16);
    rs.set_high();
    delay.delay_ms(500_u16);


    let mut spi = Spi::spi1(
        peripherals.SPI1,
        (sclk, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1.mhz(), // upt to 8mhz?
        clocks,
        &mut rcc.apb2,
    );



    let mut w5500 = W5500::new(spi, cs_w5500).unwrap();

    w5500.set_mac(&MacAddress::new(0x02, 0x00, 0x00, 0x00, 0x01, 0x00)).unwrap();
    w5500.set_ip(&IpAddress::new(192, 168, 3, 223)).unwrap();
    w5500.set_subnet(&IpAddress::new(255, 255, 255, 0)).unwrap();
    w5500.set_gateway(&IpAddress::new(192, 168, 3, 1)).unwrap();


    let _ = w5500.listen_udp(Socket::Socket1, 51);

    let mut wire = OneWire::new(one, false);
    let _ = wire.reset(&mut delay);

    let mut tick = 0_u64;

    loop {
        if tick % 100 == 0 {
            if led.is_low() {
                led.set_high();
            } else {
                led.set_low();
            }
        }

        match handle_udp_requests(&mut wire, &mut delay, &mut w5500, Socket::Socket1, Socket::Socket0, &mut [0u8; 2048]) {
            Err(e) => {
                // writeln!(display, "Error:");
                // writeln!(display, "{:?}", e);
                delay.delay_ms(2_000_u16);
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

fn handle_udp_requests<T: InputPin + OutputPin, S: FullDuplex<u8, Error = spi::Error> + Sized, O: OutputPin>(wire: &mut OneWire<T>, delay: &mut Delay, w5500: &mut W5500<spi::Error, S , O>, socket_rcv: Socket, socket_send: Socket, buffer: &mut [u8]) -> Result<Option<(IpAddress, u16)>, HandleError> {
    if let Some((ip, port, size)) = w5500.try_receive_udp(socket_rcv, buffer)? {
        let (whole_request_buffer, response_buffer) = buffer.split_at_mut(size);

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

             match request {
                 Request::ReadAll(id) | Request::ReadAllOnBus(id, Bus::OneWire) => {
                     Response::Ok(id, Format::AddressValuePairs(Type::Array(8), Type::F32)).write(writer)?;
                     transmit_all_on_one_wire(wire, delay, writer)?;
                 },
                 Request::DiscoverAll(id) | Request::DiscoverAllOnBus(id, Bus::OneWire) => {
                     Response::Ok(id, Format::AddressOnly(Type::Array(8))).write(writer)?;
                     discover_all_on_one_wire(wire, delay, writer)?;
                 },
                 Request::ReadSpecified(id, Bus::OneWire) => {
                     Response::Ok(id, Format::AddressValuePairs(Type::Array(8), Type::F32)).write(writer)?;
                     let ms_till_ready = prepare_requested_on_one_wire(wire, delay, &mut &*request_content_buffer, writer)?;
                     delay.delay_ms(ms_till_ready);
                     transmit_requested_on_one_wire(wire, delay, &mut &*request_content_buffer, writer)?;
                 }
                 _ => {
                     Response::NotImplemented(id).write(writer)?;
                 }
            };

            available - writer.available()
        };

        w5500.send_udp(
            socket_send,
            50,
            &ip,
            port,
            &response_buffer[..response_size]
        )?;
        Ok(Some((ip, port)))
    } else {
        Ok(None)
    }
}

fn transmit_all_on_one_wire<T: InputPin + OutputPin>(wire: &mut OneWire<T>, delay: &mut Delay, writer: &mut sensor_common::Write) -> Result<(), HandleError> {
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

fn discover_all_on_one_wire<T: InputPin + OutputPin>(wire: &mut OneWire<T>, delay: &mut Delay, writer: &mut sensor_common::Write) -> Result<(), HandleError> {
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

fn prepare_requested_on_one_wire<T: InputPin + OutputPin>(wire: &mut OneWire<T>, delay: &mut Delay, reader: &mut sensor_common::Read, writer: &mut sensor_common::Write) -> Result<u16, HandleError> {
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

fn transmit_requested_on_one_wire<T: InputPin + OutputPin>(wire: &mut OneWire<T>, delay: &mut Delay, reader: &mut sensor_common::Read, writer: &mut sensor_common::Write) -> Result<(), HandleError> {
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

fn measure_retry_once_on_crc_error<T: InputPin + OutputPin>(wire: &mut OneWire<T>, device: &Device, delay: &mut Delay) -> f32 {
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

fn prepare_measurement<T: InputPin + OutputPin>(wire: &mut OneWire<T>, device: &Device, delay: &mut Delay) -> Result<u16, HandleError> {
    match device.family_code() {
        ds18b20::FAMILY_CODE => {
            Ok(DS18B20::new(device.clone())?.start_measurement(wire, delay)?)
        },
        _ => Err(HandleError::Unavailable)
    }
}

fn measure<T: InputPin + OutputPin>(wire: &mut OneWire<T>, device: &Device, delay: &mut Delay) -> Result<f32, HandleError> {
    match device.family_code() {
        ds18b20::FAMILY_CODE => {
            Ok(DS18B20::new(device.clone())?.read_measurement(wire, delay)?)
        },
        _ => Err(HandleError::Unavailable)
    }
}

#[derive(Debug)]
enum HandleError {
    Spi(spi::Error),
    Parsing(sensor_common::Error),
    OneWire(onewire::Error),
    Unavailable
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
