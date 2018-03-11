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
extern crate enc28j60;
extern crate w5500;
extern crate sensor_common;

use stm32f103xx_hal::prelude::*;
use stm32f103xx_hal::prelude::_embedded_hal_digital_OutputPin as OutputPin;
use stm32f103xx_hal::gpio::Output;
use stm32f103xx_hal::gpio::OpenDrain;
use stm32f103xx_hal::gpio::gpioc::PCx;
use stm32f103xx_hal::delay::Delay;
use stm32f103xx_hal::spi::Spi;

use embedded_hal::spi::Mode;
use embedded_hal::spi::Phase;
use embedded_hal::spi::Polarity;
use embedded_hal::blocking::spi;

use stm32f103xx::GPIOC;

use core::fmt::Write;
use core::result::*;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;

use onewire::*;
use pcd8544::*;
use enc28j60::*;
use w5500::*;
use sensor_common::*;

use byteorder::ByteOrder;
use byteorder::LittleEndian;

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

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f103xx_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();
    let mut one : PCx<Output<OpenDrain>> = gpioc.pc14.into_open_drain_output(&mut gpioc.crh).downgrade();//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();


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

    let mut display = PCD8544::new(
        &mut pcd_clk,
        &mut pcd_din,
        &mut pcd_dc,
        &mut pcd_ce,
        &mut pcd_rst,
        &mut pcd_light,
    );

    display.reset();
    display.write_str("Hello World");


    /*
    let mut test = 0u8;
    let mut probe2 = [0u8; 18024];
    let mut probe3 = 0u8;
    display.clear();
    //write!(display, "sizeof: {}", core::mem::size_of::<PCx<Output<OpenDrain>>>());
    write!(display, "address: {}  {}  {}", probe0 as usize - 0x20000000_usize, (&mut "tes" as *mut _) as usize - 0x20000000_usize, (&mut probe2[0] as *mut _) as usize - 0x20000000_usize);
    */
    display.clear();


    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

    // SPI1
    let sck  = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let mut cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let mut rs = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);

    rs.set_low();
    delay.delay_ms(100_u16);
    rs.set_high();
    delay.delay_ms(100_u16);


    let mut spi = Spi::spi1(
        peripherals.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1.mhz(), // upt to 8mhz?
        clocks,
        &mut rcc.apb2,
    );


    // let mut enc = ENC28J60::new(spi, cs);
    // enc.init().unwrap();

    let mut w5500 = W5500::new(spi, cs).unwrap();
    let _ = w5500.listen_udp(Socket::Socket1, 51);

    loop {
        led.set_low();
        delay.delay_ms(speed);
        led.set_high();
        delay.delay_ms(speed);

        let mut wire = OneWire::new(&mut one, false);
        for _ in 0..16 {
            let result = wire.reset(&mut delay);
            stdout(|out| writeln!(out, "reset: {:?}", result));
            // display.clear();
            // write!(display, "{:?}", result);
            if let Ok(ref result) = result {
                if *result {
                    speed = 25_u16;
                } else {
                    speed = 500_u16;
                }
            } else {
                speed = 2_000_u16;
            }
        }
        display.set_light(true);
        display.set_x_position(0);
        display.set_y_position(0);
        writeln!(display, "TmpSensorAddr");
        let mut search = OneWireDeviceSearch::new();
        let addr = match wire.search_next(&mut search, &mut delay) {
            Err(e) => {
                write!(display, "E: {:?}", e);
                None
            },
            Ok(addr) => {
                if let Some(ref addr) = addr {
                    writeln!(display, "{}", addr);
                } else {
                    writeln!(display, "None");
                }
                addr
            }
        };
        if let Some(ref device) = addr {
            if let Ok(ref mut ds18b20) = DS18B20::new(device.clone()) {
                let _measure_resolution = ds18b20.measure_temperature(&mut wire, &mut delay);
                match _measure_resolution {
                    Err(e) => {},
                    Ok(res)=> delay.delay_ms(res.time_ms())
                };
                let temp = ds18b20.read_temperature(&mut wire, &mut delay);

                writeln!(display);
                match temp {
                    Ok(temp) => {
                        writeln!(display, " {:.1}°C", temp);

                        let mut buffer = [0u8; 2048];
                        let result = w5500.try_receive_udp(Socket::Socket1, &mut buffer);
                        match result {
                            Err(e) => {
                                writeln!(display, "{:?}", e);
                            },
                            Ok(option) => if let Some((ip, port, length)) = option {
                                writeln!(display, "{} {}", ip, length);

                                let request = Request::read(&mut &buffer[..length]);
                                match request {
                                    Err(e) => {
                                        writeln!(display, "{:?}", e);
                                    }, // ignore
                                    Ok(request) => {
                                        match request {
                                            Request::ReadAllOnBus(id, bus) if bus == Bus::OneWire => {
                                                let response = Response::Ok(id, Format::AddressValuePairs(Type::Array(8), Type::F32));
                                                let size = response.write(&mut &mut buffer[..]);
                                                if let Ok(size) = size {
                                                    buffer[size + 0] = device.address[0];
                                                    buffer[size + 1] = device.address[1];
                                                    buffer[size + 2] = device.address[2];
                                                    buffer[size + 3] = device.address[3];
                                                    buffer[size + 4] = device.address[4];
                                                    buffer[size + 5] = device.address[5];
                                                    buffer[size + 6] = device.address[6];
                                                    buffer[size + 7] = device.address[7];

                                                    LittleEndian::write_f32(&mut buffer[size+8..], temp);

                                                    let _ = w5500.send_udp(
                                                        Socket::Socket0,
                                                        50,
                                                        &ip,
                                                        port,
                                                        &buffer[..size+12],
                                                    );
                                                }
                                            },

                                            Request::ReadAllOnBus(id, _) | Request::ReadSingle(id) | Request::ReadAll(id) |Request::DiscoverAll(id) |Request::DiscoverAllOnBus(id, _) => {
                                                let response = Response::NotImplemented(id);
                                                let size = response.write(&mut &mut buffer[..]);
                                                if let Ok(size) = size {
                                                    let _ = w5500.send_udp(
                                                        Socket::Socket0,
                                                        50,
                                                        &ip,
                                                        port,
                                                        &buffer[..size],
                                                    );
                                                }
                                            },
                                        };
                                    }
                                };

                                /*
                                let mut buffer = [0u8; 4];
                                LittleEndian::write_f32(&mut buffer, temp);

                                let _ = w5500.send_udp(
                                    Socket::Socket0,
                                    50,
                                    &ip,
                                    5354,
                                    &buffer,
                                );
                                */

                            } else {
                                writeln!(display, "None");
                            }
                        };


                        /*
                        match w5500.get_mac() {
                            Ok(mac) => writeln!(display, "{}", mac),
                            Err(e) =>  writeln!(display, "{:?}", e),
                        };
                        */
                    },
                    Err(err) => {
                        writeln!(display, "E: {:?}", err);
                    }
                };
                //write!(display, " {:.1}°C 0x{:02x}{:02x}", tempf32, content[0], content[1]);
                //write!(display, " {:}  °C 0x{:02x}{:02x}", temp as i16 / 16_i16, content[0], content[1]);
                /*for b in content.iter() {
                    write!(display, "{:02x} ", b);
                }*/
//            delay.delay_ms(1_500_u16);
            } else {
                writeln!(display, "family code mismatch");
            }
        }
        /*
        if let Err(e) = enc.reset() {
            writeln!(display, "1 {:?}", e);
        } else {
            if let Err(e) = enc.test() {
                writeln!(display, "2 {:?}", e);
            } else {
                writeln!(display, "{:?}", enc.send(&[
                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // destination mac address
                    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, // source mac address,
                    0x08, 0x00, // IPv4 Frame
                    /// --- IP
                    /*version*/ 0x40 | /*header length in 32bytes x */0x05,
                    0x00, 0x00, // total IP Frame length TODO
                    0b010_00000 /*dont fragment, last 5 bits fragment offset*/, 0x00, // fragment offset
                    0xFF, // TTL
                    17, // UDP
                    0x00, 0x00, // checksum TODO
                    192, 168, 3, 177, // source address
                    192, 168, 3, 55, // destination address
                    // --- UDP
                ]));
                /*
                match enc.read_mac() {
                    Err(e) => {
                        writeln!(display, "3 {:?}", e);
                    },
                    Ok(addr) => {
                        write!(display, "{:02x}", addr[5]);
                        write!(display, ":{:02x}", addr[4]);
                        write!(display, ":{:02x}", addr[3]);
                        write!(display, ":{:02x}", addr[2]);
                        write!(display, ":{:02x}", addr[1]);
                        writeln!(display, ":{:02x}", addr[0]);
                    }
                };*/
            }
        }*/
    }
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    // asm::bkpt();
}