use crate::platform::subsystem_spi::SpiBus;
use crate::platform::{DeviceInformation, ErrorFlags};
use w5500::net::Ipv4Addr;
use w5500::udp::UdpSocket;

pub struct Handle {
    pub socket: Option<UdpSocket>,
    pub request_sent: bool,
    pub last_update_millis: Option<u64>,
    pub last_offset_millis: Option<u64>,
}

impl Handle {
    pub const fn new() -> Self {
        Handle {
            socket: None,
            request_sent: false,
            last_update_millis: None,
            last_offset_millis: None,
        }
    }

    pub fn reset_network(&mut self) {
        self.socket = None;
        self.last_update_millis = None;
        self.request_sent = false;
    }

    pub fn set_socket(&mut self, socket: UdpSocket) {
        self.socket = Some(socket);
        self.last_update_millis = None;
    }

    pub fn last_offset_millis(&self) -> Option<u64> {
        self.last_offset_millis
    }

    pub fn current_time_millis(&self, info: &DeviceInformation) -> Option<u64> {
        self.last_offset_millis()
            .map(|millis| millis + info.uptime_ms())
    }

    pub fn update(&mut self, info: &DeviceInformation, spi: &mut SpiBus) -> Result<(), ErrorFlags> {
        const SNTP_FIRST_TRY_OFFSET: u64 = 3_000; // 3s
        const SNTP_UPDATE_INTERVAL: u64 = 10 * 60_000; // 10 min

        if let Some(socket) = self.socket.as_mut() {
            if info.uptime_ms()
                > self
                    .last_update_millis
                    .map(|v| v.saturating_add(SNTP_UPDATE_INTERVAL))
                    .unwrap_or(SNTP_FIRST_TRY_OFFSET)
            {
                // discard any buffered data
                let _ = spi.network_receive_udp(&mut [], socket);
                self.last_update_millis = Some(info.uptime_ms());

                if spi
                    .network_send_udp_to(
                        Ipv4Addr::new(192, 168, 2, 120),
                        123,
                        &protocol::Request.bytes(),
                        socket,
                    )
                    .is_err()
                {
                    return Err(ErrorFlags::SNTP_REQUEST);
                } else {
                    self.request_sent = true;
                }
            }
            if self.request_sent {
                let mut buffer = [0u8; protocol::PACKET_SIZE];
                if let Ok(resp) = spi.network_receive_udp(&mut buffer, socket) {
                    if let Some(..) = resp {
                        self.request_sent = false;
                        if let Ok(timestamp) =
                            protocol::Response::from_bytes_timestamp_only(&buffer)
                        {
                            self.last_offset_millis = Some(timestamp.to_epoch_millis());
                            self.last_update_millis = Some(info.uptime_ms());
                        } else {
                            return Err(ErrorFlags::SNTP_RESPONSE);
                        }
                    }
                } else {
                    return Err(ErrorFlags::SNTP_RESPONSE);
                }
            }
        }
        Ok(())
    }
}

#[allow(unused)]
mod protocol {
    const EPOCH_TIME_OFFSET: u32 = 2_208_988_800;
    pub const PACKET_SIZE: usize = 48;

    pub struct Request;

    impl Request {
        pub fn bytes(&self) -> [u8; PACKET_SIZE] {
            let mut packet = [0u8; PACKET_SIZE];

            // based on https://github.com/risoflora/sntp_request/blob/7e9b74cedddeae8d5e41d95105cfaf86a0b79b2d/src/lib.rs
            // LI (2 bit) - 3 (not in sync), VN (3 bit) - 4 (version),
            // mode (3 bit) - 3 (client)
            // TLDR; I AM A STUPID DEVICE, PLZ SEND SIMPLE TIMESTAMP
            packet[0] = (3 << 6) | (4 << 3) | 3;
            packet
        }
    }

    pub struct Response;

    impl Response {
        pub fn from_bytes_timestamp_only(bytes: &[u8]) -> Result<Timestamp, Error> {
            if bytes.len() < PACKET_SIZE {
                return Err(Error::InvalidPacketSize);
            }

            let hdr = bytes[0];
            if (hdr & 0x38) >> 3 != 4 {
                return Err(Error::WrongSntpServerVersion);
            } else {
                let mode = hdr & 0x7;
                if mode != 4 && mode != 5 {
                    return Err(Error::NoSntpReply);
                }
            }

            if bytes[1] == 0 {
                Err(Error::KissOfDeath)
            } else {
                let mut secs = [0u8; 4];
                let mut frac = [0u8; 4];

                secs.copy_from_slice(&bytes[40..44]);
                frac.copy_from_slice(&bytes[44..48]);

                Ok(Timestamp {
                    secs: u32::from_be_bytes(secs),
                    frac: u32::from_be_bytes(frac),
                })
            }
        }
    }

    pub enum Error {
        InvalidPacketSize,
        WrongSntpServerVersion,
        NoSntpReply,
        KissOfDeath,
    }

    pub struct Timestamp {
        secs: u32,
        frac: u32,
    }

    impl Timestamp {
        /// Seconds since era epoch
        pub fn as_secs(&self) -> u32 {
            self.secs
        }

        /// Fraction of second
        pub fn sec_fracs(&self) -> u32 {
            self.frac
        }

        pub fn to_epoch_millis(&self) -> u64 {
            let mut millis = u64::from(self.secs.saturating_sub(EPOCH_TIME_OFFSET)) * 1_000;
            // millis += (1_000_f32 * (self.frac as f32 / u32::MAX as f32)) as u64;
            millis
        }
    }
}
