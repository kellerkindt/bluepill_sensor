use crate::cnf::NetworkConfiguration;
use crate::ds93c46::DS93C46;
use crate::io_utils::OutputPinInfallible;
use crate::platform::SpiError;
use embedded_hal::blocking::delay::DelayMs;
use embedded_nal::UdpFullStack;
use embedded_nal::{SocketAddr, SocketAddrV4, UdpClientStack};
use stm32f1xx_hal::gpio::gpioa::{PA4, PA5, PA6, PA7};
use stm32f1xx_hal::gpio::gpiob::{PB0, PB1, PB10};
use stm32f1xx_hal::gpio::{Alternate, Floating, Input, Output, PushPull};
use stm32f1xx_hal::pac::SPI1;
use stm32f1xx_hal::spi::{Spi, Spi1NoRemap};
use w5500::bus::{FourWireError, FourWireRef};
use w5500::net::Ipv4Addr;
use w5500::udp::{UdpSocket, UdpSocketError};
use w5500::{
    ArpResponses, ConnectionType, Manual, OnPingRequest, OnWakeOnLan, UninitializedDevice,
};
use w5500::{DeviceRefMut, InactiveDevice};

pub struct SpiBus {
    /// W5500 interrupt pin
    #[allow(unused)]
    #[cfg(feature = "board-rev-2")]
    pub(super) w5500_intr: PB10<Input<Floating>>,
    #[allow(unused)]
    #[cfg(not(feature = "board-rev-2"))]
    pub(super) w5500_intr: PA1<Input<Floating>>,

    //pub(super) w5500: W5500<PB1<Output<PushPull>>>,
    pub(super) w5500: Option<InactiveDevice<w5500::Manual>>,
    pub(super) w5500_cs: PB1<Output<PushPull>>,

    /// EEPROM for configuration
    pub(super) ds93c46: DS93C46<PB0<Output<PushPull>>>,

    pub(super) spi: Spi<
        SPI1,
        Spi1NoRemap,
        (
            PA5<Alternate<PushPull>>,
            PA6<Input<Floating>>,
            PA7<Alternate<PushPull>>,
        ),
        u8,
    >,

    /// Low-Active pin to restart the W5500
    pub(super) w5500_reset: PA4<Output<PushPull>>,
}

impl SpiBus {
    pub fn init_network(
        &mut self,
        delay: &mut impl DelayMs<u16>,
        config: &NetworkConfiguration,
    ) -> Result<(), ()> {
        self.w5500 = None;

        // reset the network chip (timings are really generous)
        self.w5500_reset.set_low_infallible();
        delay.delay_ms(250_u16); // Datasheet 5.5.1 says 500_us to 1_ms (?)
        self.w5500_reset.set_high_infallible();
        delay.delay_ms(250_u16); // Datasheet says read RDY pin, the network chip needs some time to boot!

        self.w5500 = Some(
            UninitializedDevice::new(FourWireRef::new(&mut self.spi, &mut self.w5500_cs))
                .initialize_advanced(
                    config.mac,
                    config.ip,
                    config.gateway,
                    config.subnet,
                    w5500::Mode {
                        on_wake_on_lan: OnWakeOnLan::Ignore,
                        on_ping_request: OnPingRequest::Respond,
                        connection_type: ConnectionType::Ethernet,
                        arp_responses: ArpResponses::Cache,
                    },
                )
                .map_err(drop)?
                .deactivate()
                .1,
        );

        Ok(())
    }

    pub fn network(
        &mut self,
    ) -> Option<
        DeviceRefMut<
            FourWireRef<
                Spi<
                    SPI1,
                    Spi1NoRemap,
                    (
                        PA5<Alternate<PushPull>>,
                        PA6<Input<Floating>>,
                        PA7<Alternate<PushPull>>,
                    ),
                    u8,
                >,
                PB1<Output<PushPull>>,
            >,
            Manual,
        >,
    > {
        Some(
            self.w5500
                .as_mut()?
                .activate_ref(FourWireRef::new(&mut self.spi, &mut self.w5500_cs)),
        )
    }

    pub fn network_send_udp_to(
        &mut self,
        ip: Ipv4Addr,
        port: u16,
        data: &[u8],
        socket: &mut UdpSocket,
    ) -> Result<(), SpiError> {
        if let Some(device) = self.w5500.as_mut() {
            block!(device
                .activate_ref(FourWireRef::new(&mut self.spi, &mut self.w5500_cs))
                .send_to(socket, SocketAddr::V4(SocketAddrV4::new(ip, port)), data))
            .map_err(|e| match e {
                UdpSocketError::NoMoreSockets
                | UdpSocketError::UnsupportedAddress
                | UdpSocketError::WriteTimeout => SpiError::ModeFault,
                UdpSocketError::Other(e) => match e {
                    FourWireError::TransferError(_)
                    | FourWireError::WriteError(_)
                    | FourWireError::ChipSelectError(_) => SpiError::ModeFault,
                },
            })
        } else {
            Err(SpiError::ModeFault)
        }
    }

    pub fn network_receive_udp(
        &mut self,
        buffer: &mut [u8],
        socket: &mut UdpSocket,
    ) -> Result<Option<(Ipv4Addr, u16, usize)>, SpiError> {
        if let Some(device) = self.w5500.as_mut() {
            return match device
                .activate_ref(FourWireRef::new(&mut self.spi, &mut self.w5500_cs))
                .receive(socket, buffer)
            {
                Ok((_, SocketAddr::V6(_))) => unreachable!(),
                Ok((len, SocketAddr::V4(v4))) => Ok(Some((*v4.ip(), v4.port(), len))),
                Err(nb::Error::WouldBlock) => Ok(None),
                Err(nb::Error::Other(e)) => Err(match e {
                    UdpSocketError::NoMoreSockets
                    | UdpSocketError::UnsupportedAddress
                    | UdpSocketError::WriteTimeout => SpiError::ModeFault,
                    UdpSocketError::Other(e) => match e {
                        FourWireError::TransferError(_)
                        | FourWireError::WriteError(_)
                        | FourWireError::ChipSelectError(_) => SpiError::ModeFault,
                    },
                }),
            };
        } else {
            Ok(None)
        }
    }
}
