use crate::block_while;
use byteorder::ByteOrder;
use byteorder::NetworkEndian;

pub type TimeMillis = u64;

#[derive(Default)]
pub struct Palt {
    pub co2: Option<(f32, TimeMillis)>,
}

impl Palt {
    pub fn update_co2<F>(
        &mut self,
        time: F,
        tx: &mut embedded_hal::serial::Write<u8, Error = void::Void>,
        rx: &mut embedded_hal::serial::Read<u8, Error = stm32f103xx_hal::serial::Error>,
    ) -> Result<f32, ()>
    where
        F: Fn() -> TimeMillis,
    {
        const START_BYTE: u8 = 0xFF;
        const COMMAND: u8 = 0x86;

        let in_time = (|| {
            let time = &time;
            let timeout = time() + 500;
            move || timeout > time()
        })();

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
                    self.co2.update(value, time());
                    Ok(value)
                } else {
                    Err(())
                }
            })
    }
}

pub trait TimedValue {
    fn age(&self, current: TimeMillis) -> TimeMillis;

    fn value(&self) -> Option<f32>;

    fn update(&mut self, value: f32, current: TimeMillis);
}

impl TimedValue for Option<(f32, TimeMillis)> {
    fn age(&self, current: TimeMillis) -> TimeMillis {
        if let Some((_, last_updated)) = self {
            current.overflowing_sub(current).0
        } else {
            current
        }
    }

    fn value(&self) -> Option<f32> {
        self.map(|(value, _)| value)
    }

    fn update(&mut self, value: f32, current: TimeMillis) {
        *self = Some((value, current));
    }
}
