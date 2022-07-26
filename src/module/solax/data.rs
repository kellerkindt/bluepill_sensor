pub const LIVE_DATA_LEN: usize = 50;

pub struct LiveData([u8; LIVE_DATA_LEN]);

impl LiveData {
    #[inline]
    pub fn from(array: [u8; LIVE_DATA_LEN]) -> Self {
        Self(array)
    }

    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }
}
