pub const SOLAX_MESSAGE_LEN: usize = 9;

#[derive(Default, Copy, Clone)]
pub struct SolaxMessage([u8; SOLAX_MESSAGE_LEN]);

impl SolaxMessage {
    pub const BROADCAST_ADDRESS: [u8; 2] = [0xFF, 0xFF];
    pub const DISCOVER_DEVICES: SolaxMessage = SolaxMessage::new()
        .with_source([0x01, 0x00])
        .with_destination([0x00, 0x00])
        .with_control_code(0x10)
        .with_function_code(0x00)
        .with_data_length(0x00);

    getter_setter!(
        header,
        set_header,
        with_header,
        HEADER_OFFSET = 0,
        HEADER_LEN = 2
    );
    getter_setter!(
        source,
        set_source,
        with_source,
        SOURCE_OFFSET = Self::HEADER_OFFSET + Self::HEADER_LEN,
        SOURCE_LEN = 2
    );
    getter_setter!(
        destination,
        set_destination,
        with_destination,
        DESTINATION_OFFSET = Self::SOURCE_OFFSET + Self::SOURCE_LEN,
        DESTINATION_LEN = 2
    );
    getter_setter!(
        control_code,
        set_control_code,
        with_control_code,
        CONTROL_CODE_OFFSET = Self::DESTINATION_OFFSET + Self::DESTINATION_LEN,
        CONTROL_CODE_LEN = 1
    );
    getter_setter!(
        function_code,
        set_function_code,
        with_function_code,
        FUNCTION_CODE_OFFSET = Self::CONTROL_CODE_OFFSET + Self::CONTROL_CODE_LEN,
        FUNCTION_CODE_LEN = 1
    );
    getter_setter!(
        data_length,
        set_data_length,
        with_data_length,
        DATA_LENGTH_OFFSET = Self::FUNCTION_CODE_OFFSET + Self::FUNCTION_CODE_LEN,
        DATA_LENGTH_LEN = 1
    );

    #[inline]
    pub const fn new() -> Self {
        Self::from([0u8; SOLAX_MESSAGE_LEN])
    }

    #[inline]
    pub const fn from(array: [u8; SOLAX_MESSAGE_LEN]) -> Self {
        Self(array)
    }

    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    #[inline]
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }

    #[inline]
    pub fn into_array(self) -> [u8; SOLAX_MESSAGE_LEN] {
        self.0
    }

    #[inline]
    pub fn checksum_u16(&self, data: &[u8]) -> u16 {
        let mut sum = 0u16;
        for i in 0..self.0.len() {
            sum = sum.wrapping_add(u16::from(self.0[i]));
        }
        for i in 0..data.len() {
            sum = sum.wrapping_add(u16::from(data[i]));
        }
        sum
    }
}
