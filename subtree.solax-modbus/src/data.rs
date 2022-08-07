pub const LIVE_DATA_LEN: usize = 52;

#[derive(Copy, Clone)]
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

    getter_setter!(
        temperature,
        set_tempeature,
        with_temperature,
        TEMPERATURE_OFFSET = 0,
        TEMPERATURE_LEN = 2
    );

    #[inline]
    pub fn temperature_f32(&self) -> f32 {
        u16::from_be_bytes(self.temperature()) as f32
    }

    getter_setter!(
        energy_today,
        set_energy_today,
        with_energy_today,
        ENERGY_TODAY_OFFSET = Self::TEMPERATURE_OFFSET + Self::TEMPERATURE_LEN,
        ENERGY_TODAY_LEN = 2
    );

    #[inline]
    pub fn energy_today_f32(&self) -> f32 {
        u16::from_be_bytes(self.energy_today()) as f32 / 10_f32
    }

    getter_setter!(
        pv1_voltage,
        set_pv1_voltage,
        with_pv1_voltage,
        PV1_VOLTAGE_OFFSET = Self::ENERGY_TODAY_OFFSET + Self::ENERGY_TODAY_LEN,
        PV1_VOLTAGE_LEN = 2
    );

    #[inline]
    pub fn pv1_voltage_f32(&self) -> f32 {
        u16::from_be_bytes(self.pv1_voltage()) as f32 / 10_f32
    }

    getter_setter!(
        pv2_voltage,
        set_pv2_voltage,
        with_pv2_voltage,
        PV2_VOLTAGE_OFFSET = Self::PV1_VOLTAGE_OFFSET + Self::PV1_VOLTAGE_LEN,
        PV2_VOLTAGE_LEN = 2
    );

    #[inline]
    pub fn pv2_voltage_f32(&self) -> f32 {
        u16::from_be_bytes(self.pv2_voltage()) as f32 / 10_f32
    }

    getter_setter!(
        pv1_current,
        set_pv1_current,
        with_pv1_curent,
        PV1_CURRENT_OFFSET = Self::PV2_VOLTAGE_OFFSET + Self::PV2_VOLTAGE_LEN,
        PV1_CURRENT_LEN = 2
    );

    #[inline]
    pub fn pv1_current_f32(&self) -> f32 {
        u16::from_be_bytes(self.pv1_current()) as f32 / 10_f32
    }

    getter_setter!(
        pv2_current,
        set_pv2_current,
        with_pv2_curent,
        PV2_CURRENT_OFFSET = Self::PV1_CURRENT_OFFSET + Self::PV1_CURRENT_LEN,
        PV2_CURRENT_LEN = 2
    );

    #[inline]
    pub fn pv2_current_f32(&self) -> f32 {
        u16::from_be_bytes(self.pv2_current()) as f32 / 10_f32
    }

    getter_setter!(
        ac_current,
        set_ac_current,
        with_ac_current,
        AC_CURRENT_OFFSET = Self::PV2_CURRENT_OFFSET + Self::PV2_CURRENT_LEN,
        AC_CURRENT_LEN = 2
    );

    #[inline]
    pub fn ac_current_f32(&self) -> f32 {
        u16::from_be_bytes(self.ac_current()) as f32 / 10_f32
    }

    getter_setter!(
        ac_voltage,
        set_ac_voltage,
        with_ac_voltage,
        AC_VOLTAGE_OFFSET = Self::AC_CURRENT_OFFSET + Self::AC_CURRENT_LEN,
        AC_VOLTAGE_LEN = 2
    );

    #[inline]
    pub fn ac_voltage_f32(&self) -> f32 {
        u16::from_be_bytes(self.ac_voltage()) as f32 / 10_f32
    }

    getter_setter!(
        ac_frequency,
        set_ac_frequency,
        with_ac_frequency,
        AC_FREQUENCY_OFFSET = Self::AC_VOLTAGE_OFFSET + Self::AC_VOLTAGE_LEN,
        AC_FREQUENCY_LEN = 2
    );

    #[inline]
    pub fn ac_frequency_f32(&self) -> f32 {
        u16::from_be_bytes(self.ac_frequency()) as f32 / 100_f32
    }

    getter_setter!(
        ac_power,
        set_ac_power,
        with_ac_power,
        AC_POWER_OFFSET = Self::AC_FREQUENCY_OFFSET + Self::AC_FREQUENCY_LEN,
        AC_POWER_LEN = 2
    );

    #[inline]
    pub fn ac_power_f32(&self) -> f32 {
        u16::from_be_bytes(self.ac_power()) as f32
    }

    getter_setter!(
        unused,
        set_unused,
        with_unused,
        UNUSED_OFFSET = Self::AC_POWER_OFFSET + Self::AC_POWER_LEN,
        UNUSED_LEN = 2
    );

    getter_setter!(
        energy_total,
        set_energy_total,
        with_energy_total,
        ENERGY_TOTAL_OFFSET = Self::UNUSED_OFFSET + Self::UNUSED_LEN,
        ENERGY_TOTAL_LEN = 4
    );

    #[inline]
    pub fn energy_total_f32(&self) -> f32 {
        u32::from_be_bytes(self.energy_total()) as f32 / 10_f32
    }

    getter_setter!(
        runtime_total,
        set_runtime_total,
        with_runtime_total,
        RUNTIME_TOTAL_OFFSET = Self::ENERGY_TOTAL_OFFSET + Self::ENERGY_TOTAL_LEN,
        RUNTIME_TOTAL_LEN = 4
    );

    #[inline]
    pub fn runtime_total_f32(&self) -> f32 {
        u32::from_be_bytes(self.runtime_total()) as f32
    }

    getter_setter!(
        mode,
        set_mode,
        with_mode,
        MODE_OFFSET = Self::RUNTIME_TOTAL_OFFSET + Self::RUNTIME_TOTAL_LEN,
        MODE_LEN = 2
    );

    #[inline]
    pub fn mode_str(&self) -> Option<&'static str> {
        Some(match u16::from_be_bytes(self.mode()) {
            0 => "wait",
            1 => "check",
            2 => "normal",
            3 => "fault",
            4 => "permanent fault",
            5 => "update",
            6 => "EPS check",
            7 => "EPS",
            8 => "self test",
            9 => "idle",
            _ => return None,
        })
    }

    #[inline]
    pub fn mode_str_or_unknown(&self) -> &'static str {
        self.mode_str().unwrap_or("unknown")
    }

    getter_setter!(
        grid_voltage_fault,
        set_grid_voltage_fault,
        with_grid_voltage_fault,
        GRID_VOLTAGE_FAULT_OFFSET = Self::MODE_OFFSET + Self::MODE_LEN,
        GRID_VOLTAGE_FAULT_LEN = 2
    );

    #[inline]
    pub fn grid_voltage_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.grid_voltage_fault()) as f32 / 10_f32
    }

    getter_setter!(
        grid_frequency_fault,
        set_grid_frequency_fault,
        with_grid_frequency_fault,
        GRID_FREQUENCY_FAULT_OFFSET =
            Self::GRID_VOLTAGE_FAULT_OFFSET + Self::GRID_VOLTAGE_FAULT_LEN,
        GRID_FREQUENCY_FAULT_LEN = 2
    );

    #[inline]
    pub fn grid_frequency_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.grid_frequency_fault()) as f32 / 10_f32
    }

    getter_setter!(
        dc_injection_fault,
        set_dc_injection_fault,
        with_dc_injection_fault,
        DC_INJECTION_FAULT_OFFSET =
            Self::GRID_FREQUENCY_FAULT_OFFSET + Self::GRID_FREQUENCY_FAULT_LEN,
        DC_INJECTION_FAULT_LEN = 2
    );

    #[inline]
    pub fn dc_injection_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.dc_injection_fault()) as f32 / 10_f32
    }

    getter_setter!(
        temperature_fault,
        set_temperature_fault,
        with_temperature_fault,
        TEMPERATURE_FAULT_OFFSET = Self::DC_INJECTION_FAULT_OFFSET + Self::DC_INJECTION_FAULT_LEN,
        TEMPERATURE_FAULT_LEN = 2
    );

    #[inline]
    pub fn temperature_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.temperature_fault()) as f32 / 10_f32
    }

    getter_setter!(
        pv1_voltage_fault,
        set_pv1_voltage_fault,
        with_pv1_voltage_fault,
        PV1_VOLTAGE_FAULT_OFFSET = Self::TEMPERATURE_FAULT_OFFSET + Self::TEMPERATURE_FAULT_LEN,
        PV1_VOLTAGE_FAULT_LEN = 2
    );

    #[inline]
    pub fn pv1_voltage_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.pv1_voltage_fault()) as f32 / 10_f32
    }

    getter_setter!(
        pv2_voltage_fault,
        set_pv2_voltage_fault,
        with_pv2_voltage_fault,
        PV2_VOLTAGE_FAULT_OFFSET = Self::PV1_VOLTAGE_FAULT_OFFSET + Self::PV1_VOLTAGE_FAULT_LEN,
        PV2_VOLTAGE_FAULT_LEN = 2
    );

    #[inline]
    pub fn pv2_voltage_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.pv2_voltage_fault()) as f32 / 10_f32
    }

    getter_setter!(
        gfc_fault,
        set_gfc_fault,
        with_gfc_fault,
        GFC_FAULT_OFFSET = Self::PV2_VOLTAGE_FAULT_OFFSET + Self::PV2_VOLTAGE_FAULT_LEN,
        GFC_FAULT_LEN = 2
    );

    #[inline]
    pub fn gfc_fault_f32(&self) -> f32 {
        u16::from_be_bytes(self.gfc_fault()) as f32 / 10_f32
    }

    getter_setter!(
        error_message,
        set_error_message,
        with_error_message,
        ERROR_MESSAGE_OFFSET = Self::GFC_FAULT_OFFSET + Self::GFC_FAULT_LEN,
        ERROR_MESSAGE_LEN = 4
    );

    getter_setter!(
        unknown,
        set_unknown,
        with_unknown,
        UNKNOWN_OFFSET = Self::ERROR_MESSAGE_OFFSET + Self::ERROR_MESSAGE_LEN,
        UNKNOWN_LEN = 2
    );
}
