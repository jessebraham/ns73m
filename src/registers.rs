use bitflags::bitflags;

// Register addresses
pub const REG0: u8 = 0x00;
pub const REG1: u8 = 0x01;
pub const REG2: u8 = 0x02;
pub const REG3: u8 = 0x03;
pub const REG4: u8 = 0x04;
pub const REG6: u8 = 0x06;
pub const REG8: u8 = 0x08;
pub const REG14: u8 = 0x0E;

// REG0 bitflags
bitflags! {
    pub struct R0: u8 {
        const PE = 0b0000_0001;
        const PDX = 0b0000_0010;
        const MUTE = 0b0000_0100;
        const EM = 0b0001_0000;
        const EMS = 0b0010_0000;
        const AG = 0b1100_0000;
    }
}

// REG1 bitflags
bitflags! {
    pub struct R1: u8 {
        const PLT = 0b0000_1000;
        const SUBC = 0b0100_0000;
    }
}

// REG2 bitflags
bitflags! {
    pub struct R2: u8 {
        const PL = 0b0000_0011;
        const ULD = 0b0000_0100;
    }
}

// REG6 bitflags
bitflags! {
    pub struct R6: u8 {
        const CIA = 0b0000_0110;
        const CIB = 0b0001_1000;
    }
}

// REG8 bitflags
bitflags! {
    pub struct R8: u8 {
        const CEX = 0b0000_0011;
    }
}

/// Generic operating modes
#[derive(Copy, Clone, Debug)]
pub enum Mode {
    /// Feature is disabled
    Disabled,

    /// Feature is enabled
    Enabled,
}

/// Pre-emphasis values
#[derive(Copy, Clone, Debug)]
pub enum PreEmphasis {
    /// No pre-emphasis
    Disabled,

    /// European country (50μs)
    Pe50us,

    /// Other country, like USA (75μs)
    Pe75us,
}

/// Audio input level values
#[derive(Copy, Clone, Debug)]
pub enum InputLevel {
    /// Low input volume (100mV)
    Low = 0x0,

    /// Medium input volume (140mV)
    Medium = 0x40,

    /// High input volume (200mV)
    High = 0x80,
}

/// Transmit power level values
#[derive(Copy, Clone, Debug)]
pub enum TransmitPowerLevel {
    /// Low power output (0.5mW)
    Low = 0x1,

    /// Medium power output (1.0mW)
    Medium = 0x2,

    /// High power output (2.0mW)
    High = 0x3,
}

/// Charge pump values
#[derive(Copy, Clone, Debug)]
pub enum ChargePump {
    /// Charge pumps at 1.25μA
    Cp1_25uA = 0x1,

    /// Charge pumps at 80μA
    Cp80uA = 0x3,
}

#[derive(Copy, Clone, Debug)]
pub enum FrequencyBand {
    Band0 = 0x0,
    Band1 = 0x1,
    Band2 = 0x2,
    Band3 = 0x3,
}

impl From<u64> for FrequencyBand {
    fn from(frequency: u64) -> Self {
        use FrequencyBand::*;

        if frequency < 88_500_000 {
            Band3
        } else if frequency < 97_900_000 {
            Band2
        } else if frequency < 103_000_000 {
            Band1
        } else {
            Band0
        }
    }
}
