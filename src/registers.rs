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

// R0 bitflags
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

// R1 bitflags
bitflags! {
    pub struct R1: u8 {
        const PLT = 0b0000_1000;
        const SUBC = 0b0100_0000;
    }
}

// R2 bitflags
bitflags! {
    pub struct R2: u8 {
        const PL = 0b0000_0011;
        const ULD = 0b0000_0100;
    }
}

// R6 bitflags
bitflags! {
    pub struct R6: u8 {
        const CIA = 0b0000_0110;
        const CIB = 0b0001_1000;
    }
}

// R8 bitflags
bitflags! {
    pub struct R8: u8 {
        const CEX = 0b0000_0011;
    }
}
