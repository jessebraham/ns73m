//! NS73M Driver

#![no_std]

pub use embedded_hal as hal;

use core::fmt::Debug;

use hal::blocking::delay::DelayUs;
use hal::digital::OutputPin;

mod registers;

/// Generic enabled/disabled modes
#[derive(Copy, Clone, Debug)]
pub enum Mode {
    /// Option is disabled
    Disabled,

    /// Option is enabled
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
    Low,

    /// Medium input volume (140mV)
    Medium,

    /// High input volume (200mV)
    High,
}

/// Transmit power level values
#[derive(Copy, Clone, Debug)]
pub enum TransmitPowerLevel {
    /// Low power output (0.5mW)
    Low,

    /// Medium power output (1.0mW)
    Medium,

    /// High power output (2.0mW)
    High,
}

/// Charge pumps values
#[derive(Copy, Clone, Debug)]
pub enum ChargePump {
    /// Charge pumps at 1.25μA
    Cp1_25uA,

    /// Charge pumps at 80μA
    Cp80uA,
}

/// Driver error types
#[derive(Copy, Clone, Debug)]
pub enum Error<E> {
    /// Output Pin Error
    OutputPinError(E),

    /// Delay Error
    DelayError,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Error<E> {
        Error::OutputPinError(err)
    }
}

/// NS73M driver
pub struct NS73M<'a, P, E, D>
where
    P: OutputPin<Error = E>,
    D: DelayUs<u8>,
{
    /// Clock pin for I/O input
    ck: P,

    /// Data pin I/O for interface
    da: P,

    /// Latch pin for 3-wire or Address pin for I²C
    la: P,

    /// I2C or 3-wire interface selector pin
    iic: P,

    /// Delay peripheral reference
    delay: &'a mut D,

    /// Internal register state
    regs: [u8; 14],
}

impl<'a, P, E, D> NS73M<'a, P, E, D>
where
    E: Debug,
    P: OutputPin<Error = E>,
    D: DelayUs<u8>,
{
    /// Construct a new driver without causing any side-effects.
    pub fn new(ck: P, da: P, la: P, iic: P, delay: &'a mut D) -> Self {
        Self {
            ck,
            da,
            la,
            iic,
            delay,
            regs: [0u8; 14],
        }
    }

    /// Initialize the device.
    pub fn init(&mut self) -> Result<&mut Self, Error<E>> {
        // Reset the internal register state.
        self.reset_registers();

        // Unlatch the transmitter.
        self.la.try_set_low()?;

        // The I2C or the 3-wire bus mode can be selectable by the level of Pin
        // "IIC" as:
        //   "IIC" = HIGH(VDD) -> I2C mode
        //   "IIC" = LOW(0V)   -> 3-wire mode
        self.iic.try_set_low()?;

        Ok(self)
    }

    /// Begin broadcasting.
    pub fn begin(&mut self) -> Result<(), Error<E>> {
        self.reset_registers();
        self.reset()?;

        self.pilot_tone(Mode::Enabled)?;
        self.unlock_detect(Mode::Enabled)?;
        self.forced_subcarrier(Mode::Disabled)?;
        self.power_on()?;
        self.unmute()?;
        self.pre_emphasis(PreEmphasis::Pe50us)?;
        self.reset()?;

        Ok(())
    }

    /// Power the device on.
    pub fn power_on(&mut self) -> Result<(), Error<E>> {
        self.regs[0] |= registers::R0::PE.bits(); // ON = 1
        self.regs[0] &= !registers::R0::PDX.bits(); // ON = 0
        self.send(registers::REG0, self.regs[0])?;

        Ok(())
    }

    /// Power the device off.
    pub fn power_off(&mut self) -> Result<(), Error<E>> {
        self.regs[0] &= !registers::R0::PE.bits(); // OFF = 0
        self.regs[0] |= registers::R0::PDX.bits(); // OFF = 1
        self.send(registers::REG0, self.regs[0])?;

        Ok(())
    }

    /// Mute the device.
    pub fn mute(&mut self) -> Result<(), Error<E>> {
        self.regs[0] |= registers::R0::MUTE.bits();
        self.send(registers::REG0, self.regs[0])?;

        Ok(())
    }

    /// Unmute the device.
    pub fn unmute(&mut self) -> Result<(), Error<E>> {
        self.regs[0] &= !registers::R0::MUTE.bits();
        self.send(registers::REG0, self.regs[0])?;

        Ok(())
    }

    /// Set the pre-emphasis.
    pub fn pre_emphasis(&mut self, pe: PreEmphasis) -> Result<(), Error<E>> {
        match pe {
            PreEmphasis::Disabled => {
                self.regs[0] |= registers::R0::PE.bits(); // OFF = 1
            }
            PreEmphasis::Pe50us => {
                self.regs[0] &= !registers::R0::PE.bits(); // ON = 0
                self.regs[0] &= !registers::R0::EMS.bits(); // 50μs = 0
            }
            PreEmphasis::Pe75us => {
                self.regs[0] &= !registers::R0::PE.bits(); // ON = 0
                self.regs[0] |= registers::R0::EMS.bits(); // 75μs = 0
            }
        }

        self.send(registers::REG0, self.regs[0])?;

        Ok(())
    }

    /// Set the mode for the pilot tone.
    pub fn pilot_tone(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Enabled => {
                self.regs[1] &= !registers::R1::PLT.bits(); // ON = 0
            }
            Mode::Disabled => {
                self.regs[1] |= registers::R1::PLT.bits(); // OFF = 1
            }
        }

        self.send(registers::REG1, self.regs[1])?;

        Ok(())
    }

    /// Force the subcarrier.
    pub fn forced_subcarrier(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Enabled => {
                self.regs[1] &= !registers::R1::SUBC.bits(); // ON = 0
            }
            Mode::Disabled => {
                self.regs[1] |= registers::R1::SUBC.bits(); // OFF = 1
            }
        }

        self.send(registers::REG1, self.regs[1])?;

        Ok(())
    }

    /// Reset the device.
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.send(registers::REG14, 0x05)?;

        Ok(())
    }

    /// Unlock detection.
    pub fn unlock_detect(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Enabled => {
                self.regs[2] |= registers::R2::ULD.bits();
            }
            Mode::Disabled => {
                self.regs[2] &= !registers::R2::ULD.bits();
            }
        }

        self.send(registers::REG2, self.regs[2])?;

        Ok(())
    }

    //
    // private functions
    //

    fn reset_registers(&mut self) {
        self.regs[0] = 0x00;
        self.regs[1] = 0xB4;
        self.regs[2] = 0x05;
        self.regs[3] = 0x8A; // Default frequency : 97.3 Mhz
        self.regs[4] = 0x2E;
        self.regs[6] = 0x1A;
        self.regs[8] = 0x1B;
    }

    fn send(&mut self, reg: u8, data: u8) -> Result<(), Error<E>> {
        self.la.try_set_low()?;

        // Send the register address (4 bits)
        for x in 0..4 {
            let bit_is_set = (reg >> x) & 1 > 0;
            self.send_bit(bit_is_set)?;
        }
        // Send the data (8 bits)
        for x in 0..8 {
            let bit_is_set = (data >> x) & 1 > 0;
            self.send_bit(bit_is_set)?;
        }
        self.delay.try_delay_us(1).map_err(|_| Error::DelayError)?;

        self.la.try_set_high()?;
        self.delay.try_delay_us(4).map_err(|_| Error::DelayError)?;
        self.la.try_set_low()?;

        self.ck.try_set_low()?;

        Ok(())
    }

    fn send_bit(&mut self, bit_is_set: bool) -> Result<(), Error<E>> {
        self.ck.try_set_low()?;
        if bit_is_set {
            self.da.try_set_high()?;
        } else {
            self.da.try_set_low()?;
        }
        self.ck.try_set_high()?;

        Ok(())
    }
}
