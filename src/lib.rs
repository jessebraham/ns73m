//! `NS73M` FM Transmitter Module Driver
//!
//! The `NS73M` is a small and inexpensive FM transmitter module. It requires
//! only 2 external components, and utilizes a purely digital interface. It is
//! digitally tune-able from 87MHz to 108MHz.

#![no_std]

pub use embedded_hal as hal;

use core::fmt::Debug;

use hal::blocking::delay::DelayUs;
use hal::digital::{OutputPin, PinState};

mod registers;
use registers::*;
pub use registers::{
    ChargePump, InputLevel, Mode, PreEmphasis, TransmitPowerLevel,
};

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
        self.la.try_set_low()?;

        // Reset the internal register state to their default values.
        self.reset_registers();

        // The I2C or the 3-wire bus mode can be selectable by the level of Pin
        // "IIC" as:
        //   "IIC" = HIGH(VDD) -> I2C mode
        //   "IIC" = LOW(0V)   -> 3-wire mode
        self.iic.try_set_low()?;

        Ok(self)
    }

    /// Begin broadcasting.
    pub fn begin(&mut self, frequency: u64) -> Result<(), Error<E>> {
        self.reset_registers();
        self.reset()?;

        self.pilot_tone(Mode::Enabled)?;
        self.transmit_power_level(TransmitPowerLevel::Medium)?;
        self.unlock_detect(Mode::Enabled)?;
        self.forced_subcarrier(Mode::Disabled)?;
        self.tune_to(frequency, false)?;
        self.power_on()?;
        self.unmute()?;
        self.pre_emphasis(PreEmphasis::Pe75us)?;
        self.audio_input_level(InputLevel::Medium)?;
        self.reset()?;

        Ok(())
    }

    /// Tune to the specified frequency.
    pub fn tune_to(
        &mut self,
        frequency: u64,
        reset: bool,
    ) -> Result<(), Error<E>> {
        let band = FrequencyBand::from(frequency);
        self.select_band(band)?;

        let frequency = ((frequency + 304_000) / 8_192) as u8;
        self.regs[3] = frequency & 0xFF;
        self.regs[4] = frequency >> 8;

        self.charge_pump(ChargePump::Cp80uA)?;
        self.send(REG3, self.regs[3])?;
        self.send(REG4, self.regs[4])?;
        self.charge_pump(ChargePump::Cp1_25uA)?;

        if reset {
            self.reset()?;
        }

        Ok(())
    }

    /// Power the device on.
    pub fn power_on(&mut self) -> Result<(), Error<E>> {
        self.regs[0] |= R0::PE.bits(); // ON = 1
        self.regs[0] &= !R0::PDX.bits(); // ON = 0
        self.send(REG0, self.regs[0])?;

        Ok(())
    }

    /// Power the device off.
    pub fn power_off(&mut self) -> Result<(), Error<E>> {
        self.regs[0] &= !R0::PE.bits(); // OFF = 0
        self.regs[0] |= R0::PDX.bits(); // OFF = 1
        self.send(REG0, self.regs[0])?;

        Ok(())
    }

    /// Mute the device.
    pub fn mute(&mut self) -> Result<(), Error<E>> {
        self.regs[0] |= R0::MUTE.bits();
        self.send(REG0, self.regs[0])?;

        Ok(())
    }

    /// Unmute the device.
    pub fn unmute(&mut self) -> Result<(), Error<E>> {
        self.regs[0] &= !R0::MUTE.bits();
        self.send(REG0, self.regs[0])?;

        Ok(())
    }

    /// Set the pre-emphasis.
    pub fn pre_emphasis(&mut self, pre: PreEmphasis) -> Result<(), Error<E>> {
        match pre {
            PreEmphasis::Disabled => {
                self.regs[0] |= R0::PE.bits(); // OFF = 1
            }
            PreEmphasis::Pe50us => {
                self.regs[0] &= !R0::PE.bits(); // ON = 0
                self.regs[0] &= !R0::EMS.bits(); // 50μs = 0
            }
            PreEmphasis::Pe75us => {
                self.regs[0] &= !R0::PE.bits(); // ON = 0
                self.regs[0] |= R0::EMS.bits(); // 75μs = 1
            }
        }
        self.send(REG0, self.regs[0])?;

        Ok(())
    }

    /// Configure the audio input level to get 100% modulation.
    pub fn audio_input_level(
        &mut self,
        level: InputLevel,
    ) -> Result<(), Error<E>> {
        self.regs[0] &= !R0::AG.bits(); // mask off register
        self.regs[0] |= level as u8;
        self.send(REG0, self.regs[0])?;

        Ok(())
    }

    /// Set the mode for the pilot tone.
    pub fn pilot_tone(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Enabled => self.regs[1] &= !R1::PLT.bits(), // ON = 0
            Mode::Disabled => self.regs[1] |= R1::PLT.bits(), // OFF = 1
        }
        self.send(REG1, self.regs[1])?;

        Ok(())
    }

    /// Force the subcarrier.
    pub fn forced_subcarrier(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Enabled => self.regs[1] &= !R1::SUBC.bits(), // ON = 0
            Mode::Disabled => self.regs[1] |= R1::SUBC.bits(), // OFF = 1
        }
        self.send(REG1, self.regs[1])?;

        Ok(())
    }

    /// Reset the device.
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.send(REG14, 0x05)?;

        Ok(())
    }

    /// Set the transmit power level.
    pub fn transmit_power_level(
        &mut self,
        level: TransmitPowerLevel,
    ) -> Result<(), Error<E>> {
        self.regs[2] &= !R2::PL.bits(); // mask off register
        self.regs[2] |= level as u8;
        self.send(REG2, self.regs[2])?;

        Ok(())
    }

    /// Unlock detection.
    pub fn unlock_detect(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Enabled => self.regs[2] |= R2::ULD.bits(), // ON = 1
            Mode::Disabled => self.regs[2] &= !R2::ULD.bits(), // OFF = 0
        }
        self.send(REG2, self.regs[2])?;

        Ok(())
    }

    /// Set the charge pump current and select the clock generator.
    pub fn charge_pump(&mut self, cp: ChargePump) -> Result<(), Error<E>> {
        self.regs[6] &= !R6::CIA.bits(); // mask off register
        self.regs[6] |= cp as u8 | R6::CIB.bits();
        self.send(REG6, self.regs[6])?;

        Ok(())
    }

    //
    // private functions
    //

    fn reset_registers(&mut self) {
        self.regs[0] = 0x00;
        self.regs[1] = 0xB4;
        self.regs[2] = 0x05;
        self.regs[3] = 0x8A; // Default frequency : 97.3MHz
        self.regs[4] = 0x2E;
        self.regs[6] = 0x1A;
        self.regs[8] = 0x1B;
    }

    fn select_band(&mut self, band: FrequencyBand) -> Result<(), Error<E>> {
        self.regs[8] &= !R8::CEX.bits(); // mask off register
        self.regs[8] |= band as u8;
        self.send(REG8, self.regs[8])?;

        Ok(())
    }

    fn send(&mut self, reg: u8, data: u8) -> Result<(), Error<E>> {
        self.la.try_set_low()?;

        // Send the register address (4 bits)
        for x in 0..4 {
            let bit = (reg >> x) & 1 != 0;
            self.send_bit(bit)?;
        }
        // Send the data (8 bits)
        for x in 0..8 {
            let bit = (data >> x) & 1 != 0;
            self.send_bit(bit)?;
        }
        self.delay.try_delay_us(1).map_err(|_| Error::DelayError)?;

        self.la.try_set_high()?;
        self.delay.try_delay_us(4).map_err(|_| Error::DelayError)?;
        self.la.try_set_low()?;

        self.ck.try_set_low()?;

        Ok(())
    }

    fn send_bit(&mut self, bit: bool) -> Result<(), Error<E>> {
        self.ck.try_set_low()?;
        self.da.try_set_state(PinState::from(bit))?;
        self.ck.try_set_high()?;

        Ok(())
    }
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
