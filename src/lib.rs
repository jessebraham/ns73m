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
