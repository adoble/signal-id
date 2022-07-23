#![warn(missing_docs)]
#![no_std]
#![no_main]
//! # Crate signal_id
//!
//! This crate generates on a set of pins a pulse chain that identifies each pin.
//!
//! # Motivation
//!
//! When wiring up on a breadboard this can can result in rats nest of jumper cables.
//! Checking of the pins of a microcontroller have  been correctly wired to any peripherals
//! or other chips (such as display, rotary controllers, level conversion buffers) can be
//! difficult.
//!  
//! Using this crate allows each wire connected to the pins to be checked if it is the correct one
//! by seeing if the  wire has a serial binary signal that matches the pin.
//!
//! # Usage
//!
//! Assuming that you have GPIO pins 0 to 4 wired to other components. The following sets up an array
//! of those pins and then outputs a serial binary signal on each one to identify them.
//!
//!```
//! let d0: DynPin = pins.gpio0.into_push_pull_output().into();
//! let d1: DynPin = pins.gpio1.into_push_pull_output().into();
//! let d2: DynPin = pins.gpio2.into_push_pull_output().into();
//! let d3: DynPin = pins.gpio3.into_push_pull_output().into();
//! let d4: DynPin = pins.gpio4.into_push_pull_output().into();
//!
//! let mut data_pins: [DynPin; 5] = [d0, d1, d2, d3, d4];
//!
//! let mut signal_ids = match SignalIds::new(&mut data_pins) {
//!    Ok(s) => s,
//!    Err(_) => panic!();
//!    };
//!
//! loop {
//!    signal_ids.pulse(&mut delay);
//! }
//! ```
//!
//! The pulses generated have the following characteristics (but these can be changed, see later):
//! * Each pulse has a width of 250 ms.
//! * The space between pulses is 62 ms (the pulse width / 4).
//! * After each pulse chain the signal is set low for 1000ms (250 ms * 4).
//! * The MSB is output first.
//!
//! The default values can be overwritten:
//!
//!```
//! signal_ids
//!     .pulse_width_ms(5)
//!     .pulse_gap_ms(10)
//!     .stop_gap_ms(70);
//!```
//!
//! If using an oscilloscope to view the pulses, one can set a "clock" pin as trigger. This goes high with the first (MSB)
//! and low after the last (LSB) pulse.
//!```
//! let mut clk: DynPin = pins.gpio28.into_push_pull_output().into();
//! signal_ids.clock_pin(&mut clk);
//!```
//!
//! If debugging using an external LED then set the options so that the pulse can be identified with the human eye. For instance
//! ```
//! signal_ids
//!     .pulse_width_ms(500)
//!     .pulse_gap_ms(500)
//!     .stop_gap_ms(2500);
//!```
//!
//! # Limitations
//! * As the array of pins uses DynPin then it can only be used with the HALs for:
//!    + [RP2040](https://docs.rs/rp2040-hal/latest/rp2040_hal/)
//!    + [ATSAMD](https://crates.io/crates/atsamd-hal)  (not tested!)
//! * Times set with the *_us functions are not very accurate.
//!

use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal as hal;

use hal::gpio::DynPin;

enum LogicLevel {
    High,
    Low,
}

/// Used if the pulse width as not been set.
pub const DEFAULT_PULSE_WIDTH: u32 = 250_000; // 250 ms
/// If the the pulse gap has not been configured then the pulse width divided by this is used.
pub const DEFAULT_PULSE_GAP_FACTOR: u32 = 4; // Gaps between pulses are 4 times shorter than the pulse width
/// If the gaps between pulse chains has not been configured then the pulse width times this is used.
pub const DEFAULT_STOP_GAP_FACTOR: u32 = 4; // Gaps between pulse chains are 4 times longer than the pulse width

/// Error type
pub enum SignalIdError {
    /// The number of signals specified is more that 64.
    TooManySignals,
}

/// Configuration of the signal ids.
pub struct SignalIds<'a, 'b> {
    signals: &'a mut [DynPin],
    pulse_count: u16,
    pulse_chain_length: u16,
    // TODO use embedded_time:Duration ?
    pulse_width_us: Option<u32>,
    pulse_gap_us: Option<u32>,
    stop_frame_duration_us: Option<u32>,
    stop_frame_level: LogicLevel,
    clock_pin: Option<&'b mut DynPin>,
}

impl<'a, 'b> SignalIds<'a, 'b> {
    /// Creates a new SignalIds with the specified pins.
    ///
    /// Each pin will have a unique serial binary signal depending on its position in the array.
    ///
    /// If the array of pins is longer than 64 then returns `Err(SignalIdError::TooManySignals)` .
    pub fn new(signals: &'a mut [DynPin]) -> Result<SignalIds<'a, 'b>, SignalIdError> {
        // Calculate how many pulses (i.e. bits) are required to identify all signals.
        // This could have simply been calculated as log2(signals.len()).ceil(), but these
        // functions are only available in the std library! Instead, use this simple
        // look-up approach and restricting the number of pulses to 6 representing up to 64
        // signals (which probably is too many for normal use).
        let pulse_chain_length: u16 = match signals.len() {
            0..=1 => 1,
            2..=4 => 2,
            5..=8 => 3,
            9..=16 => 4,
            17..=32 => 5,
            33..=64 => 6,
            _ => return Err(SignalIdError::TooManySignals),
        };

        Ok(SignalIds {
            signals,
            pulse_count: 0,
            pulse_chain_length,
            pulse_width_us: None,
            pulse_gap_us: None,
            stop_frame_duration_us: None,
            stop_frame_level: LogicLevel::Low,
            clock_pin: None,
        })
    }

    /// Set the width of a pulse in microseconds.
    ///
    /// If not set then the width defaults to DEFAULT_PULSE_WIDTH in **milliseconds**.
    pub fn pulse_width_us(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.pulse_width_us = Some(duration);
        self
    }

    // Set the width of a pulse in millseconds.
    ///
    /// If not set then the width defaults to DEFAULT_PULSE_WIDTH.
    pub fn pulse_width_ms(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.pulse_width_us = Some(duration * 1000);
        self
    }

    /// Set the gap between pulses in microseconds.
    ///
    /// If not set then the gap defaults to pulse_width / DEFAULT_PULSE_GAP_FACTOR.
    pub fn pulse_gap_us(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.pulse_gap_us = Some(duration);
        self
    }

    /// Set the gap between pulses in millseconds.
    ///
    /// If not set then the gap defaults to pulse_width / DEFAULT_PULSE_GAP_FACTOR.
    pub fn pulse_gap_ms(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.pulse_gap_us = Some(duration * 1000);
        self
    }

    /// Set the  gap between each group of pulses in microseconds.
    ///
    /// If not set it defaults to the pulse_width * DEFAULT_STOP_GAP_FACTOR
    pub fn stop_gap_us(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.stop_frame_duration_us = Some(duration);
        self.stop_frame_level = LogicLevel::Low;
        self
    }

    /// The gap between each group of pulses in millseconds
    ///
    /// If not set it defaults to the pulse_width * DEFAULT_STOP_GAP_FACTOR
    pub fn stop_gap_ms(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.stop_frame_duration_us = Some(duration * 1000);
        self.stop_frame_level = LogicLevel::Low;
        self
    }

    /// Sets the signal to high for the specified duration in microseconds
    /// between each chain of pulses.
    pub fn stop_pulse_us(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.stop_frame_duration_us = Some(duration);
        self.stop_frame_level = LogicLevel::High;
        self
    }

    /// Sets the signal to high for the specified duration in milliseconds
    /// between each chain of pulses.
    pub fn stop_pulse_ms(&mut self, duration: u32) -> &mut SignalIds<'a, 'b> {
        self.stop_frame_duration_us = Some(duration * 1000);
        self.stop_frame_level = LogicLevel::High;
        self
    }

    /// Generates a "clock" pulse on a pin that is pulled hign for the duration of the
    /// pulse chain and then low during the stop gap.
    ///
    /// If viewing the pulses with an oscilloscope you may need a trigger so
    /// that the traces remain stable on the screen. This can be used as
    /// that trigger.
    pub fn clock_pin(&mut self, pin: &'b mut DynPin) -> &mut SignalIds<'a, 'b> {
        self.clock_pin = Some(pin);
        self
    }

    /// On each call send on each pin an set of pulses in binary code to
    /// identify the signal.
    pub fn pulse<D: DelayMs<u32> + DelayUs<u32>>(&mut self, delay: &mut D) {
        // let mut shift_div = 1;
        // if self.pulse_count > 0 {
        //     shift_div = self.pulse_count * 2;
        // }

        // If the clock pin has been set and we are at the start of the pulse chain then set it high
        if let Some(clock_pin) = self.clock_pin.as_deref_mut() {
            if self.pulse_count == 0 {
                clock_pin.set_high().unwrap();
            }
        }

        for signal_id in 0..self.signals.len() {
            // Inspect the bits and putput with the msb first
            let shift = self.pulse_chain_length - self.pulse_count - 1;
            if (signal_id >> shift) & 0x0001 == 1 {
                // Is the rightmost bit 1 ..
                self.signals[signal_id].set_high().unwrap();
            } else {
                // .. or 0
                self.signals[signal_id].set_low().unwrap();
            }
        }

        // Leave the pulse set high for its duration
        let pulse_width = self.pulse_width_us.unwrap_or(DEFAULT_PULSE_WIDTH);
        delay.delay_us(pulse_width); // Pulse length defaults to DEFAULT_PULSE_WIDTH. TODO move default to struct

        // Pulse set low
        self.all_low();

        // Pulse gap. If not set then defaults to the pulse width divided by DEFAULT_PULSE_GAP_FACTOR.
        delay.delay_us(
            self.pulse_gap_us
                .unwrap_or(pulse_width / DEFAULT_PULSE_GAP_FACTOR),
        );

        // If the clock pin has been set and we are at the end of the pulse chain then set it low.
        if let Some(clock_pin) = self.clock_pin.as_deref_mut() {
            if self.pulse_count == self.pulse_chain_length - 1 {
                clock_pin.set_low().unwrap();
            }
        }

        // Stop gap or pulse if at the end of the pulse chain. If this has not been set then use the
        // pulse width times the DEFAULT_STOP_GAP_FACTOR
        if self.pulse_count == self.pulse_chain_length - 1 {
            if let Some(duration) = self.stop_frame_duration_us {
                match self.stop_frame_level {
                    LogicLevel::Low => self.all_low(),
                    LogicLevel::High => self.all_high(),
                }
                delay.delay_us(duration);
            } else {
                // Use defaults
                self.all_low();
                delay.delay_us(pulse_width * DEFAULT_STOP_GAP_FACTOR);
            }
        };

        // Decrement the pulse count
        if self.pulse_count == self.pulse_chain_length - 1 {
            self.pulse_count = 0;
        } else {
            self.pulse_count += 1;
        }
    }

    /// Set all pins to low.
    pub fn all_low(&mut self) {
        for pin in self.signals.iter_mut() {
            pin.set_low().unwrap();
        }
    }

    /// Set all pins to high.
    pub fn all_high(&mut self) {
        for pin in self.signals.iter_mut() {
            pin.set_high().unwrap();
        }
    }
}
