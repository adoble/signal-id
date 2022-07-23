# signal-id


This crate generates on a set of pins a pulse chain that identifies each pin.

## Motivation

When wiring up on a breadboard this can can result in rats nest of jumper cables.
Checking of the pins of a microcontroller have  been correctly wired to any peripherals
or other chips (such as display, rotary controllers, level conversion buffers) can be
difficult.

Using this crate allows each wire connected to the pins to be checked if it is the correct one
by seeing if the  wire has a serial binary signal that matches the pin.

## Usage

Assuming that you have GPIO pins 0 to 4 wired to other components. The following sets up an array
of those pins and then outputs a serial binary signal on each one to identify them.

```rust
let d0: DynPin = pins.gpio0.into_push_pull_output().into();
let d1: DynPin = pins.gpio1.into_push_pull_output().into();
let d2: DynPin = pins.gpio2.into_push_pull_output().into();
let d3: DynPin = pins.gpio3.into_push_pull_output().into();
let d4: DynPin = pins.gpio4.into_push_pull_output().into();

let mut data_pins: [DynPin; 5] = [d0, d1, d2, d3, d4];

let mut signal_ids = match SignalIds::new(&mut data_pins) {
   Ok(s) => s,
   Err(_) => panic!();
   };

loop {
   signal_ids.pulse(&mut delay);
}
```

The pulses generated have the following characteristics (but these can be changed, see later):
* Each pulse has a width of 250 ms.
* The space between pulses is 62 ms (the pulse width / 4).
* After each pulse chain the signal is set low for 1000ms (250 ms * 4).
* The MSB is output first.

The default values can be overwritten:

```rust
signal_ids
    .pulse_width_ms(5)
    .pulse_gap_ms(10)
    .stop_gap_ms(70);
```

If using an oscilloscope to view the pulses, one can set a "clock" pin as trigger. This goes high with the first (MSB)
and low after the last (LSB) pulse.
```rust
let mut clk: DynPin = pins.gpio28.into_push_pull_output().into();
signal_ids.clock_pin(&mut clk);
```

If debugging using an external LED then set the options so that the pulse can be identified with the human eye. For instance
```rust
signal_ids
    .pulse_width_ms(500)
    .pulse_gap_ms(500)
    .stop_gap_ms(2500);
```

## Limitations
* As the array of pins uses DynPin then it can only be used with the HALs for:
   + [RP2040](https://docs.rs/rp2040-hal/latest/rp2040_hal/)
   + [ATSAMD](https://crates.io/crates/atsamd-hal)  (not tested!)
* Times set with the *_us functions are not very accurate.

