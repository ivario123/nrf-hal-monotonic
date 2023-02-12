/*!
Implements the [Monotonic](rtic_monotonic::Monotonic) trait for the TIMERs and the RTCs.

## Usage
```no_run
#![no_main]
#![no_std]

use core::panic::PanicInfo;
use nrf52840_hal as hal;
use hal::monotonic::MonotonicTimer;
use hal::pac;

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[rtic::app(device = pac, dispatchers = [TIMER2, TIMER3])]
mod app {
    use super::*;
    #[monotonic(binds = RTC0, default = true)]
    type Mono = MonotonicTimer<pac::RTC0, 32_768u32>

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let clocks = hal::clocks::Clocks::new(cx.device.CLOCK)
        .set_lfclk_src_external(hal::clocks::LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk()
        .enable_ext_hfosc();

        let mono = Mono::new(cx.device.RTC0, &clocks).unwrap();

        // Return shared, local and mono
        (Shared {}, Local {}, init::Monotonics(mono))
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }
}
```


## RTC vs TIMER

The RTC's use a low power clock, which means that using one of them instead of a TIMER will save power.
It does however come with a lower base frequency meaning that the resolution of the rtc is lower
than that of the timer.

### RTC

The RTC has a base frequency of 32.768 KHz, it uses a 12 bit prescaler to achieve the requested frequency.
The abstraction will report an error for any frequency that requires a prescaler larger than 12 bits
or any frequency that does not yield an integer prescaler.

#### Overflow

The RTC is only 24 bits wide which means that the time until overflow is given by the following formula:
`T_{overflow} = 2^24/freq`. Therefore the time until overflow for the maximum frequency is `2^24 / 32768 = 512` seconds,
the [`MonotonicTimer`] extends the RTC with a 8 bit overflow counter, which means that the RTC can report time
up to `2^(24+8) / 327567 = 131072` seconds, which is 36 hours.

### TIMER

The [`Timer`] [§6.30](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A5455%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C555.923%2Cnull%5D)
has 2 different clock sources that can drive it, one 16Mhz clock that is used when
the timers frequency is higher than 1 mhz where the timers frequency is given by:

`f_TIMER = 16 MHz / (2^PRESCALER)`
Where the prescaler is a 4 bit integer.

And one 1Mhz clock source which is used when the f_TIMER is at or lower than 1Mhz.
The 1MHz clock is lower power than the 16MHz clock source, so for low applications it could be beneficial to use a
frequency at or below 1MHz.

### Overflow

The TIMER's are configured to use a 32 bit wide counter, this means that the time until overflow is given by the following formula:
`T_overflow = 2^32/freq`. Therefore the time until overflow for the maximum frequency (16MHz) is `2^32/(16*10^6) = 268` seconds, using a
1MHz TIMER yields time till overflow `2^32/(10^6) = 4295` seconds or 1.2 hours.

## References

All of the references are for the nRF52840, but the same applies to all devices supported by this crate.

## Credit
This is heavily inspired by @kroken89's [gist](https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd) and
@kalkyl's [nrf-play](https://github.com/kalkyl/nrf-play/blob/47f4410d4e39374c18ff58dc17c25159085fb526/src/mono.rs)
*/
#![deny(missing_docs)]
#![deny(non_camel_case_types)]
#![deny(clippy::all)]

use core::marker::PhantomData;
use fugit;
use paste::paste;
use rtic_monotonic::Monotonic;

/// The errors that can occur when using the [`MonotonicTimer`]
#[derive(Debug)]
pub enum Error {
    /// There is no way of achieving the requested frequency.
    ///
    /// The requested prescaler is larger than the field width.
    /// The prescaler size depends on wether the timer is a TIMER or a RTC.
    /// - TIMER: 4 bits
    /// - RTC: 12 bits
    ImpossibleFrequency(u32),

    /// The requested frequency is not a valid result of either:
    ///
    /// - TIMER: `f_TIMER = 16MHz / (2^PRESCALER)` [§6.30](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A5455%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C555.923%2Cnull%5D)
    /// - RTC: `f_RTC [kHz] = 32.768 / (PRESCALER + 1 )` [§6.22.2](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4356%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C109.877%2Cnull%5D)
    InvalidFrequency(u32),
}

/// Hardware access abstraction for the monotonic timers.
///
/// Implementors of this trait can be used as sources for the [`MonotonicTimer`].
pub trait Mono {
    /// The base frequency of the underlying clock.
    ///
    /// - RTC: 32_768 Hz
    /// - TIMER: 16_000_000 Hz
    const BASE_FREQ: u32;

    /// The type of the underlying register
    type RegPtr: Sized;

    /// Checks if the requested frequency is valid.
    ///
    /// - Returns the prescaler value and the actual frequency.
    /// - Returns [`Error::InvalidFrequency`] if the frequency is not valid.
    /// - Return [`Error::ImpossibleFrequency`] if the requested prescaler exceeds
    /// possible prescalers.
    fn check_freq(freq: u32) -> Result<(u32, u32), Error>;

    /// Returns a reference to the underlying register.
    unsafe fn reg<'a>() -> &'a Self::RegPtr;

    /// Sets the clock freq.
    unsafe fn init<const FREQ: u32>() -> Result<u32, Error>;

    /// Returns the current clock count.
    unsafe fn now<const FREQ: u32>(overflow: &mut u8) -> fugit::TimerInstantU32<FREQ>;

    /// Sets the comparison value.
    unsafe fn set_compare<const FREQ: u32>(
        instant: fugit::TimerInstantU32<FREQ>,
        overflow: &mut u8,
    );

    /// Clears the comparison flag.
    unsafe fn clear_compare_flag();

    /// Sets the clock to 0 and clears all pending interrupts.
    unsafe fn reset();

    /// Returns zero.
    unsafe fn zero<const FREQ: u32>() -> fugit::TimerInstantU32<FREQ>;
}

/// Monotonic Timer
///
/// Wrapper that allows the usage of the [`timer`](crate::timer)'s or [`rtc`](crate::rtc)'s
/// as [`rtic_monotonic`] timers for use in [`rtic`](https://docs.rs/cortex-m-rtic/1.1.3/rtic/) applications.
pub struct MonotonicTimer<T: Mono, const FREQ: u32> {
    timer: PhantomData<T>,
    overflow: u8,
}

impl<T: Mono, const FREQ: u32> Monotonic for MonotonicTimer<T, FREQ> {
    type Instant = fugit::TimerInstantU32<FREQ>;
    type Duration = fugit::TimerDurationU32<FREQ>;

    fn now(&mut self) -> Self::Instant {
        unsafe { T::now(&mut self.overflow) }
    }

    fn set_compare(&mut self, instant: Self::Instant) {
        unsafe {
            T::set_compare(instant, &mut self.overflow);
        }
    }

    fn clear_compare_flag(&mut self) {
        unsafe {
            T::clear_compare_flag();
        }
    }

    fn zero() -> Self::Instant {
        unsafe { T::zero() }
    }

    unsafe fn reset(&mut self) {
        T::reset()
    }
}

// Credit to Henrik Alsér @kalkyl on github [https://github.com/kalkyl/nrf-play/blob/47f4410d4e39374c18ff58dc17c25159085fb526/src/mono.rs]
macro_rules! timers {
    (
        $(
            $(#[$feature_gate:meta])?
            $timer:ident,
            $reg_block:ident;
        )+
    ) => {
        $(
            $( #[$feature_gate] )?
            paste!{
                /// 1MHz timer
                ///
                /// The fastest timer that uses the low power clock.
                pub type [<$timer:camel  _1MHz>] = MonotonicTimer<$timer, 1_000_000>;

                /// 16MHz timer
                ///
                /// The fastest possible timer.
                pub type [<$timer:camel  _16MHz>] = MonotonicTimer<$timer, 16_000_000>;


            }
            $(#[$feature_gate])?
            impl<const FREQ:u32> MonotonicTimer<$timer,FREQ> {
                paste!{
                    #[doc = "Creates a new [`MonotonicTimer`] using the [`"[<$timer>]"`](pac::"[<$timer>]") and [`clocks`](crate::clocks)."]
                    #[doc = "This will ensure that the clocks have been configured before configuring the timer"]
                    pub fn new<H, L, LfOsc>(_timer: $timer, _clocks: &crate::clocks::Clocks<H, L, LfOsc>) -> Result<Self,Error> {
                        unsafe {
                            $timer::init::<FREQ>()?;
                        }
                        Ok(Self {
                            overflow: 0,
                            timer: PhantomData,
                        })
                    }
                }
            }
            $(#[$feature_gate])?
            impl Mono for $timer {
                const BASE_FREQ: u32 = 16_000_000;
                type RegPtr = $reg_block;

                #[inline(always)]
                unsafe fn reg<'a>() -> &'a Self::RegPtr {
                    &*$timer::PTR
                }

                fn check_freq(freq: u32) -> Result<(u32,u32), Error> {
                    // This is done in setup, panicking on errors is expected
                    let prescaler = (Self::BASE_FREQ / freq).checked_ilog2().unwrap();
                    // We want to panic as this is done in setup
                    let actual_freq = Self::BASE_FREQ / (2u32.checked_pow(prescaler).unwrap());
                    if actual_freq != freq{
                        return Err(Error::InvalidFrequency(freq));
                    }
                    match prescaler {
                        0..=0b1111 => Ok((prescaler, Self::BASE_FREQ / prescaler)),
                        _ => Err(Error::ImpossibleFrequency(freq)),
                    }
                }

                /// Configures the timer to run at the requested frequency if it is valid.
                ///
                /// A frequency is valid if the result from `log2(16Mhz/freq)` is less than or equal to 15 and
                /// the the result is a whole number.
                ///
                /// See [§6.30](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A5455%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C555.923%2Cnull%5D) in
                /// the nRF52840 Product Specification for more information.
                unsafe fn init<const FREQ:u32>() -> Result<u32,Error> {
                    let timer = Self::reg();
                    let pre = Self::check_freq(FREQ)?;
                    timer.prescaler.write(|w| w.prescaler().bits(pre.0 as u8));
                    timer.bitmode.write(|w| w.bitmode()._32bit());
                    Ok(pre.1)
                }

                unsafe fn now<const FREQ:u32>(_overflow: &mut u8) -> fugit::TimerInstantU32<FREQ> {
                    let timer = Self::reg();
                    timer.tasks_capture[1].write(|w| w.bits(1));
                    fugit::TimerInstantU32::from_ticks(timer.cc[1].read().bits())
                }
                /// Sets the compare value
                ///
                /// The target value is set to one of the following:
                /// - The target value if it is in the future
                /// - 0 if the target value is in the past or will overflow
                ///
                /// This is done to prevent the timer from overflowing.
                unsafe fn set_compare<const FREQ:u32>(instant: fugit::TimerInstantU32<FREQ>,overflow:&mut u8) {
                    let target = match instant.checked_duration_since(Self::now(overflow)) {
                        Some(time)  => time.ticks(), // In the future
                        None        => 0             // In the past
                    };
                    Self::reg().cc[0].write(|w| w.cc().bits(target));
                }

                unsafe fn clear_compare_flag() {
                    Self::reg().events_compare[0].write(|w| w); // Clear the value
                }

                unsafe fn reset() {
                    let timer = Self::reg();
                    timer.intenset.modify(|_, w| w.compare0().set());
                    timer.tasks_clear.write(|w| w.bits(1)); // Clear the value
                    timer.tasks_start.write(|w| w.bits(1)); // Start the timer
                }

                unsafe fn zero<const FREQ:u32>() -> fugit::TimerInstantU32<FREQ>{
                    fugit::TimerInstantU32::from_ticks(0) // Return zero
                }
            }
        )+
    };
}

// Credit @kroken89 on github [https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd]
macro_rules! rtcs {
    (
        $(
            $(#[$feature_gate:meta])?
            $rtc:ident;
        )+
    ) => {
        $(
            $( #[$feature_gate] )?
            paste!{
                /// Provides a default valid rtc configuration, this is the maximum frequency that is
                /// available for the rtcs.
                pub type [<$rtc:camel _32768Hz>] = MonotonicTimer<$rtc, 32_768_u32>;
            }

            $( #[$feature_gate] )?
            impl<const FREQ:u32> MonotonicTimer<$rtc,FREQ> {
                paste!{
                    #[doc = "Creates a new [`MonotonicTimer`] using [`" [<$rtc>] "`](pac::"[<$rtc>]") and a configured [`clocks`](crate::clocks) object."]
                    #[doc = "To use the RTC as a monotonic timer, the LFCLK must be running [§6.22](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4356%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C566.023%2Cnull%5D)."]
                    pub fn new<H, L>(_timer: $rtc, _clocks: &crate::clocks::Clocks<H, L, crate::clocks::LfOscStarted>) -> Result<Self,Error> {
                        unsafe {
                            // If the freq is not valid, return the appropriate error
                            $rtc::init::<FREQ>()?;
                        }
                        Ok(Self {
                            overflow: 0,
                            timer: PhantomData,
                        })
                    }
                }
            }
            $( #[$feature_gate] )?
            impl Mono for $rtc {
                const BASE_FREQ : u32 = crate::clocks::LFCLK_FREQ;
                type RegPtr = RtcRegisterBlock;

                #[inline(always)]
                unsafe fn reg<'a>() -> &'a Self::RegPtr {
                    &*$rtc::PTR
                }

                fn check_freq(freq: u32) -> Result<(u32,u32), Error> {
                    let prescaler = Self::BASE_FREQ / freq;
                    // Checks that the prescaler fits in 12 bits
                    match prescaler {
                        0 => Err(Error::InvalidFrequency(freq)),
                        _ if prescaler*freq != Self::BASE_FREQ => Err(Error::InvalidFrequency(freq)),
                        _ if prescaler > (1 << 12) => Err(Error::InvalidFrequency(freq)),
                        _ => Ok((prescaler,Self::BASE_FREQ / prescaler)),
                    }
                }

                /// Configures the rtc to run at the requested frequency if it is valid.
                ///
                /// See [§6.22.2](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4356%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C109.877%2Cnull%5D) for
                /// more information on valid clock frequencies
                unsafe fn init<const FREQ:u32>() -> Result<u32,Error> {
                    let timer = Self::reg();
                    let pre = Self::check_freq(FREQ)?;
                    timer.prescaler.write(|w| w.bits(pre.0)); // No prescaler
                    Ok(pre.1)
                }

                unsafe fn now<const FREQ:u32>(overflow: &mut u8) -> fugit::TimerInstantU32<FREQ>{
                    let timer = Self::reg();
                    let cnt = timer.counter.read().bits();
                    // Reading the timer overflow register clears the overflow flag
                    let ovf = (if timer.events_ovrflw.read().bits() == 1 {
                        overflow.wrapping_add(1)
                    } else {
                        *overflow
                    }) as u32;

                    fugit::TimerInstantU32::from_ticks((ovf << 24) | cnt)
                }

                /// Sets comparison target.
                ///
                /// The time actually set is will be either:
                /// - `target_time` if it is in the future, less than `2^24` ticks away and at least 3 ticks away.
                /// - `2^24` if it is in the future but at least one of the above conditions are not met.
                /// - `0` if the target time is in the past or past the maximum value of the counter.
                ///
                ///
                /// ## References
                /// See [§6.22.7](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4382%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C388.185%2Cnull%5D)
                /// for more details
                ///
                /// ## Credit
                /// This is lifted straight from <https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd>
                unsafe fn set_compare<const FREQ:u32>(instant:fugit::TimerInstantU32<FREQ>, overflow: &mut u8) {
                    let now = Self::now(overflow);

                    const MIN_TICKS_FOR_COMPARE: u32 = 3;

                    // Since the timer may or may not overflow based on the requested compare val, we check
                    // how many ticks are left.
                    //
                    // Note: The RTC cannot have a compare value too close to the current timer value,
                    // so we use the `MIN_TICKS_FOR_COMPARE` to set a minimum offset from now to the set value.
                    let val = match instant.checked_duration_since(now) {
                        Some(x) if x.ticks() <= 0xffffff && x.ticks() > MIN_TICKS_FOR_COMPARE => {
                            instant.duration_since_epoch().ticks() & 0xffffff
                        } // Will not overflow
                        Some(x) => {
                            (instant.duration_since_epoch().ticks() + (MIN_TICKS_FOR_COMPARE - x.ticks())) &
                            0xffffff
                        } // Will not overflow
                        _ => 0, // Will overflow or in the past, set the same value as after overflow to not get extra interrupts
                    };
                    Self::reg().cc[0].write(|w| w.bits(val))
                }

                unsafe fn clear_compare_flag() {
                    Self::reg().events_compare[0].write(|w| w.bits(0));
                }

                unsafe fn reset() {
                    let timer = Self::reg();
                    timer.intenset.modify(|_, w| w.compare0().set());
                    timer.evtenset.write(|w| w.compare0().set());
                    timer.tasks_clear.write(|w| w.bits(1));
                    timer.tasks_start.write(|w| w.bits(1));
                }

                unsafe fn zero<const FREQ:u32>() -> fugit::TimerInstantU32<FREQ>{
                    fugit::TimerInstantU32::from_ticks(0)
                }
            }
        )+
    };
}
use crate::pac;
#[cfg(any(feature = "9160", feature = "5340-app", feature = "5340-net"))]
use pac::{rtc0_ns::RegisterBlock as RtcRegisterBlock, RTC0_NS as RTC0, RTC1_NS as RTC1};

#[cfg(not(any(feature = "9160", feature = "5340-app", feature = "5340-net")))]
use pac::{rtc0::RegisterBlock as RtcRegisterBlock, RTC0, RTC1};

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use pac::RTC2;

rtcs! {
    RTC0;
    RTC1;
    #[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
    RTC2;
}

#[cfg(any(feature = "9160", feature = "5340-app", feature = "5340-net"))]
use pac::{
    timer0_ns::RegisterBlock as RegBlock0, TIMER0_NS as TIMER0, TIMER1_NS as TIMER1,
    TIMER2_NS as TIMER2,
};

#[cfg(not(any(feature = "9160", feature = "5340-app", feature = "5340-net")))]
use pac::{timer0::RegisterBlock as RegBlock0, TIMER0, TIMER1, TIMER2};

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use pac::{TIMER3, TIMER4};

#[cfg(any(feature = "52832", feature = "52840"))]
use pac::timer3::RegisterBlock as RegBlock3;

#[cfg(feature = "52833")]
use pac::timer0::RegisterBlock as RegBlock3;

timers!(
    TIMER0,RegBlock0;
    TIMER1,RegBlock0;
    TIMER2,RegBlock0;
    #[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
    TIMER3,RegBlock3;
    #[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
    TIMER4,RegBlock3;

);
