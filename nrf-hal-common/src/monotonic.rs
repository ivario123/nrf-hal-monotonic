//! Implements the [Monotonic](rtic_monotonic::Monotonic) trait on the [timer](crate::timer)'s and the [rtc](crate::rtc)'s.
//! ---
//!
//!
//! ## Usage
//! ```rust
//! use nrf52840_hal as hal;
//! use hal::prelude::*;
//! use hal::rtc::Rtc;
//! use hal::timer::Timer;
//! use hal::monotonic::*;
//!
//! #[rtic::app(device = hal::pac,dispatchers=[])]
//! mod app {
//!     use super::*;
//!     // If we want to use the TIMER as the monotonic timer, we need to
//!     #[monotonic(binds = TIMER0, default = true)]
//!     type MyMono = MonotonicTimer<TIMER0>;
//!     // If we want to use the RTC as the monotonic timer, we need to
//!     #[monotonic(binds = RTC0, default = true)]
//!     type MyMono = MonotonicTimer<RTC0>;
//!
//!     #[init]
//!     fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
//!         let clocks = hal::clocks::Clocks::new(cx.device.CLOCK)
//!            .set_lfclk_src_external(hal::clocks::LfOscConfiguration::NoExternalNoBypass) // Depends on board configuration
//!            .start_lfclk() // This is needed for the RTC
//!            .enable_ext_hfosc();
//!
//!         // ...
//!         // Initialize the monotonic timer
//!         let mono = MonotonicTimer::new(cx.device.TIMER0, &clocks); // This will ensure that the clocks have been configured
//!         // or
//!         let mono = MonotonicTimer::new(cx.device.RTC0, &clocks);   // This will ensure that the lfclock is running
//!         // ...
//!
//!         // Return the monotonic timer
//!         (Shared {}, Local {}, init::Monotonics(mono))
//!     }
//!     // ...
//! }
//! ```
//!
//! ## RTC vs TIMER
//!
//! The RTC is a low power clock, which means that using it instead of the TIMER will save power.
//!
//! ### RTC
//!
//! The RTC is only 24 bits wide and runs at 32.768 KHz, which means that it will overflow
//! after `2^24 / 32768 = 512` seconds, the [`MonotonicTimer`] extends the RTC with a 8 bit overflow counter,
//! which means that the RTC can report time up to `2^(24+8) / 327567 = 131072` seconds, which is 36 hours.
//! While it can report time up to 36 hours, it can only set comparison values up to 512 seconds in the future.
//! This means that the longest scheduling interval is 512 seconds or 8 minutes.
//!
//! ### TIMER
//!
//! The TIMER is 32 bits wide and runs at 1 MHz (highest freq that uses the low power clock [§6.30](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A5455%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C555.923%2Cnull%5D)),
//! which means that it will overflow after `2^32 / 1_000_000 = 4295` seconds.
//! Since the TIMER has no overflow flag the [`MonotonicTimer`] does not extend the TIMER with an overflow counter,
//! the longest scheduling interval is 4295 seconds or 71 minutes.
//!
//! ## References
//!
//! All of the references are for the nRF52840, but the same applies to all devices supported by this crate.
//!
//! ## Credit
//! This is heavily inspired by @kroken89's [gist](https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd) and
//! @kalkyl's [nrf-play](https://github.com/kalkyl/nrf-play/blob/47f4410d4e39374c18ff58dc17c25159085fb526/src/mono.rs)
#![deny(missing_docs)]

use rtic_monotonic::Monotonic;

use core::marker::PhantomData;
use core::ops::{ Add, Sub };
use fugit;

/// Implementors of this trait can be used as sources for the [`MonotonicTimer`].
pub trait Mono {
    /// The register type
    type RegPtr: Sized;
    /// The instant type
    type Instant: Ord +
        Copy +
        Add<Self::Duration, Output = Self::Instant> +
        Sub<Self::Duration, Output = Self::Instant> +
        Sub<Self::Instant, Output = Self::Duration>;
    /// The duration type
    type Duration;
    /// Returns a reference to the underlying register
    unsafe fn reg<'a>() -> &'a Self::RegPtr;
    /// Sets the clock freq
    unsafe fn init();
    /// Returns the current clock count
    unsafe fn now(overflow: &mut u8) -> Self::Instant;
    /// Sets the comparison value
    unsafe fn set_compare(instant: Self::Instant, overflow: &mut u8);
    /// Clears the comparison flag
    unsafe fn clear_compare_flag();
    /// Sets the clock to 0 and clears all pending interrupts
    unsafe fn reset();
    /// Returns zero.
    unsafe fn zero() -> Self::Instant;
}

/// Monotonic Timer
/// ---
///
/// Wrapper that allows the usage of the [`timer`](crate::timer)'s or [`rtc`](crate::rtc)'s
/// as [`rtic_monotonic`] timers for use in real time applications.
pub struct MonotonicTimer<T: Mono> {
    timer: PhantomData<T>,
    overflow: u8,
}

/// Blanket [`Monotonic`] implementation
/// This is just a frontend, allowing the backend implementations
/// to handle all the different register accesses needed.
impl<T: Mono> Monotonic for MonotonicTimer<T> {
    type Instant = T::Instant;
    type Duration = T::Duration;

    fn now(&mut self) -> Self::Instant {
        unsafe { T::now(&mut self.overflow) }
    }
    /// Sets the compare value of the timer/counter.
    ///
    /// If the compare value is less than the current value, the value will be set to 0,
    /// since the
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
            $(#[$feature_gate])?
            impl MonotonicTimer<$timer> {
                /// Creates a new [`MonotonicTimer`] using the given [`timer`](crate::timer) and [`clocks`](crate::clocks).
                /// The timer will be configured to run at 1 MHz.
                pub fn new<H, L, LfOsc>(_timer: $timer, _clocks: &crate::clocks::Clocks<H, L, LfOsc>) -> Self {
                    unsafe {
                        $timer::init();
                    }
                    Self {
                        overflow: 0,
                        timer: PhantomData,
                    }
                }
            }
            $(#[$feature_gate])?
                impl Mono for $timer {
                    type Instant = fugit::TimerInstantU32<1_000_000>;
                    type Duration = fugit::TimerDurationU32<1_000_000>;
                    type RegPtr = $reg_block;

                    #[inline(always)]
                    unsafe fn reg<'a>() -> &'a Self::RegPtr {
                        &*$timer::PTR
                    }

                    /// Configures the timer to run at 1 MHz, with a 32 bit counter.
                    /// This means that it will overflow after `2^32 / 1_000_000 = 4295` seconds.
                    /// 
                    /// See [§6.30](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4382%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C388.185%2Cnull%5D) in
                    /// the nRF52840 Product Specification for more information.
                    unsafe fn init() {
                        let timer = Self::reg();
                        timer.prescaler.write(|w| w.prescaler().bits(4)); // 1 MHz
                        timer.bitmode.write(|w| w.bitmode()._32bit());   
                    }

                    unsafe fn now(_:&mut u8) -> Self::Instant {
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
                    unsafe fn set_compare(instant: Self::Instant,overflow:&mut u8) {
                        let target = match instant.checked_duration_since(Self::now(overflow)) {
                            Some(time)  => time.ticks(),// In the future
                            None        => 0            // In the past
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

                    unsafe fn zero() -> Self::Instant{
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
            impl MonotonicTimer<$rtc> {
                /// Creates a new [`MonotonicTimer`] using the given [`rtc`](crate::rtc) and [`clocks`](crate::clocks)
                /// 
                /// To use the RTC as a monotonic timer, the LFCLK must be running [§6.22](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4356%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C566.023%2Cnull%5D).
                pub fn new<H, L>(_timer: $rtc, _clocks: &crate::clocks::Clocks<H, L, crate::clocks::LfOscStarted>) -> Self {
                    unsafe {
                        $rtc::init();
                    }
                    Self {
                        overflow: 0,
                        timer: PhantomData,
                    }
                }
            }
            $( #[$feature_gate] )?
            impl Mono for $rtc {
                type Instant = fugit::TimerInstantU32<{crate::clocks::LFCLK_FREQ}>;
                type Duration = fugit::TimerDurationU32<{crate::clocks::LFCLK_FREQ}>;
                type RegPtr = RtcRegisterBlock;

                #[inline(always)]
                unsafe fn reg<'a>() -> &'a Self::RegPtr {
                    &*$rtc::PTR
                }
                /// Initializes the rtc to run at 32_768 hz
                unsafe fn init() {
                    let timer = Self::reg();
                    timer.prescaler.write(|w| w.bits(0)); // No prescaler
                }

                unsafe fn now(overflow: &mut u8) -> Self::Instant{
                    
                    let timer = Self::reg();
                    let cnt = timer.counter.read().bits();
                    // Reading timerthe overflow register clears the overflow flag
                    let ovf = (if timer.events_ovrflw.read().bits() == 1 { 
                        overflow.wrapping_add(1)
                    } else {
                        *overflow
                    }) as u32;

                    fugit::TimerInstantU32::from_ticks((ovf << 24) | cnt)
                }
                /// Sets comparison target.
                /// ---
                /// 
                /// The time actually set is will be either:
                /// - `target_time` if it is in the future, less than `2^24` ticks away and atleast 3 ticks away.
                /// - `2^24` if it is in the future but atleast one of the above conditions are not met.	
                /// - `0` if the target time is in the past or past the maximum value of the counter.
                /// 
                /// 
                /// ## Refferences
                /// See [§6.22.7](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.7.pdf#%5B%7B%22num%22%3A4382%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C85.039%2C388.185%2Cnull%5D) 
                /// for more details
                /// 
                /// ## Credit
                /// This is lifted straight from [https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd]
                unsafe fn set_compare(instant:Self::Instant, overflow: &mut u8) {
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

                /// Clears the compare flag
                unsafe fn clear_compare_flag() {
                    Self::reg().events_compare[0].write(|w| w.bits(0));
                }

                /// Clears all flags and compare values, then starts the timer
                unsafe fn reset() {
                    let timer = Self::reg();
                    timer.intenset.modify(|_, w| w.compare0().set());
                    timer.evtenset.write(|w| w.compare0().set());
                    timer.tasks_clear.write(|w| w.bits(1));
                    timer.tasks_start.write(|w| w.bits(1));
                }

                unsafe fn zero() -> Self::Instant{
                    fugit::TimerInstantU32::from_ticks(0)
                }
            }
          )+
    };
}
use crate::pac;
#[cfg(any(feature = "9160", feature = "5340-app", feature = "5340-net"))]
use pac::{ rtc0_ns::RegisterBlock as RtcRegisterBlock, RTC0_NS as RTC0, RTC1_NS as RTC1 };

#[cfg(not(any(feature = "9160", feature = "5340-app", feature = "5340-net")))]
use pac::{ rtc0::RegisterBlock as RtcRegisterBlock, RTC0, RTC1 };

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
    timer0_ns::RegisterBlock as RegBlock0,
    TIMER0_NS as TIMER0,
    TIMER1_NS as TIMER1,
    TIMER2_NS as TIMER2,
};

#[cfg(not(any(feature = "9160", feature = "5340-app", feature = "5340-net")))]
use pac::{ timer0::RegisterBlock as RegBlock0, TIMER0, TIMER1, TIMER2 };

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use pac::{ TIMER3, TIMER4 };

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