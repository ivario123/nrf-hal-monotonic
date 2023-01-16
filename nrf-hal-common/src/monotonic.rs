//! Implements the [Monotonic](rtic_monotonic::Monotonic) trait on the [timer](crate::timer)'s and the [rtc](crate::rtc)'s.
//! ---
//!
//! ## Credit
//! This is heavily inspired by @kroken89's [gist](https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd) and
//! @kalkyl's [nrf-play](https://github.com/kalkyl/nrf-play/blob/47f4410d4e39374c18ff58dc17c25159085fb526/src/mono.rs)

use rtic_monotonic::Monotonic;

use core::marker::PhantomData;
use fugit;

/// Implementors of this trait can be used as sources for the [`MonotonicTimer`].
pub trait Mono {
    type RegPtr: Sized;
    unsafe fn reg<'a>() -> &'a Self::RegPtr;
    unsafe fn init();
    unsafe fn now(overflow: &mut u8) -> fugit::TimerInstantU32<1_000_000>;
    unsafe fn set_compare(instant: fugit::TimerInstantU32<1_000_000>, overflow: &mut u8);
    unsafe fn clear_compare_flag();
    unsafe fn reset();
    unsafe fn zero() -> fugit::TimerInstantU32<1_000_000> {
        fugit::TimerInstantU32::from_ticks(0)
    }
}

/// Monotonic Timer
/// ---
///
/// Wrapper that allows the usage of the [`timer`](crate::timer)'s or [`rtc`]'s
/// as [`rtic_monotonic`] timers for use in real time applications.
pub struct MonotonicTimer<T: Mono> {
    timer: PhantomData<T>,
    overflow: u8,
}
impl<T: Mono> MonotonicTimer<T> {
    /// Creates a new [`MonotonicTimer`]
    /// ---
    ///
    /// See documentation for [`rtic_monotonic`]
    /// for more information.
    pub fn new(_timer: T) -> Self {
        unsafe {
            T::init();
        }
        Self {
            overflow: 0,
            timer: PhantomData,
        }
    }
}

impl<T: Mono> Monotonic for MonotonicTimer<T> {
    type Instant = fugit::TimerInstantU32<1_000_000>;

    type Duration = fugit::TimerDurationU32<1_000_000>;

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
            $(#[$feature_gate])?
                impl Mono for $timer {
                    type RegPtr = $reg_block;
                    
                    #[inline(always)]   
                    unsafe fn reg<'a>() -> &'a Self::RegPtr {
                        &*$timer::PTR
                    }
                
                    unsafe fn init() {
                        let timer = Self::reg();
                        timer.prescaler.write(|w| { w.prescaler().bits(4) }); // 1 MHz
                        timer.bitmode.write(|w| { w.bitmode()._32bit() });
                    }
                
                    unsafe fn now(_:&mut u8) -> fugit::TimerInstantU32<1_000_000> {
                        let timer = Self::reg();
                        timer.tasks_capture[1].write(|w| w.bits(1));
                        fugit::TimerInstantU32::from_ticks(timer.cc[1].read().bits())
                    }
                
                    unsafe fn set_compare(instant: fugit::TimerInstantU32<1_000_000>,_:&mut u8) {
                        Self::reg().cc[0].write(|w| w.cc().bits(instant.duration_since_epoch().ticks()));
                    }
                
                    unsafe fn clear_compare_flag() {
                        Self::reg().events_compare[0].write(|w| w); // Clear the value
                    }
                
                    unsafe fn reset() {
                        let timer = Self::reg();
                        timer.intenset.modify(|_, w| w.compare0().set());
                        timer.tasks_clear.write(|w| w.bits(1));
                        timer.tasks_start.write(|w| w.bits(1));
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
            impl Mono for $rtc {
                type RegPtr = RtcRegisterBlock;
                
                #[inline(always)]
                unsafe fn reg<'a>() -> &'a Self::RegPtr {
                    &*$rtc::PTR
                }
            
                unsafe fn init() {
                    let timer = Self::reg();
                    timer.prescaler.write(|w| w.bits(0)); //32_768 hz
                }
            
                unsafe fn now(overflow: &mut u8) -> fugit::TimerInstantU32<1_000_000> {
                    let timer = Self::reg();
                    let cnt = timer.counter.read().bits();
                    let ovf = (if timer.events_ovrflw.read().bits() == 1 {
                        overflow.wrapping_add(1)
                    } else {
                        *overflow
                    }) as u32;
            
                    fugit::TimerInstantU32::from_ticks((ovf << 24) | cnt)
                }
                /// This is lifted straight from [https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd]
                unsafe fn set_compare(instant: fugit::TimerInstantU32<1_000_000>, overflow: &mut u8) {
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
                    Self::reg().events_compare[0].write(|w| w.bits(0)); // Clear the value
                }
            
                unsafe fn reset() {
                    let timer = Self::reg();
                    timer.intenset.modify(|_, w| w.compare0().set());
                    timer.evtenset.write(|w| w.compare0().set());
                    timer.tasks_clear.write(|w| w.bits(1));
                    timer.tasks_start.write(|w| w.bits(1));
                }
            }
          )+
    };
}

#[cfg(any(feature = "9160", feature = "5340-app", feature = "5340-net"))]
use crate::pac::{ rtc0_ns::RegisterBlock as RtcRegisterBlock, RTC0_NS as RTC0, RTC1_NS as RTC1 };

#[cfg(not(any(feature = "9160", feature = "5340-app", feature = "5340-net")))]
use crate::pac::{ rtc0::RegisterBlock as RtcRegisterBlock, RTC0, RTC1 };

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use crate::pac::RTC2;

rtcs! {
    RTC0;
    RTC1;
    #[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
    RTC2;
}

#[cfg(any(feature = "9160", feature = "5340-app", feature = "5340-net"))]
use crate::pac::{
    timer0_ns::{ RegisterBlock as RegBlock0 },
    TIMER0_NS as TIMER0,
    TIMER1_NS as TIMER1,
    TIMER2_NS as TIMER2,
};

#[cfg(not(any(feature = "9160", feature = "5340-app", feature = "5340-net")))]
use crate::pac::{ timer0::{ RegisterBlock as RegBlock0 }, TIMER0, TIMER1, TIMER2 };

#[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
use crate::pac::{ TIMER3, TIMER4 };

#[cfg(any(feature = "52832", feature = "52840"))]
use crate::pac::timer3::{ RegisterBlock as RegBlock3 };

#[cfg(feature = "52833")]
use crate::pac::timer0::{ RegisterBlock as RegBlock3 };

timers!(
    TIMER0,RegBlock0;
    TIMER1,RegBlock0;
    TIMER2,RegBlock0;
    #[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
    TIMER3,RegBlock3;
    #[cfg(any(feature = "52832", feature = "52833", feature = "52840"))]
    TIMER4,RegBlock3;
    
);