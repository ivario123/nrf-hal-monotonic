#![macro_use]
use core::{
    future::poll_fn, hint::unreachable_unchecked, marker::PhantomData,
    sync::atomic::compiler_fence, task::Poll,
};
use embedded_hal_async::spi::*;
use nrf52840_pac::spis0;
use spis0::config::ORDER_A as Bitorder;

use crate::{
    gpio::{Floating, Input, Pin},
    target_constants::EASY_DMA_SIZE,
};

use self::sealed::{Argument, Instance, Setters};

/// This type will litter the codebase.
type P = Pin<Input<Floating>>;

/// Enumerates the errors that the async spi can throw.
pub enum Error {
    /// Thrown when the buffer does not fit in to the dma buffer.
    BufferTooLarget { requested: usize, max: usize },

    /// Thrown when the reading procedure somehow read more bytes
    /// than what the buffer can store.
    BufferOverRun(usize),
}

pub enum Pins<const HAS_CS: bool, const RX: bool, const TX: bool> {
    /// only capable of recieving data.
    RX {
        mosi: P,
        clock: P,
        chip_select: Option<P>,
    },
    /// Only capable of transmitting data.
    TX {
        miso: P,
        clock: P,
        chip_select: Option<P>,
    },
    /// Can transmit and receive data.
    Duplex {
        mosi: P,
        miso: P,
        clock: P,
        chip_select: Option<P>,
    },
}

pub struct Config {
    /// Spi mode as defined in [`Mode`].
    pub mode: embedded_hal_async::spi::Mode,

    /// Bit order.
    pub bit_order: Bitorder,

    /// Read when ever the read buffer is overrun.
    pub over_read_character: u8,

    /// Read when the user space owns the lock and the master sets cs high.
    pub defualt_byte: u8,

    /// Auto lock the semaphore on transfer completion.
    pub auto_acquire: bool,
}

pub struct Spi<I: Instance, const TX: bool, const RX: bool, const CS: bool, const BUFFERSIZE: usize>
{
    instance: PhantomData<I>,
    tx: Option<*mut [u8; BUFFERSIZE]>,
    rx: Option<*const [u8; BUFFERSIZE]>,
}

impl<I: Instance, const CS: bool, const TX: bool, const RX: bool, const BUFFERSIZE: usize>
    Spi<I, TX, RX, CS, BUFFERSIZE>
{
    pub fn new<A: Argument<BUFFERSIZE>>(
        mut spi: I,
        arguments: A,
        config: Config,
    ) -> Result<Self, Error> {
        if BUFFERSIZE > EASY_DMA_SIZE {
            return Err(Error::BufferTooLarget {
                requested: BUFFERSIZE,
                max: EASY_DMA_SIZE,
            });
        }
        let mut ret = Spi {
            instance: PhantomData,
            tx: None,
            rx: None,
        };
        arguments.apply(&mut ret, &mut spi);
        config.apply(&mut spi);

        let spi = spi.spi();
        spi.enable.write(|w| w.enable().enabled());

        Ok(ret)
    }
}
//
// impl<I: Instance, const CS: bool, const BUFFERSIZE: usize> Spi<I, true, false, CS, BUFFERSIZE> {
//     const _BOUNDS_CHECK: () = assert!(BUFFERSIZE <= EASY_DMA_SIZE);
//     pub fn new(
//         mut spi: I,
//         pins: Pins<CS, false, true>,
//         tx_buffer: *mut [u8; BUFFERSIZE],
//         config: Config,
//     ) -> Result<Self, Error> {
//         if BUFFERSIZE > EASY_DMA_SIZE {
//             return Err(Error::BufferTooLarget {
//                 requested: BUFFERSIZE,
//                 max: EASY_DMA_SIZE,
//             });
//         }
//         pins.configure(&mut spi);
//         config.apply(&mut spi);
//
//         let spi = spi.spi();
//         spi.enable.write(|w| w.enable().enabled());
//
//         Ok(Spi {
//             instance: PhantomData,
//             tx: Some(tx_buffer),
//             rx: None,
//         })
//     }
// }
//
// impl<I: Instance, const CS: bool, const BUFFERSIZE: usize> Spi<I, false, true, CS, BUFFERSIZE> {
//     const _BOUNDS_CHECK: () = assert!(BUFFERSIZE <= EASY_DMA_SIZE);
//     pub fn new(
//         mut spi: I,
//         pins: Pins<CS, true, false>,
//         rx_buffer: *const [u8; BUFFERSIZE],
//         config: Config,
//     ) -> Result<Self, Error> {
//         if BUFFERSIZE > EASY_DMA_SIZE {
//             return Err(Error::BufferTooLarget {
//                 requested: BUFFERSIZE,
//                 max: EASY_DMA_SIZE,
//             });
//         }
//         pins.configure(&mut spi);
//         config.apply(&mut spi);
//
//         let spi = spi.spi();
//         spi.enable.write(|w| w.enable().enabled());
//
//         Ok(Spi {
//             instance: PhantomData,
//             tx: None,
//             rx: Some(rx_buffer),
//         })
//     }
// }
//
// impl<I: Instance, const CS: bool, const BUFFERSIZE: usize> Spi<I, true, true, CS, BUFFERSIZE> {
//     const _BOUNDS_CHECK: () = assert!(BUFFERSIZE <= EASY_DMA_SIZE);
//     pub fn new(
//         mut spi: I,
//         pins: Pins<CS, true, true>,
//         tx_buffer: *mut [u8; BUFFERSIZE],
//         rx_buffer: *const [u8; BUFFERSIZE],
//         config: Config,
//     ) -> Result<Self, Error> {
//         if BUFFERSIZE > EASY_DMA_SIZE {
//             return Err(Error::BufferTooLarget {
//                 requested: BUFFERSIZE,
//                 max: EASY_DMA_SIZE,
//             });
//         }
//         pins.configure(&mut spi);
//         config.apply(&mut spi);
//         let spi = spi.spi();
//
//         spi.enable.write(|w| w.enable().enabled());
//
//         Ok(Spi {
//             instance: PhantomData,
//             tx: Some(tx_buffer),
//             rx: Some(rx_buffer),
//         })
//     }
// }

// SECTION: Inner helpers.
//
// This section includes some code coppied from the embassy project.
//
/*
Copyright (c) Embassy project contributors

Permission is hereby granted, free of charge, to any
person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the
Software without restriction, including without
limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following
conditons
*/
impl<I: Instance, const CS: bool, const TX: bool, const RX: bool, const BUFFERSIZE: usize>
    Spi<I, TX, RX, CS, BUFFERSIZE>
{
    /// heavily inspired by https://github.com/embassy-rs/embassy/blob/main/embassy-nrf/src/spis.rs
    /// More or less lifted straight from it.
    async fn inner(&mut self) -> (usize, usize) {
        let r = I::const_spi();
        let s = I::state();

        // Clear status register.
        r.status.write(|w| w.overflow().clear().overread().clear());

        // Acquire semaphore.
        if r.semstat.read().bits() != 1 {
            // Reset and enable the acquire event.
            r.events_acquired.reset();
            r.intenset.write(|w| w.acquired().set());

            // Request acquiring the SPIS semaphore.
            r.tasks_acquire.write(|w| unsafe { w.bits(1) });

            // Wait until CPU has acquired the semaphore.
            poll_fn(|cx| {
                s.waker.register(cx.waker());
                if r.events_acquired.read().bits() == 1 {
                    r.events_acquired.reset();
                    return Poll::Ready(());
                }
                Poll::Pending
            })
            .await;
        }

        self.prepare();

        // Wait for 'end' event.
        r.intenset.write(|w| w.end().set());
        poll_fn(|cx| {
            s.waker.register(cx.waker());
            if r.events_end.read().bits() != 0 {
                r.events_end.reset();
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;

        let n_rx = r.rxd.amount.read().bits() as usize;
        let n_tx = r.txd.amount.read().bits() as usize;

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        (n_rx, n_tx)
    }

    // Acquires the semaphore.
    async fn lock(&self) {
        let spi = I::const_spi();
        let waker = I::state();
        // Acquire semaphore.
        if spi.semstat.read().bits() != 1 {
            // Reset and enable the acquire event.
            spi.events_acquired.reset();
            spi.intenset.write(|w| w.acquired().set());

            // Request acquiring the SPIS semaphore.
            spi.tasks_acquire.write(|w| unsafe { w.bits(1) });

            // Wait until CPU has acquired the semaphore.
            poll_fn(|cx| {
                waker.waker.register(cx.waker());
                if spi.events_acquired.read().bits() == 1 {
                    spi.events_acquired.reset();
                    return Poll::Ready(());
                }
                Poll::Pending
            })
            .await;
        }
    }

    // Acquires the semaphore.
    async fn locked<F: FnOnce() -> ()>(&self, callable: F) {
        self.lock().await;
        callable();
        self.release();
    }

    // Releases the semaphore.
    fn release(&self) {
        let spi = I::const_spi();
        spi.tasks_release.write(|w| unsafe { w.bits(1) });
    }

    /// heavily inspired by https://github.com/embassy-rs/embassy/blob/main/embassy-nrf/src/spis.rs
    /// More or less lifted straight from it.
    fn prepare(&mut self) {
        let r = I::const_spi();

        if TX {
            // Set up the DMA write.
            let ptr = self
                .tx
                .unwrap_or_else(|| unsafe { unreachable_unchecked() });

            r.txd.ptr.write(|w| unsafe { w.ptr().bits(ptr as _) });
            r.txd
                .maxcnt
                .write(|w| unsafe { w.maxcnt().bits(BUFFERSIZE as _) });
        }
        if RX {
            // Set up the DMA read.
            let ptr = self
                .rx
                .unwrap_or_else(|| unsafe { unreachable_unchecked() });

            r.rxd.ptr.write(|w| unsafe { w.ptr().bits(ptr as _) });
            r.rxd
                .maxcnt
                .write(|w| unsafe { w.maxcnt().bits(BUFFERSIZE as _) });
        }

        // Reset end event.
        r.events_end.reset();

        // Release the semaphore.
        r.tasks_release.write(|w| unsafe { w.bits(1) });
    }
}

impl<I: Instance, const CS: bool, const TX: bool, const BUFFERSIZE: usize>
    Spi<I, TX, true, CS, BUFFERSIZE>
{
    pub async fn read(&mut self) -> Result<(usize, [u8; BUFFERSIZE]), Error> {
        let read_bytes = self.inner().await.0;
        // This should not possibly happen.
        if read_bytes > BUFFERSIZE {
            return Err(Error::BufferOverRun(read_bytes));
        }

        let mut ret_buff = [0; BUFFERSIZE];

        self.locked(|| {
            // Get the buffer from the DMA and return a clone of it to the user.
            let rx = unsafe { *self.rx.unwrap_or_else(|| unreachable_unchecked()) };
            ret_buff.copy_from_slice(&rx);
        })
        .await;

        Ok((read_bytes, ret_buff))
    }
}

impl<I: Instance, const CS: bool, const RX: bool, const BUFFERSIZE: usize>
    Spi<I, true, RX, CS, BUFFERSIZE>
{
    pub async fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.locked(|| {
            // Move the data from the buffer in to the DMA buffer.
            let tx: &mut [u8; BUFFERSIZE] =
                unsafe { &mut *self.tx.unwrap_or_else(|| unreachable_unchecked()) };
            tx.copy_from_slice(data)
        })
        .await;

        let written_bytes = self.inner().await.1;

        Ok(written_bytes)
    }
}

impl<I: Instance, const CS: bool, const BUFFERSIZE: usize> Spi<I, true, true, CS, BUFFERSIZE> {
    pub async fn transfer(
        &mut self,
        data: &[u8],
    ) -> Result<((usize, usize), [u8; BUFFERSIZE]), Error> {
        self.locked(|| {
            // Move the data from the buffer in to the DMA buffer.
            let tx: &mut [u8; BUFFERSIZE] =
                unsafe { &mut *self.tx.unwrap_or_else(|| unreachable_unchecked()) };
            tx.copy_from_slice(data)
        })
        .await;

        let (read_bytes, written_bytes) = self.inner().await;

        // This should not possibly happen.
        if read_bytes > BUFFERSIZE {
            return Err(Error::BufferOverRun(read_bytes));
        }

        let mut ret_buff = [0; BUFFERSIZE];

        self.locked(|| {
            // Get the buffer from the DMA and return a clone of it to the user.
            let rx = unsafe { *self.rx.unwrap_or_else(|| unreachable_unchecked()) };
            ret_buff.copy_from_slice(&rx);
        })
        .await;

        Ok(((read_bytes, written_bytes), ret_buff))
    }
}

impl<I: Instance, const CS: bool, const TX: bool, const RX: bool, const BUFFERSIZE: usize>
    Setters<BUFFERSIZE> for Spi<I, TX, RX, CS, BUFFERSIZE>
{
    fn underlying(&self) -> &spis0::RegisterBlock {
        I::const_spi()
    }
    fn set_rx_buff(&mut self, ptr: *const [u8; BUFFERSIZE]) {
        self.rx = Some(ptr)
    }
    fn set_tx_buff(&mut self, ptr: *mut [u8; BUFFERSIZE]) {
        self.tx = Some(ptr)
    }
}

mod sealed {
    use rtic_common::waker_registration::CriticalSectionWakerRegistration;

    use crate::pac::spis0;

    use super::Pins;

    pub struct State {
        pub waker: CriticalSectionWakerRegistration,
    }

    impl State {
        const fn new() -> Self {
            State {
                waker: CriticalSectionWakerRegistration::new(),
            }
        }
    }

    pub trait Instance {
        /// The underlying register.
        const REG: *const spis0::RegisterBlock;
        fn spi(&self) -> &spis0::RegisterBlock {
            unsafe { &*Self::REG }
        }
        fn const_spi<'a>() -> &'a spis0::RegisterBlock {
            unsafe { &*Self::REG }
        }
        /// Returns the current waker state.
        fn state() -> &'static State;
    }

    macro_rules! spi {
        ($($spi:ident),*) => {
            $(
                use crate::pac::$spi;
                impl Instance for $spi {
                    const REG:*const spis0::RegisterBlock = $spi::PTR;

                    fn state() -> &'static State {
                         static STATE: State = State::new();
                         &STATE
                    }
                }
            )*
        };
    }
    spi!(SPIS0, SPIS1, SPIS2);

    pub trait Setters<const BUFFERSIZE: usize> {
        fn set_rx_buff(&mut self, ptr: *const [u8; BUFFERSIZE]);
        fn set_tx_buff(&mut self, ptr: *mut [u8; BUFFERSIZE]);
        fn underlying(&self) -> &spis0::RegisterBlock;
    }
    pub trait Argument<const BUFFERSIZE: usize> {
        fn apply<I: Setters<BUFFERSIZE>, S: Instance>(&self, instance: &mut I, source: &mut S);
    }

    impl<const CS: bool, const BUFFERSIZE: usize> Argument<BUFFERSIZE>
        for (
            Pins<CS, true, true>,
            // TX Buffer
            *mut [u8; BUFFERSIZE],
            // RX Buffer
            *const [u8; BUFFERSIZE],
        )
    {
        fn apply<I: Setters<BUFFERSIZE>, S: Instance>(&self, instance: &mut I, source: &mut S) {
            self.0.configure(source);
            instance.set_rx_buff(self.2);
            instance.set_tx_buff(self.1);
        }
    }

    impl<const CS: bool, const BUFFERSIZE: usize> Argument<BUFFERSIZE>
        for (
            Pins<CS, true, false>,
            // TX Buffer
            *mut [u8; BUFFERSIZE],
        )
    {
        fn apply<I: Setters<BUFFERSIZE>, S: Instance>(&self, instance: &mut I, source: &mut S) {
            self.0.configure(source);
            instance.set_tx_buff(self.1)
        }
    }

    impl<const CS: bool, const BUFFERSIZE: usize> Argument<BUFFERSIZE>
        for (
            Pins<CS, false, true>,
            // RX Buffer
            *const [u8; BUFFERSIZE],
        )
    {
        fn apply<I: Setters<BUFFERSIZE>, S: Instance>(&self, instance: &mut I, source: &mut S) {
            self.0.configure(source);
            instance.set_rx_buff(self.1)
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: Bitorder::MSB_FIRST,
            over_read_character: 0,
            defualt_byte: 0,
            auto_acquire: true,
        }
    }
}

impl Config {
    fn apply<I: Instance>(self, spi: &mut I) {
        let spi = spi.spi();

        spi.config.write(|w| {
            w.order().variant(self.bit_order);
            match self.mode {
                MODE_0 => {
                    w.cpol().active_high();
                    w.cpha().leading();
                }
                MODE_1 => {
                    w.cpol().active_high();
                    w.cpha().trailing();
                }
                MODE_2 => {
                    w.cpol().active_low();
                    w.cpha().leading();
                }
                MODE_3 => {
                    w.cpol().active_low();
                    w.cpha().trailing();
                }
            }

            unsafe {
                spi.orc.write(|w| w.orc().bits(self.over_read_character));
                spi.def.write(|w| w.def().bits(self.defualt_byte));
                spi.shorts.write(|w| w.end_acquire().bit(self.auto_acquire));
            };

            w
        });
    }
}

impl Pins<false, true, false> {
    pub fn rx(clock: P, mosi: P) -> Self {
        Pins::RX {
            mosi,
            clock,
            chip_select: None,
        }
    }
}
impl Pins<false, false, true> {
    pub fn tx(clock: P, miso: P) -> Self {
        Pins::TX {
            miso,
            clock,
            chip_select: None,
        }
    }
}
impl Pins<false, true, true> {
    pub fn duplex(clock: P, miso: P, mosi: P) -> Self {
        Pins::Duplex {
            miso,
            mosi,
            clock,
            chip_select: None,
        }
    }
}

impl Pins<true, true, false> {
    pub fn rx_cs(clock: P, mosi: P, cs: P) -> Self {
        Pins::RX {
            mosi,
            clock,
            chip_select: Some(cs),
        }
    }
}
impl Pins<true, false, true> {
    pub fn tx_cs(clock: P, miso: P, cs: P) -> Self {
        Pins::TX {
            miso,
            clock,
            chip_select: Some(cs),
        }
    }
}
impl Pins<true, true, true> {
    pub fn duplex_cs(clock: P, miso: P, mosi: P, cs: P) -> Self {
        Pins::Duplex {
            miso,
            mosi,
            clock,
            chip_select: Some(cs),
        }
    }
}

impl<const CS: bool, const RX: bool, const TX: bool> Pins<CS, RX, TX> {
    fn configure_cs<I: Instance>(&self, instance: &mut I) {
        if CS {
            let cs = match self {
                Self::RX {
                    mosi: _,
                    clock: _,
                    chip_select,
                } => chip_select.as_ref().unwrap_or_else(|| unreachable!()),
                Self::TX {
                    miso: _,
                    clock: _,
                    chip_select,
                } => chip_select.as_ref().unwrap_or_else(|| unreachable!()),
                Self::Duplex {
                    mosi: _,
                    miso: _,
                    clock: _,
                    chip_select,
                } => chip_select.as_ref().unwrap_or_else(|| unreachable!()),
            };
            instance
                .spi()
                .psel
                .csn
                .write(|w| w.pin().variant(cs.psel_bits() as u8))
        }
    }
}

impl<const CS: bool, const RX: bool, const TX: bool> Pins<CS, RX, TX> {
    fn configure_mosi<I: Instance>(&self, instance: &mut I) {
        if RX {
            let mosi = match self {
                Self::RX {
                    mosi,
                    clock: _,
                    chip_select: _,
                } => mosi,
                Self::TX {
                    miso: _,
                    clock: _,
                    chip_select: _,
                } => unreachable!(),
                Self::Duplex {
                    mosi,
                    miso: _,
                    clock: _,
                    chip_select: _,
                } => mosi,
            };
            instance
                .spi()
                .psel
                .mosi
                .write(|w| w.pin().variant(mosi.psel_bits() as u8))
        }
    }
}

impl<const CS: bool, const RX: bool, const TX: bool> Pins<CS, RX, TX> {
    fn configure_miso<I: Instance>(&self, instance: &mut I) {
        if TX {
            let miso = match self {
                Self::RX {
                    mosi: _,
                    clock: _,
                    chip_select: _,
                } => unreachable!(),
                Self::TX {
                    miso,
                    clock: _,
                    chip_select: _,
                } => miso,
                Self::Duplex {
                    mosi: _,
                    miso,
                    clock: _,
                    chip_select: _,
                } => miso,
            };
            instance
                .spi()
                .psel
                .miso
                .write(|w| w.pin().variant(miso.psel_bits() as u8))
        }
    }
}

impl<const CS: bool, const RX: bool, const TX: bool> Pins<CS, RX, TX> {
    fn configure<I: Instance>(&self, instance: &mut I) {
        self.configure_cs(instance);
        self.configure_mosi(instance);
        self.configure_miso(instance);
        let clk = match self {
            Self::RX {
                mosi: _,
                clock: _,
                chip_select: _,
            } => unreachable!(),
            Self::TX {
                miso,
                clock: _,
                chip_select: _,
            } => miso,
            Self::Duplex {
                mosi: _,
                miso,
                clock: _,
                chip_select: _,
            } => miso,
        };

        instance
            .spi()
            .psel
            .sck
            .write(|w| w.pin().variant(clk.psel_bits() as u8))
    }
}
