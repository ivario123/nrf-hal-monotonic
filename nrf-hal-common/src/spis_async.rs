#![macro_use]
use core::{future::poll_fn, marker::PhantomData, sync::atomic::compiler_fence, task::Poll};
use embedded_hal_async::spi::*;
use nrf52840_pac::spis0;
use spis0::config::ORDER_A as Bitorder;

use crate::{
    gpio::{Floating, Input, Pin},
    target_constants::EASY_DMA_SIZE,
};

use self::sealed::Instance;

/// This type will litter the codebase.
type P = Pin<Input<Floating>>;

pub enum Pins<const HasCs: bool, const Rx: bool, const Tx: bool> {
    /// only capable of recieving data.
    Rx {
        mosi: P,
        clock: P,
        chip_select: Option<P>,
    },
    /// Only capable of transmitting data.
    Tx {
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

pub struct Spi<I: Instance, const Tx: bool, const Rx: bool, const Cs: bool, const BufferSize: usize>
{
    instance: PhantomData<I>,
    tx: Option<*const [u8; BufferSize]>,
    rx: Option<*const [u8; BufferSize]>,
}

impl<I: Instance, const Cs: bool, const BufferSize: usize> Spi<I, true, false, Cs, BufferSize> {
    pub fn new(
        mut spi: I,
        pins: Pins<Cs, false, true>,
        tx_buffer: *const [u8; BufferSize],
        config: Config,
    ) -> Self {
        if BufferSize > EASY_DMA_SIZE {
            panic!()
        }
        pins.configure(&mut spi);
        config.apply(&mut spi);

        let spi = spi.spi();
        spi.enable.write(|w| w.enable().enabled());

        Spi {
            instance: PhantomData,
            tx: Some(tx_buffer),
            rx: None,
        }
    }
}

impl<I: Instance, const Cs: bool, const BufferSize: usize> Spi<I, false, true, Cs, BufferSize> {
    pub fn new(
        mut spi: I,
        pins: Pins<Cs, true, false>,
        rx_buffer: *const [u8; BufferSize],
        config: Config,
    ) -> Self {
        if BufferSize > EASY_DMA_SIZE {
            panic!()
        }
        pins.configure(&mut spi);
        config.apply(&mut spi);

        let spi = spi.spi();
        spi.enable.write(|w| w.enable().enabled());

        Spi {
            instance: PhantomData,
            tx: None,
            rx: Some(rx_buffer),
        }
    }
}

impl<I: Instance, const Cs: bool, const BufferSize: usize> Spi<I, true, true, Cs, BufferSize> {
    pub fn new(
        mut spi: I,
        pins: Pins<Cs, true, true>,
        tx_buffer: *const [u8; BufferSize],
        rx_buffer: *const [u8; BufferSize],
        config: Config,
    ) -> Self {
        if BufferSize > EASY_DMA_SIZE {
            panic!()
        }
        pins.configure(&mut spi);
        config.apply(&mut spi);
        let spi = spi.spi();

        spi.enable.write(|w| w.enable().enabled());

        Spi {
            instance: PhantomData,
            tx: Some(tx_buffer),
            rx: Some(rx_buffer),
        }
    }
}

impl<I: Instance, const Cs: bool, const Rx: bool, const BufferSize: usize>
    Spi<I, true, Rx, Cs, BufferSize>
{
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

    fn prepare(&mut self) {
        let r = I::const_spi();

        // Set up the DMA write.
        let ptr = self.tx.unwrap_or_else(|| panic!());

        r.txd.ptr.write(|w| unsafe { w.ptr().bits(ptr as _) });
        r.txd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(BufferSize as _) });

        // Set up the DMA read.
        let ptr = self.rx.unwrap_or_else(|| panic!());

        r.txd.ptr.write(|w| unsafe { w.ptr().bits(ptr as _) });
        r.txd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(BufferSize as _) });

        // Reset end event.
        r.events_end.reset();

        // Release the semaphore.
        r.tasks_release.write(|w| unsafe { w.bits(1) });
    }
}

mod sealed {
    use rtic_common::waker_registration::CriticalSectionWakerRegistration;

    use crate::pac::spis0;

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

impl<const Rx: bool, const Tx: bool> Pins<false, Rx, Tx> {
    pub fn rx(clock: P, mosi: P) -> Pins<false, true, false> {
        Pins::Rx {
            mosi,
            clock,
            chip_select: None,
        }
    }
    pub fn tx(clock: P, miso: P) -> Pins<false, false, true> {
        Pins::Tx {
            miso,
            clock,
            chip_select: None,
        }
    }
    pub fn duplex(clock: P, miso: P, mosi: P) -> Pins<false, true, true> {
        Pins::Duplex {
            miso,
            mosi,
            clock,
            chip_select: None,
        }
    }
}

impl<const Rx: bool, const Tx: bool> Pins<true, Rx, Tx> {
    pub fn rx(clock: P, mosi: P, cs: P) -> Pins<true, true, false> {
        Pins::Rx {
            mosi,
            clock,
            chip_select: Some(cs),
        }
    }
    pub fn tx(clock: P, miso: P, cs: P) -> Pins<true, false, true> {
        Pins::Tx {
            miso,
            clock,
            chip_select: Some(cs),
        }
    }
    pub fn duplex(clock: P, miso: P, mosi: P, cs: P) -> Pins<true, true, true> {
        Pins::Duplex {
            miso,
            mosi,
            clock,
            chip_select: Some(cs),
        }
    }
}

impl<const Cs: bool, const Rx: bool, const Tx: bool> Pins<Cs, Rx, Tx> {
    fn configure_cs<I: Instance>(&self, instance: &mut I) {
        if Cs {
            let cs = match self {
                Self::Rx {
                    mosi: _,
                    clock: _,
                    chip_select,
                } => chip_select.as_ref().unwrap_or_else(|| unreachable!()),
                Self::Tx {
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

impl<const Cs: bool, const Rx: bool, const Tx: bool> Pins<Cs, Rx, Tx> {
    fn configure_mosi<I: Instance>(&self, instance: &mut I) {
        if Rx {
            let mosi = match self {
                Self::Rx {
                    mosi,
                    clock: _,
                    chip_select: _,
                } => mosi,
                Self::Tx {
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

impl<const Cs: bool, const Rx: bool, const Tx: bool> Pins<Cs, Rx, Tx> {
    fn configure_miso<I: Instance>(&self, instance: &mut I) {
        if Tx {
            let miso = match self {
                Self::Rx {
                    mosi: _,
                    clock: _,
                    chip_select: _,
                } => unreachable!(),
                Self::Tx {
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

impl<const Cs: bool, const Rx: bool, const Tx: bool> Pins<Cs, Rx, Tx> {
    fn configure<I: Instance>(&self, instance: &mut I) {
        self.configure_cs(instance);
        self.configure_mosi(instance);
        self.configure_miso(instance);
        let clk = match self {
            Self::Rx {
                mosi: _,
                clock: _,
                chip_select: _,
            } => unreachable!(),
            Self::Tx {
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
