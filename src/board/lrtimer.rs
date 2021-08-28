use cortex_m::peripheral::NVIC;
/// Low resolution timer.
///
/// Meant for measuring start / stop times of events that take a relatively
/// long time to finish.
///
/// This timer assumes a timer clock of `72_000_000` Hz and runs off TIM2.
///
/// The 32 bit milliseconds output overflows every ~49 days.
use stm32f1xx_hal::{pac, rcc::Clocks, timer};

/// Timer reload value.
pub const RELOAD_VALUE: u16 = 0xffff;

/// Timer prescaler value.
// 36000 ticks = 1 timer tick
// -> 72MHz / 36000 -> 2000Hz -> 0.5ms per tick.
pub const PRESCALER_VALUE: u16 = 35999;

/// Expected `TIM2CLK` frequency.
///
/// Expressed in units of Hertz.
pub const TIM2CLK_EXPECTED_HZ: u32 = 72_000_000;

/// Interrupt that is bound to this timer.
pub const INTERRUPT: pac::Interrupt = pac::Interrupt::TIM2;

/// Number of milliseconds per counter update.
pub const MILLISECONDS_PER_UPDATE: u32 = 0x10000 / 2;

/// Number timer ticks per millisecond.
///
/// Also indirectly specifies the timer resolution.
pub const COUNTS_PER_MILLISECOND: u16 = 2;

/// Timer instant type.
pub type Instant = embedded_time::Instant<LrTimer>;

pub struct LrTimer {
    /// Hardware timer associated with this software timer.
    tim: timer::CountDownTimer<pac::TIM2>,
    /// Number of timer updates / overflows.
    updates: u32,
}

impl core::fmt::Debug for LrTimer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LrTimer")
            .field("updates", &self.updates)
            .finish_non_exhaustive()
    }
}

impl LrTimer {
    /// Creates a new LrTimer.
    ///
    /// Panics if the timer frequency mismatches.
    ///
    /// The timer is running after this function returns.
    pub fn new(tim: pac::TIM2, clocks: &Clocks) -> Self {
        assert!(clocks.pclk1_tim().0 == TIM2CLK_EXPECTED_HZ);

        let mut timer = timer::Timer::tim2(tim, clocks).start_raw(PRESCALER_VALUE, RELOAD_VALUE);
        // Not sound: this is a bit of a race - but it works if we use it in
        // RTIC.
        timer.listen(timer::Event::Update);

        Self {
            tim: timer,
            updates: 0,
        }
    }

    /// Determines if the timer interrupt needs to be serviced.
    ///
    /// Assumes that the timer is running & the timer is setup for interrupts
    /// to be generated.
    ///
    /// Should be called in a section with `INTERRUPT` disabled.
    #[inline]
    fn isr_needs_servicing(&self) -> bool {
        NVIC::is_pending(INTERRUPT) || NVIC::is_active(INTERRUPT)
    }

    /// Calculates the number of milliseconds given a counter value and an
    /// update count.
    fn calculate_ms(updates: u32, cnt: u16) -> u32 {
        updates
            .wrapping_mul(MILLISECONDS_PER_UPDATE)
            .wrapping_add((cnt / COUNTS_PER_MILLISECOND) as u32)
    }

    /// Function to be run on a timer interrupt.
    ///
    /// Users should schedule this to be run on an interrupt from the source
    /// `INTERRUPT`.
    pub fn isr(&mut self) {
        if self.isr_needs_servicing() {
            self.tim.clear_update_interrupt_flag();
            self.updates = self.updates.wrapping_add(1);
            NVIC::unpend(INTERRUPT);
        }
    }

    /// Obtain the timer's value, in units of milliseconds since it was
    /// created.
    pub fn ms(&mut self) -> u32 {
        let cnt = self.tim.cnt();
        let updates = self.updates;

        let (cnt, updates) = if self.isr_needs_servicing() {
            self.isr();
            (self.tim.cnt(), self.updates)
        } else {
            (cnt, updates)
        };

        Self::calculate_ms(updates, cnt)
    }

    /// Equivalent to `ms()`, with the added exception that an error can be
    /// returned if the timer overflows.
    ///
    /// This function disables all interrupts for a short while when reading timer
    /// registers & the timer interrupt pending flag.
    ///
    /// `isr()` should be called before `ms_no_update()` is retried.
    pub fn ms_no_update(&self) -> Result<u32, ()> {
        let (cnt, updates, needs_servicing) = cortex_m::interrupt::free(|_| {
            (self.tim.cnt(), self.updates, self.isr_needs_servicing())
        });

        if needs_servicing {
            Err(())
        } else {
            Ok(Self::calculate_ms(updates, cnt))
        }
    }

    /// Retrieves the current timer value.
    ///
    /// This function disables all interrupts for a short while when reading timer
    /// registers & the timer interrupt pending flag.
    ///
    /// It's essentially an automatically-retrying version of `Clock::try_now()`.
    pub fn now(&mut self) -> Instant {
        loop {
            match self.try_now() {
                Ok(now) => return now,
                Err(_) => self.isr(),
            }
        }
    }
}

use embedded_time::{clock::*, duration::*};

impl Clock for LrTimer {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1000);

    /// Try to obtain the current time.
    ///
    /// If `Err(Error::NotSpecified)` is returned, then `isr()` must be called
    /// on the timer before retrying (either through an ISR / manually if in a
    /// context where that cannot be called).
    ///
    /// This function disables all interrupts for a short while when reading timer
    /// registers & the timer interrupt pending flag.
    fn try_now(&self) -> Result<Instant, Error> {
        self.ms_no_update()
            .map(Instant::new)
            .map_err(|_| Error::Unspecified)
    }
}
