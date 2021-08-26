use cortex_m::peripheral::NVIC;
/// Low resolution timer.
///
/// Meant for measuring start / stop times of events that take a relatively
/// long time to finish.
///
/// This timer assumes a timer clock of `72_000_000` Hz and runs off TIM2.
///
/// The 32 bit milliseconds output overflows every ~24 days.
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

pub struct LrTimer {
    /// Hardware timer associated with this software timer.
    tim: timer::CountDownTimer<pac::TIM2>,
    /// Number of timer updates / overflows.
    updates: u32,
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
    #[inline]
    fn isr_needs_servicing(&self) -> bool {
        NVIC::is_pending(INTERRUPT)
    }

    /// Function to be run on a timer interrupt.
    ///
    /// Users should schedule this to be run on an interrupt from the source
    /// `INTERRUPT`.
    pub fn isr(&mut self) {
        if self.isr_needs_servicing() {
            self.tim.clear_update_interrupt_flag();
            self.updates = self.updates.wrapping_add(1);
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

        (cnt / COUNTS_PER_MILLISECOND) as u32 + (updates * MILLISECONDS_PER_UPDATE)
    }
}
