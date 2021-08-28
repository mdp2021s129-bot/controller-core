/// Board motion control capabilities.
use embedded_hal::{digital::v2::OutputPin, Pwm, Qei};
use fixed::FixedI32;
// Micromath is acceptable for the operations performed in this module.
#[allow(unused_imports)]
use micromath::F32Ext as _;
use qei::QeiManager;
use stm32f1xx_hal::{pwm::Channel, time::Hertz};

/// Q17_15 fixed point type.
pub type Q17_15 = FixedI32<fixed::types::extra::U15>;

/// Servo angle.
///
/// -1: Lower limit.
/// 0: Neutral.
/// 1: Upper limit.
pub type Angle = Q17_15;

/// Motor PWM duty cycle.
/// -1: Full reverse.
/// 0: stop (brake).
/// 1: Full forward.
pub type Duty = Q17_15;

/// Models the vehicle's steering (backed by a TD8120MG servo).
pub struct Steering<T: Pwm> {
    pwm: T,
    channel: T::Channel,
    min_duty: T::Duty,
    max_duty: T::Duty,
    neutral_duty: T::Duty,
}

impl<T: Pwm<Time = Hertz>> Steering<T> {
    /// Frequency at which to drive the servo.
    ///
    /// TD8120MG pulse width range is [500, 2500] usec,
    /// so we drive it at 200 Hz in order to ensure we can access the full
    /// motion range without having 0% duty or 100% duty.
    ///
    /// 200 Hz should be doable for a digital servo.
    const FREQUENCY: Hertz = Hertz(200);
}

impl<T: Pwm<Channel = Channel, Duty = u16, Time = Hertz>> Steering<T> {
    /// Creates a new servo driver backed by a PWM generator.
    ///
    /// Also resets the servo to its neutral position.
    pub fn new(mut pwm: T, channel: T::Channel) -> Self {
        let seconds_per_duty: f32 =
            (1.0_f32 / Self::FREQUENCY.0 as f32) / (pwm.get_max_duty() as f32);
        let min_duty = 500e-6_f32 / seconds_per_duty;
        let max_duty = 2500e-6_f32 / seconds_per_duty;
        let neutral_duty = (min_duty + max_duty) / 2.0;

        let min_duty = min_duty.ceil() as T::Duty;
        let max_duty = max_duty.floor() as T::Duty;
        let neutral_duty = neutral_duty.round() as T::Duty;

        pwm.disable(channel);
        pwm.set_period(Self::FREQUENCY);
        pwm.set_duty(channel, neutral_duty);
        pwm.enable(channel);

        Self {
            pwm,
            channel,
            min_duty,
            max_duty,
            neutral_duty,
        }
    }

    /// Drives the servo to the given angle.
    pub fn set(&mut self, angle: Angle) {
        if angle > 0_i16 {
            // FIXME: remove after checking.
            self.pwm.set_duty(
                self.channel,
                (Angle::from(self.max_duty - self.neutral_duty) * angle.abs())
                    .checked_to_num::<T::Duty>()
                    .unwrap()
                    + self.neutral_duty,
            )
        } else {
            self.pwm.set_duty(
                self.channel,
                self.neutral_duty
                    - ((Angle::from(self.neutral_duty - self.min_duty) * angle.abs())
                        .checked_to_num::<T::Duty>()
                        .unwrap()),
            )
        }
    }
}

/// Structure modelling a set of `TB6612FNG` control pins.
///
/// Assumes that pin I/O operations never fail.
struct TB6612FNGControlPins<P: OutputPin> {
    in1: P,
    in2: P,
}

impl<P: OutputPin> TB6612FNGControlPins<P> {
    /// Creates a new set of control pins from digital outputs controlling
    /// `in1` & `in2` as an array `[in1, in2]`.
    fn new(ins: [P; 2]) -> Self {
        let [in1, in2] = ins;

        Self { in1, in2 }
    }

    /// Commands the driver to brake the motor.
    fn brake(&mut self) {
        self.in1.set_high().ok();
        self.in2.set_high().ok();
    }

    /// Commands the driver to move the motor in the clockwise direction.
    fn cw(&mut self) {
        self.in1.set_high().ok();
        self.in2.set_low().ok();
    }

    /// Commands the driver to move the motor in the counter-clockwise
    /// direction.
    fn ccw(&mut self) {
        self.in1.set_low().ok();
        self.in2.set_high().ok();
    }

    /// Commands the driver to let the motor coast.
    fn coast(&mut self) {
        self.in1.set_low().ok();
        self.in2.set_low().ok();
    }
}

/// Enumeration across all the wheels of the chassis.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Wheel {
    /// The left wheel.
    LEFT = 0,
    /// The right wheel.
    RIGHT,
}

impl Wheel {
    /// Obtain the index of the wheel.
    pub fn index(self) -> usize {
        self as usize
    }
}

/// Models the TB6612FNG drive motors and encoders.
pub struct Wheels<T: Pwm, Q1: Qei, Q2: Qei, P: OutputPin> {
    pwm: T,
    ins: [TB6612FNGControlPins<P>; 2],
    channels: [T::Channel; 2],
    encoders: (QeiManager<Q1>, QeiManager<Q2>),
    max_duty: Duty,
}

impl<
        T: Pwm<Duty = u16, Channel = Channel>,
        Q1: Qei<Count = u16>,
        Q2: Qei<Count = u16>,
        P: OutputPin,
    > Wheels<T, Q1, Q2, P>
{
    /// Instantiates a new `Wheels` representation.
    ///
    /// `ins` is an array of  `[inA, inB]` pins.
    /// The same goes for all the other arrays.
    /// Index `0` must correspond to resources on the left side of the robot.
    ///
    /// The motors are left in the braked state after this function returns.
    pub fn new(
        mut pwm: T,
        period: T::Time,
        ins: [[P; 2]; 2],
        channels: [T::Channel; 2],
        encoders: (Q1, Q2),
    ) -> Self {
        pwm.disable(channels[0]);
        pwm.disable(channels[1]);
        pwm.set_period(period);
        pwm.enable(channels[0]);
        pwm.enable(channels[1]);

        let [insl, insr] = ins;
        let (encl, encr) = encoders;
        let max_duty = pwm.get_max_duty().into();

        let mut out = Self {
            pwm,
            ins: [
                TB6612FNGControlPins::new(insl),
                TB6612FNGControlPins::new(insr),
            ],
            channels,
            encoders: (QeiManager::new(encl), QeiManager::new(encr)),
            max_duty,
        };

        out.drive(Wheel::LEFT, 0_u16.into());
        out.drive(Wheel::RIGHT, 0_u16.into());
        out
    }

    /// Obtain the PWM resolution.
    pub fn resolution(&self) -> T::Duty {
        self.pwm.get_max_duty()
    }

    /// Command a motor to coast.
    pub fn coast(&mut self, which: Wheel) {
        self.ins[which.index()].coast()
    }

    /// Command a motor to be driven in a given direction at a provided
    /// duty cycle.
    ///
    /// If `duty == 0`, the motor is actively braked.
    pub fn drive(&mut self, which: Wheel, duty: Duty) {
        let control = &mut self.ins[which.index()];
        if duty != 0 {
            if duty > 0 {
                control.cw();
            } else {
                control.ccw();
            }
        } else {
            control.brake();
        }

        self.pwm.set_duty(
            self.channels[which.index()],
            (duty.abs() * self.max_duty).checked_to_num().unwrap(),
        );
    }

    /// Reads the positions of both motors' output shafts, while updating the
    /// internal position counter to deal with hardware encoder counter
    /// overflow.
    ///
    /// The position is given in terms of encoder counts.
    ///
    /// `[0]` is the position of the left motor's shaft and `[1]` is the
    /// position of the right one.
    ///
    /// Must be called periodically to avoid sampling errors.
    pub fn read_and_update_positions(&mut self) -> Result<[i64; 2], qei::SamplingError> {
        self.encoders
            .0
            .sample()
            .and_then(|_| self.encoders.1.sample())
            .map(|_| self.read_positions())
    }

    /// Does the same as `read_and_update_positions`, except that the last
    /// cached value is output instead.
    pub fn read_positions(&self) -> [i64; 2] {
        [self.encoders.0.count(), self.encoders.1.count()]
    }
}
