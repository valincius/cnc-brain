use core::cell::RefCell;

use embassy_futures::select;
use embassy_rp::{
    gpio::{Level, Output},
    peripherals::PIO0,
    pio::{Pio, StateMachine},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use fixed::traits::ToFixed as _;
use libm::{fabsf, floorf, hypotf, powf, sqrtf};

use crate::{Irqs, StepperResources};

static CURRENT_STATE: Mutex<CriticalSectionRawMutex, RefCell<MotionState>> =
    Mutex::new(RefCell::new(MotionState::new()));

pub static STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static MOTION_SIGNAL: Signal<CriticalSectionRawMutex, MotionState> = Signal::new();
pub static MOTION_QUEUE: Channel<CriticalSectionRawMutex, MotionCommand, 128> = Channel::new();

#[derive(Debug, Clone)]
pub struct MotionState {
    current_pos: [f32; 3], // Current position of each axis
    current_velocity: f32,

    target_pos: [f32; 3],
    target_velocity: f32,

    distance_remaining: f32,
    current_acceleration: f32,
    deceleration_phase: bool,
    effective_jerk: f32,
    traveled_distance: f32,
}

impl MotionState {
    pub const fn new() -> Self {
        Self {
            current_pos: [0.0; 3],
            current_velocity: 0.0,

            target_pos: [0.0; 3],
            target_velocity: 0.0,

            distance_remaining: 0.0,
            current_acceleration: 0.0,
            deceleration_phase: false,
            effective_jerk: 0.0,
            traveled_distance: 0.0,
        }
    }
}

#[embassy_executor::task]
pub async fn motion_task(r: StepperResources) {
    let mut io = MotionIO::from(r);

    loop {
        let command = MOTION_QUEUE.receive().await;

        match select::select(execute_motion(&mut io, command), STOP_SIGNAL.wait()).await {
            select::Either::First(_) => {
                log::info!("Motion done");
            }
            select::Either::Second(_) => {
                log::info!("Motion stopped");
            }
        }
    }
}

async fn execute_motion(io: &mut MotionIO<'_>, command: MotionCommand) {
    // Acquire current motion state.
    let mut state = { CURRENT_STATE.lock().await.borrow().clone() };
    let [mut x0, mut y0, mut z0] = state.current_pos;

    if command == MotionCommand::Zero {
        // Reset position to zero.
        state.current_pos = [0.0, 0.0, 0.0];
        state.current_velocity = 0.0;
        state.target_pos = [0.0, 0.0, 0.0];
        state.target_velocity = 0.0;
        state.distance_remaining = 0.0;
        state.current_acceleration = 0.0;
        state.deceleration_phase = false;
        state.effective_jerk = 0.0;
        state.traveled_distance = 0.0;

        {
            CURRENT_STATE.lock().await.replace(state.clone());
        }
        MOTION_SIGNAL.signal(state.clone());
        return;
    }

    // === Motion Profile Parameters (units: mm and s) ===
    const MAX_ACCEL: f32 = 20_000.0 * 60.0 * 25.4; // Maximum acceleration (i/m²)
    const JERK: f32 = 10_000.0 * 60.0 * 25.4; // Jerk limit (mm/s³)
    const DT: f32 = 0.0001; // Timestep: 100 µs
    const EPSILON: f32 = 0.001; // Distance tolerance (mm)

    // --- Boost parameters for very low speeds ---
    const LOW_VEL_THRESHOLD: f32 = 0.01; // Below 0.01 mm/s, apply boost
    const BOOST_FACTOR: f32 = 10.0; // Boost factor for jerk

    // Micro-stepping and conversion factors.
    let micro_steps = 1;
    let base_steps = [267.0, 267.0, 267.0];
    let steps_per_mm = [
        base_steps[0] * micro_steps as f32,
        base_steps[1] * micro_steps as f32,
        base_steps[2] * micro_steps as f32,
    ];

    // === Initial and Target Positions ===
    let (x1, y1, z1, feed_rate) = match command {
        MotionCommand::Linear(target, feed_rate) => (target[0], target[1], target[2], feed_rate),
        _ => panic!("Invalid command"),
    };

    state.target_pos = [x1, y1, z1];

    // Compute move vector and total Euclidean distance.
    let dx = x1 - x0;
    let dy = y1 - y0;
    let dz = z1 - z0;
    let total_distance = dist3d(x0, y0, z0, x1, y1, z1);

    // === Motion State Variables ===
    let mut traveled_distance = 0.0;
    let mut current_velocity = 0.0; // mm/s
    let target_velocity = feed_rate; // desired cruising speed (mm/s)

    // S‑curve state: start with zero acceleration.
    let mut current_acceleration = 0.0;
    let mut deceleration_phase = false;

    // Variables for accumulating motor command delays.
    let mut delay_accum_x = 0.0;
    let mut delay_accum_y = 0.0;
    let mut delay_accum_z = 0.0;

    // --- Helper Function for Deceleration Distance ---
    // For a fully jerk-limited S-curve (no constant acceleration phase),
    // the deceleration distance from a speed v is approximated by:
    //     d_est = (4/3) * (v^(3/2)) / sqrt(j)
    // (Units: mm)
    fn estimate_decel_distance(vel: f32, jerk: f32) -> f32 {
        (4.0 / 3.0) * powf(vel, 1.5) / sqrtf(jerk)
    }

    // Clamp function.
    fn clamp(val: f32, min: f32, max: f32) -> f32 {
        if val < min {
            min
        } else if val > max {
            max
        } else {
            val
        }
    }

    // === Main Motion Loop ===
    while traveled_distance < total_distance - EPSILON {
        let distance_remaining = total_distance - traveled_distance;

        // Trigger deceleration when the remaining distance is less than the
        // estimated deceleration distance.
        if estimate_decel_distance(current_velocity, JERK) >= distance_remaining {
            deceleration_phase = true;
        }

        // --- Determine Effective Jerk ---
        let effective_jerk = if current_velocity < LOW_VEL_THRESHOLD {
            JERK * BOOST_FACTOR
        } else {
            JERK
        };

        // --- Update Acceleration ---
        if deceleration_phase {
            // Compute the desired deceleration using the constant deceleration formula.
            // a_desired = -v^2 / (2 * d_remaining)
            // (Be sure to avoid division by zero.)
            let desired_accel = if distance_remaining > EPSILON {
                -(current_velocity * current_velocity) / (2.0 * distance_remaining)
            } else {
                -MAX_ACCEL
            };
            // Clamp desired_accel so that it doesn't exceed -MAX_ACCEL.
            let desired_accel = clamp(desired_accel, -MAX_ACCEL, 0.0);

            // Gradually update current_acceleration toward desired_accel using effective_jerk.
            if current_acceleration > desired_accel {
                current_acceleration -= effective_jerk * DT;
                if current_acceleration < desired_accel {
                    current_acceleration = desired_accel;
                }
            } else {
                current_acceleration += effective_jerk * DT;
                if current_acceleration > desired_accel {
                    current_acceleration = desired_accel;
                }
            }
        } else {
            // Acceleration phase: ramp up normally.
            current_acceleration += effective_jerk * DT;
            if current_acceleration > MAX_ACCEL {
                current_acceleration = MAX_ACCEL;
            }
        }

        // --- Update Velocity ---
        current_velocity += current_acceleration * DT;
        if current_velocity > target_velocity {
            current_velocity = target_velocity;
            current_acceleration = 0.0; // Maintain cruise speed.
        }
        // (Do not force current_velocity to zero—allow it to approach zero smoothly.)
        // if current_velocity < 0.0 { current_velocity = 0.0; }  <-- removed for smooth deceleration

        // --- Update Position ---
        let delta_distance = current_velocity * DT;
        traveled_distance += delta_distance;
        // Fraction of the total move for this timestep.
        let step_fraction = delta_distance / total_distance;
        let step_x_mm = dx * step_fraction;
        let step_y_mm = dy * step_fraction;
        let step_z_mm = dz * step_fraction;

        // Convert move (mm) to steps.
        let steps_x = step_x_mm * steps_per_mm[0];
        let steps_y = step_y_mm * steps_per_mm[1];
        let steps_z = step_z_mm * steps_per_mm[2];

        // Update positions.
        x0 += step_x_mm;
        y0 += step_y_mm;
        z0 += step_z_mm;

        state.current_velocity = current_velocity;
        state.current_pos = [x0, y0, z0];

        state.distance_remaining = distance_remaining;
        state.current_acceleration = current_acceleration;
        state.deceleration_phase = deceleration_phase;
        state.effective_jerk = effective_jerk;
        state.traveled_distance = traveled_distance;
        state.target_velocity = target_velocity;

        // --- Motor IO ---
        // Set motor directions.
        io.dirx.set_level(if steps_x >= 0.0 {
            Level::High
        } else {
            Level::Low
        });
        io.diry.set_level(if steps_y >= 0.0 {
            Level::High
        } else {
            Level::Low
        });
        io.dirz.set_level(if steps_z >= 0.0 {
            Level::High
        } else {
            Level::Low
        });

        // Compute absolute step values and step rates.
        let abs_steps_x = fabsf(steps_x);
        let abs_steps_y = fabsf(steps_y);
        let abs_steps_z = fabsf(steps_z);

        let rate_x = abs_steps_x / DT; // steps/s
        let rate_y = abs_steps_y / DT;
        let rate_z = abs_steps_z / DT;

        // Compute delay periods (hardware-specific constants).
        let step_period_x = (1200 * 200_000) as f32 / rate_x;
        let step_period_y = (1200 * 200_000) as f32 / rate_y;
        let step_period_z = (1200 * 200_000) as f32 / rate_z;

        delay_accum_x += step_period_x;
        let delay_cycles_x = floorf(delay_accum_x) as u32;
        delay_accum_x -= delay_cycles_x as f32;

        delay_accum_y += step_period_y;
        let delay_cycles_y = floorf(delay_accum_y) as u32;
        delay_accum_y -= delay_cycles_y as f32;

        delay_accum_z += step_period_z;
        let delay_cycles_z = floorf(delay_accum_z) as u32;
        delay_accum_z -= delay_cycles_z as f32;

        if abs_steps_x > 0.0 {
            io.smx.tx().wait_push(abs_steps_x as u32).await;
            io.smx.tx().wait_push(delay_cycles_x).await;
        }
        if abs_steps_y > 0.0 {
            io.smy.tx().wait_push(abs_steps_y as u32).await;
            io.smy.tx().wait_push(delay_cycles_y).await;
        }
        if abs_steps_z > 0.0 {
            io.smz.tx().wait_push(abs_steps_z as u32).await;
            io.smz.tx().wait_push(delay_cycles_z).await;
        }

        {
            CURRENT_STATE.lock().await.replace(state.clone());
        }
        MOTION_SIGNAL.signal(state.clone());
    }
}

struct MotionIO<'a> {
    smx: StateMachine<'a, PIO0, 0>,
    smy: StateMachine<'a, PIO0, 1>,
    smz: StateMachine<'a, PIO0, 2>,

    dirx: Output<'a>,
    diry: Output<'a>,
    dirz: Output<'a>,
}

impl<'a> MotionIO<'a> {
    fn from(resources: StepperResources) -> Self {
        let Pio {
            mut common,
            sm0: mut smx,
            sm1: mut smy,
            sm2: mut smz,
            ..
        } = Pio::new(resources.pio, Irqs);

        let stepper_prog = pio::pio_asm!(
            "set pindirs 1"

            ".wrap_target"

            "pull"
            "mov x osr" // Set X to number of steps
            "pull"
            "mov y osr" // Set Y to delay between steps
            "mov isr y" // Copy to ISR

            "step:"
            "set pins 1 [1]"
            "set pins 0"

            "mov y isr" // Reload delay
            "delay:"
            "jmp y-- delay"

            "jmp x-- step"

            ".wrap"
        );

        let stepper_prog = common.load_program(&stepper_prog.program);

        let x_step = common.make_pio_pin(resources.x_step);
        let y_step = common.make_pio_pin(resources.y_step);
        let z_step = common.make_pio_pin(resources.z_step);

        let mut cfgx = embassy_rp::pio::Config::default();
        cfgx.use_program(&stepper_prog, &[]);
        cfgx.clock_divider = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles
        cfgx.set_set_pins(&[&x_step]);
        smx.set_config(&cfgx);

        let mut cfgy = embassy_rp::pio::Config::default();
        cfgy.use_program(&stepper_prog, &[]);
        cfgy.clock_divider = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles
        cfgy.set_set_pins(&[&y_step]);
        smy.set_config(&cfgy);

        let mut cfgz = embassy_rp::pio::Config::default();
        cfgz.use_program(&stepper_prog, &[]);
        cfgz.clock_divider = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles
        cfgz.set_set_pins(&[&z_step]);
        smz.set_config(&cfgz);

        smx.set_enable(true);
        smy.set_enable(true);
        smz.set_enable(true);

        Self {
            dirx: Output::new(resources.x_dir, Level::Low),
            diry: Output::new(resources.y_dir, Level::Low),
            dirz: Output::new(resources.z_dir, Level::Low),

            smx,
            smy,
            smz,
        }
    }
}

#[derive(Clone, PartialEq)]
pub enum MotionCommand {
    Linear([f32; 3], f32),
    Zero,
}

fn dist3d(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> f32 {
    hypotf(hypotf(x1 - x0, y1 - y0), z1 - z0)
}
