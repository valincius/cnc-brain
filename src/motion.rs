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
use libm::{fabsf, floorf, hypotf};

use crate::{Irqs, StepperResources};

static CURRENT_STATE: Mutex<CriticalSectionRawMutex, RefCell<MotionState>> =
    Mutex::new(RefCell::new(MotionState::new()));

pub static STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static MOTION_SIGNAL: Signal<CriticalSectionRawMutex, MotionState> = Signal::new();
pub static MOTION_QUEUE: Channel<CriticalSectionRawMutex, MotionCommand, 128> = Channel::new();

pub async fn execute_motion(sm0: &mut StateMachine<'_, PIO0, 0>, command: MotionCommand) {
    let mut state = { CURRENT_STATE.lock().await.borrow().clone() };

    let accel = 100_000.0f32;

    let micro_steps = 16;

    let steps_per_mm = [267.0f32, 267.0f32, 267.0f32];
    let steps_per_mm = [
        steps_per_mm[0] * micro_steps as f32,
        steps_per_mm[1] * micro_steps as f32,
        steps_per_mm[2] * micro_steps as f32,
    ];

    let [mut x0, mut y0, mut z0] = state.current_pos;

    let (x1, y1, z1, feed_rate) = match command {
        MotionCommand::Linear([x, y, z], feed_rate) => (x, y, z, feed_rate),
    };

    let dx = x1 - x0;
    let dy = y1 - y0;
    let dz = z1 - z0;

    let distance = dist3d(x0, y0, z0, x1, y1, z1);

    let mut traveled = 0.0;
    let mut current_vel = 0.0; // or carry over from prior block
    let max_vel = feed_rate; // in, say, mm/s
    let dt = 0.0001; // 100us
    let mut is_decelerating = false;

    let epsilon = 0.001; // mm

    let mut delay_accum_x = 0.0f32;
    let mut delay_accum_y = 0.0f32;
    let mut delay_accum_z = 0.0f32;

    while traveled < distance - epsilon {
        let dist_left = distance - traveled;

        if is_decelerating {
            current_vel -= accel * dt;
            if current_vel < 0.0 {
                break;
            }
        } else {
            let decel_dist = (current_vel * current_vel) / (2.0 * accel);
            if decel_dist >= dist_left {
                is_decelerating = true;
            } else if current_vel < max_vel {
                current_vel += accel * dt;
                if current_vel > max_vel {
                    current_vel = max_vel;
                }
            }
        }

        let delta_dist = current_vel * dt;
        traveled += delta_dist;

        let frac = delta_dist / distance;

        let step_x_mm = dx * frac;
        let step_y_mm = dy * frac;
        let step_z_mm = dz * frac;

        let steps_x = step_x_mm * steps_per_mm[0];
        let steps_y = step_y_mm * steps_per_mm[1];
        let steps_z = step_z_mm * steps_per_mm[2];

        x0 += step_x_mm;
        y0 += step_y_mm;
        z0 += step_z_mm;

        state.current_speed = current_vel;
        state.current_pos = [x0, y0, z0];

        let _dir_x = if steps_x >= 0.0f32 { 1 } else { 0 };
        let _dir_y = if steps_y >= 0.0f32 { 1 } else { 0 };
        let _dir_z = if steps_z >= 0.0f32 { 1 } else { 0 };

        let steps_x_abs = fabsf(steps_x);
        let steps_y_abs = fabsf(steps_y);
        let steps_z_abs = fabsf(steps_z);

        let rate_x = (steps_x_abs as f32) / dt; // steps/s
        let rate_y = (steps_y_abs as f32) / dt; // steps/s
        let rate_z = (steps_z_abs as f32) / dt; // steps/s

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

        if steps_x_abs > 0.0 {
            sm0.tx().wait_push(steps_x_abs as u32).await;
            sm0.tx().wait_push(delay_cycles_x).await;
        }

        {
            CURRENT_STATE.lock().await.replace(state.clone());
        }

        MOTION_SIGNAL.signal(state.clone());
    }
}

#[embassy_executor::task]
pub async fn motion_task(r: StepperResources) {
    let Pio {
        mut common,
        mut sm0,
        ..
    } = Pio::new(r.pio, Irqs);

    let stepper_prog = pio_proc::pio_asm!(
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

    // let x_step = Output::new(x_step, Level::Low);
    let x_step = common.make_pio_pin(r.x_step);

    let _x_dir = Output::new(r.x_dir, Level::Low);

    let mut step_pio_cfg = embassy_rp::pio::Config::default();
    step_pio_cfg.set_set_pins(&[&x_step]);
    step_pio_cfg.use_program(&stepper_prog, &[]);
    step_pio_cfg.clock_divider = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles

    sm0.set_config(&step_pio_cfg);
    sm0.set_enable(true);

    loop {
        let command = MOTION_QUEUE.receive().await;

        match select::select(execute_motion(&mut sm0, command), STOP_SIGNAL.wait()).await {
            select::Either::First(_) => {
                log::info!("Motion done");
            }
            select::Either::Second(_) => {
                log::info!("Motion stopped");
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct MotionState {
    current_pos: [f32; 3], // Current position of each axis
    current_speed: f32,
}

impl MotionState {
    pub const fn new() -> Self {
        Self {
            current_pos: [0.0; 3],
            current_speed: 0.0,
        }
    }
}

#[derive(Clone)]
pub enum MotionCommand {
    Linear([f32; 3], f32),
}

fn dist3d(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> f32 {
    hypotf(hypotf(x1 - x0, y1 - y0), z1 - z0)
}
