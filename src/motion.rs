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

        io.dirx.set_level(if steps_x >= 0.0f32 {
            Level::High
        } else {
            Level::Low
        });

        io.diry.set_level(if steps_y >= 0.0f32 {
            Level::High
        } else {
            Level::Low
        });

        io.dirz.set_level(if steps_z >= 0.0f32 {
            Level::High
        } else {
            Level::Low
        });

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
            io.smx.tx().wait_push(steps_x_abs as u32).await;
            io.smx.tx().wait_push(delay_cycles_x).await;
        }

        if steps_y_abs > 0.0 {
            io.smy.tx().wait_push(steps_y_abs as u32).await;
            io.smy.tx().wait_push(delay_cycles_y).await;
        }

        if steps_z_abs > 0.0 {
            io.smz.tx().wait_push(steps_z_abs as u32).await;
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
