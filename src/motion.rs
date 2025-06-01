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
use libm::{hypotf, powf, sqrtf};

use crate::{Irqs, StepperResources};

static CURRENT_STATE: Mutex<CriticalSectionRawMutex, RefCell<MotionState>> =
    Mutex::new(RefCell::new(MotionState::new()));

pub static STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static MOTION_SIGNAL: Signal<CriticalSectionRawMutex, MotionState> = Signal::new();
pub static MOTION_QUEUE: Channel<CriticalSectionRawMutex, MotionCommand, 128> = Channel::new();

const DT: f32 = 0.001; // 1ms
const EPS: f32 = 1e-6;
const PIO_CLOCK_HZ: u32 = 125_000_000;

#[derive(Debug, Clone)]
pub struct MotionState {
    current_pos: [f32; 3], // Current position of each axis
    current_velocity: f32,

    target_pos: [f32; 3],
    target_velocity: f32,

    distance_remaining: f32,
    current_acceleration: f32,
    deceleration_phase: bool,

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

            traveled_distance: 0.0,
        }
    }
}

#[embassy_executor::task]
pub async fn motion_task(r: StepperResources) {
    let mut io = MotionIO::from(r);

    loop {
        let cmd = MOTION_QUEUE.receive().await;
        log::info!("Received command: {:?}", cmd);

        match cmd {
            MotionCommand::Zero => {
                let zeroed = MotionState::new();
                CURRENT_STATE.lock().await.replace(zeroed.clone());
                MOTION_SIGNAL.signal(zeroed);
                log::info!("Homed to zero");
            }

            MotionCommand::MoveAbsolute(target, feed) => {
                move_absolute(&mut io, target, feed).await;
            }

            MotionCommand::MoveRelative(offset, feed) => {
                move_relative(&mut io, offset, feed).await;
            }

            MotionCommand::Jog(offset, feed) => {
                jog(&mut io, offset, feed).await;
            }

            MotionCommand::Stop => {
                STOP_SIGNAL.signal(());
            }
        }
    }
}

async fn jog(io: &mut MotionIO<'_>, offset: [f32; 3], feed_rate: f32) {
    let state = CURRENT_STATE.lock().await.borrow().clone();
    let feed_rate = feed_rate.max(state.target_velocity);

    log::info!(
        "Jogging by offset: ({}, {}, {}), feed_rate: {}",
        offset[0],
        offset[1],
        offset[2],
        feed_rate
    );

    let [x0, y0, z0] = state.target_pos;
    let target = [x0 + offset[0], y0 + offset[1], z0 + offset[2]];

    move_absolute(io, target, feed_rate).await;
}

async fn move_relative(io: &mut MotionIO<'_>, offset: [f32; 3], feed_rate: f32) {
    if feed_rate <= 0.0 {
        log::warn!("Invalid feed rate: {}", feed_rate);
        return;
    }

    log::info!(
        "Moving relative by offset: ({}, {}, {}), feed_rate: {}",
        offset[0],
        offset[1],
        offset[2],
        feed_rate
    );

    let state = CURRENT_STATE.lock().await.borrow().clone();
    let [x0, y0, z0] = state.current_pos;
    let target = [x0 + offset[0], y0 + offset[1], z0 + offset[2]];

    move_absolute(io, target, feed_rate).await;
}

async fn move_absolute(io: &mut MotionIO<'_>, target: [f32; 3], feed_rate: f32) {
    if feed_rate <= 0.0 {
        log::warn!("Invalid feed rate: {}", feed_rate);
        return;
    }

    log::info!(
        "Moving absolute to target: ({}, {}, {}), feed_rate: {}",
        target[0],
        target[1],
        target[2],
        feed_rate
    );

    match select::select(_move_absolute(io, target, feed_rate), STOP_SIGNAL.wait()).await {
        select::Either::First(_) => {
            // Motion completed
            let state = CURRENT_STATE.lock().await;
            let mut s = state.borrow_mut();
            s.current_pos = target;
            s.current_velocity = 0.0;
            s.distance_remaining = 0.0;
            s.traveled_distance = 0.0;

            log::info!(
                "Arrived at target: ({}, {}, {}), feed_rate: {}",
                target[0],
                target[1],
                target[2],
                feed_rate
            );
        }

        select::Either::Second(_) => {
            // Motion was stopped
            log::info!(
                "Motion stopped before reaching target: ({}, {}, {})",
                target[0],
                target[1],
                target[2]
            );
            let state = CURRENT_STATE.lock().await;
            let mut s = state.borrow_mut();
            s.current_velocity = 0.0;
            s.distance_remaining = 0.0;
            s.traveled_distance = 0.0;
        }
    }
}

async fn _move_absolute(io: &mut MotionIO<'_>, target: [f32; 3], feed_rate: f32) {
    {
        let state = CURRENT_STATE.lock().await;
        let mut state = state.borrow_mut();
        state.target_pos = target;
        state.target_velocity = feed_rate;
    }

    let state = CURRENT_STATE.lock().await.borrow().clone();
    let [x0, y0, z0] = state.current_pos;
    let [x1, y1, z1] = target;

    let dx = x1 - x0;
    let dy = y1 - y0;
    let dz = z1 - z0;
    let total_distance = dist3d(x0, y0, z0, x1, y1, z1);
    if total_distance < EPS {
        return;
    }

    let steps_per_mm = [267.0, 267.0, 267.0];
    let dx_steps = (dx.abs() * steps_per_mm[0]) as u32;
    let dy_steps = (dy.abs() * steps_per_mm[1]) as u32;
    let dz_steps = (dz.abs() * steps_per_mm[2]) as u32;

    let major_steps = dx_steps.max(dy_steps).max(dz_steps).max(1);

    io.dirx
        .set_level(if dx >= 0.0 { Level::High } else { Level::Low });
    io.diry
        .set_level(if dy >= 0.0 { Level::High } else { Level::Low });
    io.dirz
        .set_level(if dz >= 0.0 { Level::High } else { Level::Low });

    // S-curve state
    let mut current_velocity = 0.0;
    let mut current_acceleration = 0.0;
    let mut deceleration_phase = false;
    let mut traveled = 0.0;

    let max_accel = 200_000.0; // mm/s²
    let max_jerk = max_accel / 0.05; // mm/s³

    for _ in 0..major_steps {
        // Update velocity via jerk‑limited S‑curve
        let remaining = total_distance - traveled;
        if remaining < EPS {
            MOTION_SIGNAL.signal(CURRENT_STATE.lock().await.borrow().clone());
            break;
        }

        // then each slice:
        s_curve_velocity(
            remaining,
            feed_rate,
            &mut current_velocity,
            &mut current_acceleration,
            &mut deceleration_phase,
            max_accel,
            max_jerk,
        );

        let delta_dist = current_velocity * DT; // mm
        let frac = delta_dist / total_distance;

        let raw_x = dx * frac * steps_per_mm[0];
        let raw_y = dy * frac * steps_per_mm[1];
        let raw_z = dz * frac * steps_per_mm[2];

        let sx = raw_x.abs() as u32;
        let sy = raw_y.abs() as u32;
        let sz = raw_z.abs() as u32;

        // 5) for any axis that needs steps, push “N steps @ delay”:
        if sx > 0 {
            // delay between each of those sx pulses so they fill DT
            let delay_x = (DT * (PIO_CLOCK_HZ as f32) / sx as f32) as u32;
            io.smx.tx().wait_push(sx).await;
            io.smx.tx().wait_push(delay_x).await;
        }
        if sy > 0 {
            let delay_y = (DT * (PIO_CLOCK_HZ as f32) / sy as f32) as u32;
            io.smy.tx().wait_push(sy).await;
            io.smy.tx().wait_push(delay_y).await;
        }
        if sz > 0 {
            let delay_z = (DT * (PIO_CLOCK_HZ as f32) / sz as f32) as u32;
            io.smz.tx().wait_push(sz).await;
            io.smz.tx().wait_push(delay_z).await;
        }

        traveled += delta_dist;

        // Update shared state
        {
            let st = CURRENT_STATE.lock().await;
            let mut s = st.borrow_mut();
            s.traveled_distance = traveled;
            s.current_velocity = current_velocity;
            s.distance_remaining = remaining - delta_dist;

            s.current_pos[0] += (raw_x / steps_per_mm[0]) as f32;
            s.current_pos[1] += (raw_y / steps_per_mm[1]) as f32;
            s.current_pos[2] += (raw_z / steps_per_mm[2]) as f32;

            s.current_acceleration = current_acceleration;
            s.deceleration_phase = deceleration_phase;
        }
        MOTION_SIGNAL.signal(CURRENT_STATE.lock().await.borrow().clone());
    }
}

fn s_curve_velocity(
    dist_rem: f32,
    v_target: f32,
    v_curr: &mut f32,
    a_curr: &mut f32,
    decel: &mut bool,
    a_max: f32,
    j_max: f32,
) {
    let accel_dist = (*v_curr * *v_curr) / (2.0 * a_max);
    let jerk_dist = (4.0 / 3.0) * powf(*v_curr, 1.5) / sqrtf(j_max);
    if accel_dist + jerk_dist >= dist_rem && !*decel {
        *decel = true;
        *a_curr = 0.0;
    }

    // 2) desired accel
    let desired = if *decel {
        if dist_rem > EPS {
            -(*v_curr * *v_curr) / (2.0 * dist_rem)
        } else {
            -a_max
        }
    } else {
        a_max
    }
    .clamp(-a_max, a_max);

    // 3) clamp accel change by jerk
    let delta_a = (desired - *a_curr).clamp(-j_max * DT, j_max * DT);
    *a_curr += delta_a;

    // 4) update velocity
    *v_curr = (*v_curr + *a_curr * DT).clamp(0.0, v_target);
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

        let clock_div = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles

        let stepper_prog = common.load_program(&stepper_prog.program);

        let x_step = common.make_pio_pin(resources.x_step);
        let y_step = common.make_pio_pin(resources.y_step);
        let z_step = common.make_pio_pin(resources.z_step);

        let mut cfgx = embassy_rp::pio::Config::default();
        cfgx.use_program(&stepper_prog, &[]);
        cfgx.clock_divider = clock_div;
        cfgx.set_set_pins(&[&x_step]);
        smx.set_config(&cfgx);

        let mut cfgy = embassy_rp::pio::Config::default();
        cfgy.use_program(&stepper_prog, &[]);
        cfgy.clock_divider = clock_div;
        cfgy.set_set_pins(&[&y_step]);
        smy.set_config(&cfgy);

        let mut cfgz = embassy_rp::pio::Config::default();
        cfgz.use_program(&stepper_prog, &[]);
        cfgz.clock_divider = clock_div;
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

#[derive(Clone, Debug, PartialEq)]
pub enum MotionCommand {
    MoveAbsolute([f32; 3], f32),
    MoveRelative([f32; 3], f32),
    Jog([f32; 3], f32),
    Stop,
    Zero,
}

fn dist3d(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> f32 {
    hypotf(hypotf(x1 - x0, y1 - y0), z1 - z0)
}
