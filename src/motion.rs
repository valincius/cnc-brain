use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    pio::Pio,
    Peripheral as _,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    channel::Channel,
    zerocopy_channel::{Channel as ZChannel, Receiver, Sender},
};
use fixed::traits::ToFixed as _;
use libm::{fabsf, floorf, hypotf};
use static_cell::StaticCell;

use crate::{Irqs, StepperResources};

type StepBuffer = [u32; 512];

pub static MOTION_QUEUE: Channel<CriticalSectionRawMutex, MotionCommand, 128> = Channel::new();

#[embassy_executor::task]
pub async fn motion_task(stepper_resources: StepperResources) {
    let spawner = Spawner::for_current_executor().await;

    const BLOCK_SIZE: usize = 512;
    const NUM_BLOCKS: usize = 2;

    static BUF: StaticCell<[StepBuffer; NUM_BLOCKS]> = StaticCell::new();
    let buf = BUF.init([[0; BLOCK_SIZE]; NUM_BLOCKS]);

    static CHANNEL: StaticCell<ZChannel<'_, NoopRawMutex, StepBuffer>> = StaticCell::new();
    let channel = CHANNEL.init(ZChannel::new(buf));
    let (mut sender, receiver) = channel.split();

    spawner.must_spawn(stepper_task(stepper_resources, receiver));

    let mut machine = SystemState {
        current_pos: [0.0; 3],
        _current_speed: 0.0,
    };

    loop {
        let block = MOTION_QUEUE.receive().await;
        execute_motion(&mut sender, &mut machine, block).await;
    }
}

#[embassy_executor::task]
async fn stepper_task(
    r: StepperResources,
    mut receiver: Receiver<'static, NoopRawMutex, StepBuffer>,
) {
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

    let mut dma_ref = r.step_dma0.into_ref();

    loop {
        let buffer = receiver.receive().await;

        sm0.tx().dma_push(dma_ref.reborrow(), buffer).await;

        receiver.receive_done();
    }
}

async fn execute_motion(
    sender: &mut Sender<'static, NoopRawMutex, StepBuffer>,
    machine: &mut SystemState,
    command: MotionCommand,
) {
    let accel = 100_000.0f32;

    let micro_steps = 16;

    let steps_per_mm = [267.0f32, 267.0f32, 267.0f32];
    let steps_per_mm = [
        steps_per_mm[0] * micro_steps as f32,
        steps_per_mm[1] * micro_steps as f32,
        steps_per_mm[2] * micro_steps as f32,
    ];

    let [mut x0, mut y0, mut z0] = machine.current_pos;

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
    let dt = 0.0001; // 1ms
    let mut is_decelerating = false;

    let epsilon = 0.001; // mm

    let mut delay_accum: f32 = 0.0;

    let mut step_index = 0;

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
        let _steps_y = step_y_mm * steps_per_mm[1];
        let _steps_z = step_z_mm * steps_per_mm[2];

        x0 += step_x_mm;
        y0 += step_y_mm;
        z0 += step_z_mm;

        // let dir_x = if steps_x >= 0 { 1 } else { 0 };
        // let dir_y = if steps_y >= 0 { 1 } else { 0 };
        // let dir_z = if steps_z >= 0 { 1 } else { 0 };

        let steps_x_abs = fabsf(steps_x);
        // let steps_y_abs = steps_y.abs() as u32;
        // let steps_z_abs = steps_z.abs() as u32;

        let rate_x = (steps_x_abs as f32) / dt; // steps/s
                                                // let rate_y = (steps_y_abs as f32) / dt; // steps/s
                                                // let rate_z = (steps_z_abs as f32) / dt; // steps/s

        let step_period_x = (1200 * 200_000) as f32 / rate_x;
        // let step_period_y = ((1200 * 200_000) as f32 / rate_y) as u32;
        // let step_period_z = ((1200 * 200_000) as f32 / rate_z) as u32;

        delay_accum += step_period_x;

        // Extract the integer part to use as the delay
        let delay_cycles = floorf(delay_accum) as u32;

        // Subtract the integer part, leaving the fractional remainder
        delay_accum -= delay_cycles as f32;

        let buf = sender.send().await;

        if steps_x_abs > 0.0 {
            buf[step_index] = delay_cycles;
            buf[step_index + 1] = steps_x_abs as u32;

            step_index += 2;
        }

        if step_index >= buf.len() {
            sender.send_done();
            step_index = 0;
        }
    }

    if step_index > 0 {
        sender.send_done();
    }
}

struct SystemState {
    current_pos: [f32; 3], // Current position of each axis
    _current_speed: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum MotionCommand {
    Linear([f32; 3], f32),
}

fn dist3d(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> f32 {
    hypotf(hypotf(x1 - x0, y1 - y0), z1 - z0)
}
