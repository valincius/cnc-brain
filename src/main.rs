#![no_std]
#![no_main]

use core::str;

use assign_resources::assign_resources;
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::pio::Config;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{PIO0, USB},
    pio::{Pio, StateMachine},
    rom_data::reset_to_usb_boot,
};
use embassy_rp::{peripherals, Peripheral};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb_logger::ReceiverHandler;
use fixed::traits::ToFixed;
use libm::hypotf;
use panic_probe as _;
use static_cell::StaticCell;

static mut CORE1_STACK: Stack<{ 2 << 14 }> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

static CONTROLLER_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    ControllerCommand,
    16,
> = Channel::new();

static MOTION_QUEUE: embassy_sync::channel::Channel<CriticalSectionRawMutex, MotionCommand, 128> =
    Channel::new();

const CPU_CLOCK_HZ: u32 = 125_000_000;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

assign_resources! {
    c0: Core0Resources {
        usb: USB,
        led: PIN_25,
    }

    c1: Core1Resources {
        pio: PIO0,
        x_step: PIN_0,
        x_dir: PIN_1,
        x_step_buf: DMA_CH0,
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(move |spawner| spawner.spawn(core1_main(r.c1, spawner)).unwrap());
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(move |spawner| spawner.spawn(core0_main(r.c0, spawner)).unwrap());
}

#[embassy_executor::task]
async fn core0_main(r: Core0Resources, spawner: Spawner) {
    let usb_driver = embassy_rp::usb::Driver::new(r.usb, Irqs);
    spawner.spawn(usb_comm_task(usb_driver)).unwrap();

    let mut led = Output::new(r.led, Level::Low);

    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        led.toggle();

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn core1_main(r: Core1Resources, spawner: Spawner) {
    let Pio {
        mut common,
        sm0: mut x_sm,
        sm1,
        sm2,
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

    let x_step = common.make_pio_pin(r.x_step);
    let x_dir = Output::new(r.x_dir, Level::Low);

    let mut x_cfg = Config::default();
    x_cfg.set_set_pins(&[&x_step]);
    x_cfg.use_program(&stepper_prog, &[]);
    x_cfg.clock_divider = (CPU_CLOCK_HZ / 200_000).to_fixed();

    x_sm.set_config(&x_cfg);
    x_sm.set_enable(true);

    spawner.spawn(controller(x_sm, r.x_step_buf)).unwrap();

    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        log::info!("Core 1 running");

        ticker.next().await;
    }
}

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = str::from_utf8(data) {
            let data = data.trim();

            let mut parts = data.split_whitespace();
            if let Some(cmd) = parts.next() {
                match cmd {
                    "go" => {
                        CONTROLLER_CHANNEL
                            .send(ControllerCommand::MoveTo([250.0, 250.0, 250.0], 1000.0))
                            .await;
                    }
                    "stop" => {
                        CONTROLLER_CHANNEL.send(ControllerCommand::Stop).await;
                    }
                    "q" => {
                        reset_to_usb_boot(0, 0); // Restart the chip
                    }
                    _ => {
                        log::info!("Unknown command: {}", cmd);
                    }
                }
            }
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
async fn usb_comm_task(driver: embassy_rp::usb::Driver<'static, USB>) {
    embassy_usb_logger::run!({ 2 << 12 }, log::LevelFilter::Trace, driver, Handler);
}

enum ControllerCommand {
    MoveTo([f32; 3], f32),
    Stop,
}

#[embassy_executor::task]
async fn controller(
    mut x_sm: StateMachine<'static, PIO0, 0>,
    mut x_step_buf: peripherals::DMA_CH0,
) {
    let mut machine = SystemState {
        current_pos: [0.0; 3],
        current_speed: 0.0,
    };

    let accel = 500.0f32;

    let micro_steps = 16;

    let steps_per_mm = [267.0f32, 267.0f32, 267.0f32];
    let steps_per_mm = [
        steps_per_mm[0] * micro_steps as f32,
        steps_per_mm[1] * micro_steps as f32,
        steps_per_mm[2] * micro_steps as f32,
    ];

    let mut x_step_buf_ref = x_step_buf.into_ref();
    let mut x_buffer = [0u32; 256];
    let mut iter = 0;

    loop {
        if let Ok(block) = MOTION_QUEUE.try_receive() {
            let [mut x0, mut y0, mut z0] = machine.current_pos;

            let (x1, y1, z1, feed_rate) = match block {
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

            while traveled < distance - epsilon {
                if iter >= x_buffer.len() {
                    x_sm.tx()
                        .dma_push(x_step_buf_ref.reborrow(), &x_buffer)
                        .await;
                    iter = 0;
                } else {
                    Timer::after(Duration::from_millis(1)).await;
                }

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

                let steps_x = (step_x_mm * steps_per_mm[0]) as i32;
                let steps_y = (step_y_mm * steps_per_mm[1]) as i32;
                let steps_z = (step_z_mm * steps_per_mm[2]) as i32;

                x0 += step_x_mm;
                y0 += step_y_mm;
                z0 += step_z_mm;

                // let dir_x = if steps_x >= 0 { 1 } else { 0 };
                // let dir_y = if steps_y >= 0 { 1 } else { 0 };
                // let dir_z = if steps_z >= 0 { 1 } else { 0 };

                let steps_x_abs = steps_x.abs() as u32;
                let steps_y_abs = steps_y.abs() as u32;
                let steps_z_abs = steps_z.abs() as u32;

                let rate_x = (steps_x_abs as f32) / dt; // steps/s
                let rate_y = (steps_y_abs as f32) / dt; // steps/s
                let rate_z = (steps_z_abs as f32) / dt; // steps/s

                let step_period_x = (200_000 as f32 / rate_x) as u32;
                let step_period_y = (200_000 as f32 / rate_y) as u32;
                let step_period_z = (200_000 as f32 / rate_z) as u32;

                log::info!(
                    "pos: [{}, {}, {}], steps: [{}, {}, {}], rates: [{}, {}, {}]",
                    x0,
                    y0,
                    z0,
                    steps_x,
                    steps_y,
                    steps_z,
                    step_period_x,
                    step_period_y,
                    step_period_z
                );

                if steps_x_abs > 0 {
                    x_buffer[iter] = steps_x_abs;
                    x_buffer[iter + 1] = step_period_x;

                    iter += 2;
                }

                // y_sm.tx().wait_push(rate_y as u32).await;
                // y_sm.tx().wait_push(step_period_y).await;

                // z_sm.tx().wait_push(rate_z as u32).await;
                // z_sm.tx().wait_push(step_period_z).await;
            }

            log::info!("Done moving");
        } else {
            log::info!("No motion command");
        }

        match CONTROLLER_CHANNEL.receive().await {
            ControllerCommand::MoveTo(target, max_speed) => {
                MOTION_QUEUE
                    .send(MotionCommand::Linear(target, max_speed))
                    .await;
            }

            ControllerCommand::Stop => {
                log::info!("!!Stopping!!");
            }
        }
    }
}

struct SystemState {
    current_pos: [f32; 3], // Current position of each axis
    current_speed: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum MotionCommand {
    Linear([f32; 3], f32),
}

fn dist3d(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32) -> f32 {
    hypotf(hypotf(x1 - x0, y1 - y0), z1 - z0)
}
