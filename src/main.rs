#![no_std]
#![no_main]

use core::str;

use assign_resources::assign_resources;
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{PIO0, USB},
    pio::{Pio, StateMachine},
    rom_data::reset_to_usb_boot,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embassy_usb_logger::ReceiverHandler;
use panic_probe as _;
use static_cell::StaticCell;

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

static CONTROLLER_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    ControllerCommand,
    16,
> = Channel::new();

static MOTION_QUEUE: embassy_sync::channel::Channel<CriticalSectionRawMutex, MotionCommand, 128> =
    Channel::new();

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
        mut common, sm0, ..
    } = Pio::new(r.pio, Irqs);

    spawner.spawn(controller(sm0)).unwrap();

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
                    "go" => {}
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
    embassy_usb_logger::run!(1024, log::LevelFilter::Trace, driver, Handler);
}

enum ControllerCommand {
    MoveTo([f32; 3], f32),
    Stop,
}

#[embassy_executor::task]
async fn controller(sm: StateMachine<'static, PIO0, 0>) {
    let mut machine = SystemState {
        current_step: [0; 3],
        current_speed: 0.0,
    };

    let accel = 100.0f32; // mm/s^2

    let steps_per_mm = [267, 267, 267];

    loop {
        if let Ok(block) = MOTION_QUEUE.try_receive() {
            let [x0, y0, z0] = machine.current_step;

            let (x1, y1, z1, feedRate) = match block {
                MotionCommand::Linear([x, y, z], feedRate) => (
                    (x * steps_per_mm[0] as f32) as u32,
                    (y * steps_per_mm[1] as f32) as u32,
                    (z * steps_per_mm[2] as f32) as u32,
                    feedRate,
                ),
            };

            // We have a block with: x0, y0, x1, y1, feedRate, etc.
            // Initialize:
            let distance = hypot(x1 - x1, y1 - y1);
            let traveled = 0.0;
            let current_vel = 0.0; // or carry over from prior block
            let max_vel = feedRate; // in, say, mm/s
            let dt = 0.001; // 1 ms

            while traveled < distance {
                // 1) Compute how far left
                let dist_left = distance - traveled;

                // 2) Decide if we should accelerate or decelerate:
                //    decel_dist = v^2 / (2*a)
                let decel_dist = (current_vel * current_vel) / (2.0 * accel);

                if decel_dist >= dist_left {
                    // Time to decelerate
                    current_vel -= accel * dt;
                    if current_vel < 0.0 {
                        current_vel = 0.0;
                    }
                } else {
                    // Accelerate if we're below max velocity
                    if current_vel < max_vel {
                        current_vel += accel * dt;
                        if current_vel > max_vel {
                            current_vel = max_vel;
                        }
                    }
                }

                // 3) Distance covered in this micro-block
                let delta_dist = current_vel * dt;
                traveled += delta_dist;

                // 4) Convert that distance along the line to steps for X and Y
                //    direction cosines
                let dx = x1 - x1;
                let dy = y1 - y1;
                let line_len = distance;
                let frac = delta_dist / line_len;

                // The actual change in X, Y in mm
                let step_x_mm = dx * frac; // e.g. 2.5 mm in X
                let step_y_mm = dy * frac; // e.g. 2.5 mm in Y

                // Convert mm to steps
                let steps_x = (step_x_mm * steps_per_mm[0]).round() as i32;
                let steps_y = (step_y_mm * steps_per_mm[1]).round() as i32;

                // Keep track of the "absolute" position
                x1 += dx * frac;
                y1 += dy * frac;

                // 5) Figure out step period or step rate for each axis
                //    We want the micro-block to finish in dt. So we have steps_x pulses in dt => rate_x = steps_x / dt
                //    But if we're letting PIO do one chunk at a constant rate, we might feed it:
                //
                //    axis_sm.tx().push( steps_x )    // total pulses
                //    axis_sm.tx().push( step_period )  // e.g. CPU cycles between pulses
                //
                // We'll do the same for Y axis. Or if we have a single SM per axis, we do them individually.

                // For direction pins, set them if steps_x < 0, etc. (and use abs for pulse count).
                let dir_x = if steps_x >= 0 { 1 } else { 0 };
                let dir_y = if steps_y >= 0 { 1 } else { 0 };
                let steps_x_abs = steps_x.abs();
                let steps_y_abs = steps_y.abs();

                // Step period:
                // If rate_x = steps_x_abs / dt, then step_period_x = CPU_freq / rate_x
                // or however your PIO code interprets it.
                // Example:
                let rate_x = (steps_x_abs as f32) / dt; // steps/s
                let step_period_x = (125_000_000 as f32 / rate_x) as u32;

                // Then push data to PIO
                // set_gpio_pin(X_DIR, dir_x);
                // x_sm.tx().push(steps_x_abs);
                // x_sm.tx().push(step_period_x);

                // // Similarly for Y
                // set_gpio_pin(Y_DIR, dir_y);
                // y_sm.tx().push(steps_y_abs);
                // y_sm.tx().push(step_period_x_for_y);

                // 6) Wait for the next slice or wait for PIO to finish
                //    This could be "Timer::after(dt)" or an interrupt, etc.
            }
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
    current_step: [u32; 3], // Current position of each axis
    current_speed: f32,
}

#[derive(Debug, Clone, Copy)]
pub enum MotionCommand {
    Linear([f32; 3], f32),
}
