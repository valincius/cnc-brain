#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::str;

use embassy_sync::channel::Channel;
use static_cell::StaticCell;

use assign_resources::assign_resources;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_rp::interrupt::{InterruptExt as _, Priority};
use embassy_rp::pio::Config;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    interrupt,
    peripherals::{PIO0 as pPIO0, USB as pUSB},
    pio::{Pio, StateMachine},
    rom_data::reset_to_usb_boot,
};
use embassy_rp::{peripherals, Peripheral as _};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb_logger::ReceiverHandler;
use fixed::traits::ToFixed;
use libm::{fabsf, floorf, hypotf};

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HI: InterruptExecutor = InterruptExecutor::new();

static CONTROLLER_CHANNEL: Channel<CriticalSectionRawMutex, ControllerCommand, 16> = Channel::new();

static MOTION_QUEUE: Channel<CriticalSectionRawMutex, MotionCommand, 128> = Channel::new();

static STEP_CHANNEL: Channel<CriticalSectionRawMutex, u32, 1024> = Channel::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<pPIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<pUSB>;
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

        step_dma0: DMA_CH0,
    }
}

#[interrupt]
unsafe fn SWI_IRQ_0() {
    EXECUTOR_HI.on_interrupt()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(move |spawner| spawner.spawn(core1_main(r.c1, spawner)).unwrap());
    //     },
    // );

    interrupt::SWI_IRQ_0.set_priority(Priority::P3);
    let spawner = EXECUTOR_HI.start(interrupt::SWI_IRQ_0);
    spawner.must_spawn(core1_main(r.c1));

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(move |spawner| spawner.spawn(core0_main(r.c0, spawner)).unwrap());
}

#[embassy_executor::task]
async fn core0_main(r: Core0Resources, spawner: Spawner) {
    let usb_driver = embassy_rp::usb::Driver::new(r.usb, Irqs);
    spawner.spawn(usb_comm_task(usb_driver)).unwrap();

    log::info!("Core 0 started");

    let mut led = Output::new(r.led, Level::Low);

    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        led.toggle();

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn core1_main(r: Core1Resources) {
    Timer::after_secs(5).await;

    log::info!("Core 1 started");

    let spawner = Spawner::for_current_executor().await;

    log::info!("Got spawner");

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

    log::info!("Loaded stepper program");

    let stepper_prog = common.load_program(&stepper_prog.program);

    log::info!("Loaded stepper program");

    let x_step = common.make_pio_pin(r.x_step);
    let x_dir = Output::new(r.x_dir, Level::Low);

    let mut step_pio_cfg = Config::default();
    step_pio_cfg.set_set_pins(&[&x_step]);
    step_pio_cfg.use_program(&stepper_prog, &[]);
    step_pio_cfg.clock_divider = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles

    sm0.set_config(&step_pio_cfg);
    sm0.set_enable(true);

    log::info!("Enabled stepper program");

    if spawner.spawn(controller()).is_ok() {
        log::info!("Spawned controller");
    } else {
        log::info!("Failed to spawn controller");
    }

    if spawner.spawn(step_consumer(sm0, r.step_dma0)).is_ok() {
        log::info!("Spawned step consumer");
    } else {
        log::info!("Failed to spawn step consumer");
    }

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
                            .send(ControllerCommand::MoveTo([250.0, 250.0, 250.0], 7500.0))
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
async fn usb_comm_task(driver: embassy_rp::usb::Driver<'static, pUSB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Trace, driver, Handler);
}

enum ControllerCommand {
    MoveTo([f32; 3], f32),
    Stop,
}

#[embassy_executor::task]
async fn step_consumer(mut sm0: StateMachine<'static, pPIO0, 0>, dma0: peripherals::DMA_CH0) {
    log::info!("Step consumer started");

    let mut dma0_ref = dma0.into_ref();

    log::info!("Got DMA ref");

    let mut buffer = [0u32; 1024];
    loop {
        log::info!("Waiting for buffer");

        for i in 0..buffer.len() {
            buffer[i] = STEP_CHANNEL.receive().await;
        }

        sm0.tx().dma_push(dma0_ref.reborrow(), &buffer).await;

        log::info!("Got buffer");
    }
}

async fn execute_motion(machine: &mut SystemState, command: MotionCommand) {
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

        if steps_x_abs > 0.0 {
            STEP_CHANNEL.send(steps_x_abs as u32).await;
            STEP_CHANNEL.send(delay_cycles).await;

            log::info!("Pushed steps");
        }
    }
}

#[embassy_executor::task]
async fn controller() {
    log::info!("Controller started");

    let mut machine = SystemState {
        current_pos: [0.0; 3],
        current_speed: 0.0,
    };

    loop {
        log::info!("Controller running");
        if let Ok(block) = MOTION_QUEUE.try_receive() {
            log::info!("Got block");

            execute_motion(&mut machine, block).await;

            log::info!("Done with block");
        }

        match CONTROLLER_CHANNEL.receive().await {
            ControllerCommand::MoveTo(target, max_speed) => {
                log::info!("!!Moving to!!");
                MOTION_QUEUE
                    .send(MotionCommand::Linear(target, max_speed))
                    .await;
                log::info!("!!Done moving!!");
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

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reset_to_usb_boot(0, 0); // Restart the chip

    loop {}
}
