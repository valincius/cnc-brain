#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cnc_brain::receiver::usb_comm_task;
use cnc_brain::{ControllerCommand, CONTROLLER_CHANNEL};

use cortex_m_rt::{entry, exception};
use embassy_rp::dma::Channel as _;
use embassy_rp::multicore::{spawn_core1, Stack};
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
    pio::Pio,
    rom_data::reset_to_usb_boot,
};
use embassy_rp::{peripherals, Peripheral as _};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Ticker, Timer};
use fixed::traits::ToFixed;
use libm::{fabsf, floorf, hypotf};

static mut CORE1_STACK: Stack<4096> = Stack::new();

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HI: InterruptExecutor = InterruptExecutor::new();

static MOTION_QUEUE: Channel<CriticalSectionRawMutex, MotionCommand, 128> = Channel::new();
static STEP_CHANNEL: Channel<CriticalSectionRawMutex, u32, 1024> = Channel::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<pPIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<pUSB>;
});

assign_resources! {
    for_controller: ControllerResources {
        usb: USB,
        led: PIN_25,
    }

    for_motion: MotionResources {
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

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.must_spawn(controller_task(r.for_controller)));
        },
    );

    interrupt::SWI_IRQ_0.set_priority(Priority::P3);
    let spawner = EXECUTOR_HI.start(interrupt::SWI_IRQ_0);
    spawner.must_spawn(stepper_task(r.for_motion));

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.must_spawn(main_task()));
}

#[embassy_executor::task]
async fn main_task() {
    let spawner = Spawner::for_current_executor().await;

    spawner.must_spawn(motion_task());

    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        log::info!("Core 0 running");

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn controller_task(r: ControllerResources) {
    let spawner = Spawner::for_current_executor().await;

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
async fn stepper_task(r: MotionResources) {
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

    let x_step = common.make_pio_pin(r.x_step);
    let _x_dir = Output::new(r.x_dir, Level::Low);

    let mut step_pio_cfg = Config::default();
    step_pio_cfg.set_set_pins(&[&x_step]);
    step_pio_cfg.use_program(&stepper_prog, &[]);
    step_pio_cfg.clock_divider = (104.17).to_fixed(); // 125MHz / 104.17 = 1.2MHz = we are aiming for 200khz max pulse rate (5us per pulse), each pulse is 6 cycles

    sm0.set_config(&step_pio_cfg);
    sm0.set_enable(true);

    let mut dma0_ref = r.step_dma0.into_ref();

    let mut buffer = [0u32; 1024];
    loop {
        for i in 0..buffer.len() {
            buffer[i] = STEP_CHANNEL.receive().await;
        }

        sm0.tx().dma_push(dma0_ref.reborrow(), &buffer).await;
    }
}

#[embassy_executor::task]
async fn motion_task() {
    let mut machine = SystemState {
        current_pos: [0.0; 3],
        _current_speed: 0.0,
    };

    loop {
        if let Ok(block) = MOTION_QUEUE.try_receive() {
            execute_motion(&mut machine, block).await;
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

        if steps_x_abs > 0.0 {
            STEP_CHANNEL.send(steps_x_abs as u32).await;
            STEP_CHANNEL.send(delay_cycles).await;
        }
    }

    // fill rest of the buffer
    while STEP_CHANNEL.free_capacity() > 0 {
        STEP_CHANNEL.send(0).await;
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

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reset_to_usb_boot(0, 0); // Restart the chip

    loop {}
}
