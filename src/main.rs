#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cnc_brain::motion::{self, MOTION_QUEUE, MOTION_SIGNAL, MotionCommand, STOP_SIGNAL};
use cnc_brain::receiver::usb_comm_task;
use cnc_brain::{
    AssignedResources, CONTROLLER_CHANNEL, ControllerCommand, ControllerResources, InputResources,
    Irqs, StepperResources, jog_wheel, split_resources,
};

use cortex_m_rt::entry;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_rp::{
    interrupt,
    interrupt::{InterruptExt as _, Priority},
    multicore::{Stack, spawn_core1},
    rom_data::reset_to_usb_boot,
};
use embassy_time::Timer;
use static_cell::StaticCell;

static mut CORE1_STACK: Stack<4096> = Stack::new();

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HI: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
fn SWI_IRQ_0() {
    unsafe { EXECUTOR_HI.on_interrupt() }
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
    spawner.must_spawn(motion::motion_task(r.for_motion));

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(jog_wheel::task(r.for_inputs));
        spawner.must_spawn(main_task());
    });
}

#[embassy_executor::task]
async fn main_task() {
    loop {
        match CONTROLLER_CHANNEL.receive().await {
            ControllerCommand::MoveTo(target, max_speed) => {
                MOTION_QUEUE
                    .send(MotionCommand::MoveAbsolute(target, max_speed))
                    .await;
            }

            ControllerCommand::Stop => {
                STOP_SIGNAL.signal(());
            }

            ControllerCommand::Zero => {
                MOTION_QUEUE.send(MotionCommand::Zero).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn controller_task(r: ControllerResources) {
    let spawner = Spawner::for_current_executor().await;

    let usb_driver = embassy_rp::usb::Driver::new(r.usb, Irqs);
    spawner.spawn(usb_comm_task(usb_driver)).unwrap();

    loop {
        if let Some(state) = MOTION_SIGNAL.try_take() {
            log::info!("motion_state={:?}", state);
        }

        Timer::after_millis(50).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reset_to_usb_boot(0, 0); // Restart the chip

    loop {}
}
