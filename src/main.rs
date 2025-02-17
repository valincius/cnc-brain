#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use cnc_brain::motion::{motion_task, MotionCommand, MOTION_QUEUE, MOTION_STATE};
use cnc_brain::receiver::usb_comm_task;
use cnc_brain::{
    split_resources, AssignedResources, ControllerCommand, ControllerResources, Irqs,
    StepperResources, CONTROLLER_CHANNEL,
};

use cortex_m_rt::entry;
use static_cell::StaticCell;

use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_rp::{
    gpio::{Level, Output},
    interrupt,
    interrupt::{InterruptExt as _, Priority},
    multicore::{spawn_core1, Stack},
    rom_data::reset_to_usb_boot,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HI: InterruptExecutor = InterruptExecutor::new();

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
    spawner.must_spawn(motion_task(r.for_motion));

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.must_spawn(main_task()));
}

#[embassy_executor::task]
async fn main_task() {
    loop {
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

#[embassy_executor::task]
async fn controller_task(r: ControllerResources) {
    let spawner = Spawner::for_current_executor().await;

    let usb_driver = embassy_rp::usb::Driver::new(r.usb, Irqs);
    spawner.spawn(usb_comm_task(usb_driver)).unwrap();

    let mut led = Output::new(r.led, Level::Low);

    // let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        // led.toggle();

        let state = MOTION_STATE.wait().await;

        log::info!("Motion state: {:?}", state);

        // ticker.next().await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reset_to_usb_boot(0, 0); // Restart the chip

    loop {}
}
