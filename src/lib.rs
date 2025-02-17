#![no_std]
#![feature(impl_trait_in_assoc_type)]

pub mod motion;
pub mod receiver;

use assign_resources::assign_resources;
use embassy_rp::{
    bind_interrupts,
    peripherals::{self, PIO0, USB},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

pub static CONTROLLER_CHANNEL: Channel<CriticalSectionRawMutex, ControllerCommand, 16> =
    Channel::new();

pub enum ControllerCommand {
    MoveTo([f32; 3], f32),
    Stop,
}

bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

assign_resources! {
    for_controller: ControllerResources {
        usb: USB,
        led: PIN_25,
    }

    for_motion: StepperResources {
        pio: PIO0,
        x_step: PIN_0,
        x_dir: PIN_1,

        step_dma0: DMA_CH0,
    }
}
