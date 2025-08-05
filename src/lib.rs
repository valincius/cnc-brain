#![no_std]
#![feature(impl_trait_in_assoc_type)]

pub mod jog_wheel;
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
    MoveTo([i32; 3], f32),
    Stop,
    Zero,
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
        x_step: PIN_13,
        y_step: PIN_15,
        z_step: PIN_14,
        x_dir: PIN_10,
        y_dir: PIN_12,
        z_dir: PIN_11,
    }

    for_spindle: SpindleResources {
        pwm: PWM_SLICE0,
        speed: PIN_16,
    }

    for_inputs: InputResources {
        jog_a: PIN_0,
        jog_b: PIN_1,
        jog_pwr: PIN_2,
        jog_a2: PIN_3,
        jog_b2: PIN_4,
    }
}
