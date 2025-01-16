#![no_std]
#![no_main]

use cnc_brain::prelude::*;

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

static CHANNEL: CommandChannel = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let motion_system = MotionSystem::new(
        StepperDrive::new(0),
        StepperDrive::new(1),
        StepperDrive::new(2),
    );
    let mut machine = MillSystem::new(CHANNEL.receiver(), motion_system, ServoDrive::new(3, 4));

    unwrap!(spawner.spawn(sender_task())); // send some initial commands

    machine.run().await;
}

#[embassy_executor::task]
async fn sender_task() {
    let program = [
        Motion::Linear {
            feedrate: Fixed32::from_num(100.0),
            coords: Coordinates {
                x: Some(Fixed32::from_num(150.0)),
                y: Some(Fixed32::from_num(150.0)),
                z: Some(Fixed32::from_num(0.0)),
            },
        },
        Motion::Linear {
            feedrate: Fixed32::from_num(100.0),
            coords: Coordinates {
                x: Some(Fixed32::from_num(158.0)),
                y: Some(Fixed32::from_num(253.0)),
                z: Some(Fixed32::from_num(0.0)),
            },
        },
        Motion::Linear {
            feedrate: Fixed32::from_num(100.0),
            coords: Coordinates {
                x: Some(Fixed32::from_num(200.0)),
                y: Some(Fixed32::from_num(253.0)),
                z: Some(Fixed32::from_num(0.0)),
            },
        },
        Motion::Linear {
            feedrate: Fixed32::from_num(100.0),
            coords: Coordinates {
                x: Some(Fixed32::from_num(200.0)),
                y: Some(Fixed32::from_num(260.0)),
                z: Some(Fixed32::from_num(0.0)),
            },
        },
        Motion::Linear {
            feedrate: Fixed32::from_num(100.0),
            coords: Coordinates {
                x: Some(Fixed32::from_num(200.0)),
                y: Some(Fixed32::from_num(275.0)),
                z: Some(Fixed32::from_num(0.0)),
            },
        },
        Motion::Linear {
            feedrate: Fixed32::from_num(100.0),
            coords: Coordinates {
                x: Some(Fixed32::from_num(220.0)),
                y: Some(Fixed32::from_num(280.0)),
                z: Some(Fixed32::from_num(0.0)),
            },
        },
    ];

    for motion in program.iter() {
        CHANNEL.sender().send(SystemCommand::MoveTo(*motion)).await;
        Timer::after(Duration::from_millis(100)).await;
    }
}
