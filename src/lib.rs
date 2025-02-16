#![no_std]
#![feature(impl_trait_in_assoc_type)]

pub mod receiver;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

pub static CONTROLLER_CHANNEL: Channel<CriticalSectionRawMutex, ControllerCommand, 16> =
    Channel::new();

pub enum ControllerCommand {
    MoveTo([f32; 3], f32),
    Stop,
}
