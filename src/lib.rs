#![no_std]

pub mod hardware;
pub mod prelude;
pub mod systems;

pub type Fixed32 = fixed::types::I16F16;

#[derive(Default, Copy, Clone)]
pub struct Coordinates {
    pub x: Option<Fixed32>,
    pub y: Option<Fixed32>,
    pub z: Option<Fixed32>,
}

#[derive(Copy, Clone)]
pub enum Motion {
    Rapid(Coordinates),
    Linear {
        feedrate: Fixed32,
        coords: Coordinates,
    },
}

pub enum SystemCommand {
    MoveTo(Motion),
}

const CHANNEL_BUFFER_CAPACITY: usize = 64;

pub type CommandChannel = embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    SystemCommand,
    CHANNEL_BUFFER_CAPACITY,
>;

pub type CommandReceiver = embassy_sync::channel::Receiver<
    'static,
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    SystemCommand,
    CHANNEL_BUFFER_CAPACITY,
>;

pub type CommandSender = embassy_sync::channel::Sender<
    'static,
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    SystemCommand,
    CHANNEL_BUFFER_CAPACITY,
>;
