mod stepper;

use core::future::Future;

pub use stepper::StepperDrive;

use crate::Fixed32;
use embassy_time::Duration;

pub trait Motor {}

pub trait PositionalMotor {
    fn move_over_time(
        &mut self,
        position: Fixed32,
        duration: Duration,
    ) -> impl Future<Output = ()> + Send;
    fn get_position(&self) -> Fixed32;
}

pub enum Direction {
    Forward,
    Backward,
}
