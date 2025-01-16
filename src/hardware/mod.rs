mod servo;
mod stepper;

pub use servo::ServoDrive;
pub use stepper::StepperDrive;

use crate::Fixed32;
use embassy_time::Duration;

pub trait Motor {}

pub trait PositionalMotor {
    async fn move_over_time(&mut self, position: Fixed32, duration: Duration);
    fn get_position(&self) -> Fixed32;
}

pub enum Direction {
    Forward,
    Backward,
}
