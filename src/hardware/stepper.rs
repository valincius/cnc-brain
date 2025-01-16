use embassy_time::{Duration, Timer};

use super::{Fixed32, PositionalMotor};

pub struct StepperDrive {
    pub pin: u8,
    pub steps_per_mm: u32,
    pub step_count: u32,
    pub step_delay: u64,
}

impl StepperDrive {
    pub fn new(pin: u8) -> Self {
        StepperDrive {
            pin,
            steps_per_mm: 200,
            step_count: 0,
            step_delay: 0,
        }
    }
}

impl PositionalMotor for StepperDrive {
    async fn move_over_time(&mut self, destination: Fixed32, duration: Duration) {
        // we'll eventually use dma and push to a buffer, but for now we'll just use some async delay

        // build up a step buffer, take a frequency, and step at that frequency, 1 = step, 0 = delay 1 cycle
        let steps = (destination - self.get_position()).abs().to_num::<u32>() * self.steps_per_mm;

        let step_delay = duration.as_millis() / steps as u64;

        // set the new step delay
        for _ in 0..steps {
            self.step_count += 1;
            // self.step_delay = lerp(self.step_delay, step_delay, 0.1);
            Timer::after(Duration::from_millis(self.step_delay)).await;
        }
    }

    fn get_position(&self) -> Fixed32 {
        Fixed32::from_num(self.step_count as f32 / self.steps_per_mm as f32)
    }
}
