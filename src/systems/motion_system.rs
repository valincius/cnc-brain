use embassy_futures::join;
use embassy_time::{Duration, Timer};
use heapless::Vec;

use crate::prelude::*;

const MAX_BUFFER_SIZE: usize = 100;

pub struct MotionSystem<X: PositionalMotor, Y: PositionalMotor, Z: PositionalMotor> {
    x_motor: X,
    y_motor: Y,
    z_motor: Z,

    buffer: Vec<MotionSegment, MAX_BUFFER_SIZE>,
}

#[derive(Clone, Copy)]
struct MotionSegment {
    target_position: [Fixed32; 3],
    duration_ms: u64,
}

impl<X: PositionalMotor, Y: PositionalMotor, Z: PositionalMotor> MotionSystem<X, Y, Z> {
    pub fn new(x_motor: X, y_motor: Y, z_motor: Z) -> MotionSystem<X, Y, Z> {
        MotionSystem {
            x_motor,
            y_motor,
            z_motor,
            buffer: Vec::new(),
        }
    }

    pub fn get_position(&self) -> [Fixed32; 3] {
        [
            self.x_motor.get_position(),
            self.y_motor.get_position(),
            self.z_motor.get_position(),
        ]
    }

    pub async fn tick(&mut self) {
        if let Some(segment) = self.dequeue().await {
            self.move_to(segment).await;
        }
    }

    async fn move_to(&mut self, segment: MotionSegment) {
        let duration = Duration::from_millis(segment.duration_ms);

        let x_move = self
            .x_motor
            .move_over_time(segment.target_position[0], duration);
        let y_move = self
            .y_motor
            .move_over_time(segment.target_position[1], duration);
        let z_move = self
            .z_motor
            .move_over_time(segment.target_position[2], duration);

        join::join3(x_move, y_move, z_move).await;
    }

    pub async fn queue(&mut self, motion: Motion) {
        let new_segment = self.create_segment(motion);

        while self.buffer.push(new_segment).is_err() {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    async fn dequeue(&mut self) -> Option<MotionSegment> {
        if self.buffer.is_empty() {
            None
        } else {
            Some(self.buffer.remove(0))
        }
    }

    fn create_segment(&self, motion: Motion) -> MotionSegment {
        let prev_segment = self.buffer.last();

        match motion {
            Motion::Linear { feedrate, coords } => {
                let starting_position = if let Some(prev) = prev_segment {
                    prev.target_position
                } else {
                    self.get_position()
                };

                let target_position = [
                    coords.x.unwrap_or(starting_position[0]),
                    coords.y.unwrap_or(starting_position[1]),
                    coords.z.unwrap_or(starting_position[2]),
                ];

                let dx = target_position[0] - starting_position[0];
                let dy = target_position[1] - starting_position[1];
                let dz = target_position[2] - starting_position[2];

                let distance = (dx * dx + dy * dy + dz * dz).sqrt();

                // we'll want to account for the acceleration and deceleration here
                let duration_ms = (distance / feedrate).to_num::<u64>();

                MotionSegment {
                    target_position,
                    duration_ms,
                }
            }
            Motion::Rapid(_coords) => {
                todo!()
            }
        }
    }
}
