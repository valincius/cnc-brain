use std::{sync::LazyLock, time::Duration};

use tokio::time::sleep;

use crate::Motion;

#[derive(Debug)]
pub struct MachineSettings {
    pub max_acceleration: f32, // mm/s^2
    pub max_deceleration: f32, // mm/s^2
    pub max_velocity: f32,     // mm/s
    pub max_angle_threshold: f32,
    pub straight_angle_threshold: f32,
    pub slowdown_factor: f32,
}

impl Default for MachineSettings {
    fn default() -> Self {
        Self {
            max_acceleration: 50.0,
            max_deceleration: 75.0,
            max_velocity: 100.0,
            max_angle_threshold: 45.0,
            straight_angle_threshold: 10.0,
            slowdown_factor: 0.5,
        }
    }
}

static DEFAULT_SETTINGS: LazyLock<MachineSettings> = LazyLock::new(MachineSettings::default);

#[derive(Debug, Clone)]
pub struct MotionSegment {
    pub start_position: [f32; 3],
    pub end_position: [f32; 3],
    pub distance: f32,
    pub direction: [f32; 3],
    pub v_max: f32,
    pub v_in: f32,
    pub v_out: f32,
}

#[derive(Debug, Default)]
pub struct MotionBuffer {
    buffer: Vec<MotionSegment>,

    weighted_queue_vector: [f32; 3],
}

impl MotionBuffer {
    pub fn queue(&mut self, motion: Motion) {
        let mut new_segment = self.create_segment(motion);

        if let Some(prev) = self.buffer.last_mut() {
            prev.v_out = new_segment.v_max;
            new_segment.v_in = prev.v_out;

            // "nudge" the weighted queue vector

            // 1) Compute the direction of the new segment
            let new_dir = normalize(&[
                new_segment.end_position[0] - new_segment.start_position[0],
                new_segment.end_position[1] - new_segment.start_position[1],
                new_segment.end_position[2] - new_segment.start_position[2],
            ]);

            // 2) Compute the angle between the new segment and the previous segment
            let angle = (new_dir[0] * prev.direction[0]
                + new_dir[1] * prev.direction[1]
                + new_dir[2] * prev.direction[2])
                .acos()
                .to_degrees();

            // 3) Compute the weight of the new segment
            let weight = 1.0 - (angle / DEFAULT_SETTINGS.max_angle_threshold).min(1.0);

            // 4) Update the weighted queue vector
            self.weighted_queue_vector[0] += new_dir[0] * weight;
            self.weighted_queue_vector[1] += new_dir[1] * weight;
            self.weighted_queue_vector[2] += new_dir[2] * weight;

            // 5) Normalize the weighted queue vector
            let magnitude = (self.weighted_queue_vector[0].powi(2)
                + self.weighted_queue_vector[1].powi(2)
                + self.weighted_queue_vector[2].powi(2))
            .sqrt();

            self.weighted_queue_vector[0] /= magnitude;
            self.weighted_queue_vector[1] /= magnitude;
            self.weighted_queue_vector[2] /= magnitude;

            // 6) Compute the angle between the weighted queue vector and the new segment
            let angle = (new_dir[0] * self.weighted_queue_vector[0]
                + new_dir[1] * self.weighted_queue_vector[1]
                + new_dir[2] * self.weighted_queue_vector[2])
                .acos()
                .to_degrees();

            // print out all the variables

            println!("new_dir: {:?}, prev_dir: {:?}, angle: {:.2}, weight: {:.2}, weighted_queue_vector: {:?}, angle_weighted: {:.2}", new_dir, prev.direction, angle, weight, self.weighted_queue_vector, angle);
        }

        self.buffer.push(new_segment);
    }

    pub fn dequeue(&mut self) -> Option<MotionSegment> {
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
                let start = if let Some(prev) = prev_segment {
                    prev.end_position
                } else {
                    [0.0, 0.0, 0.0]
                };

                let end = [
                    coords.x.unwrap_or(start[0]),
                    coords.y.unwrap_or(start[1]),
                    coords.z.unwrap_or(start[2]),
                ];

                let distance = ((end[0] - start[0]).powi(2)
                    + (end[1] - start[1]).powi(2)
                    + (end[2] - start[2]).powi(2))
                .sqrt();

                let direction =
                    normalize(&[end[0] - start[0], end[1] - start[1], end[2] - start[2]]);

                let v_max = feedrate;
                let v_in = 0.0;
                let v_out = 0.0;

                MotionSegment {
                    start_position: start,
                    end_position: end,
                    distance,
                    direction,
                    v_max,
                    v_in,
                    v_out,
                }
            }
            Motion::Rapid(_coords) => {
                todo!()
            }
        }
    }
}

#[derive(Debug)]
pub struct MachineState {
    pub x: f32,
    pub y: f32,
    pub z: f32,

    pub motion_buffer: MotionBuffer,
}

impl Default for MachineState {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            motion_buffer: MotionBuffer::default(),
        }
    }
}

impl MachineState {
    pub async fn move_to(&mut self, segment: MotionSegment) {
        let dt = 0.001; // 1 ms time step in seconds

        let a_max = DEFAULT_SETTINGS.max_acceleration;
        let d_max = DEFAULT_SETTINGS.max_deceleration;
        let v_in = segment.v_in;
        let v_out = segment.v_out;
        let v_max = segment.v_max.min(DEFAULT_SETTINGS.max_velocity);
        let distance = segment.distance;

        // 1) Compute times for each phase
        let t1 = (v_max - v_in) / a_max; // time to accelerate from v_in to v_max
        let t3 = (v_max - v_out) / d_max; // time to decelerate from v_max to v_out

        // distance covered in accel (phase 1) and decel (phase 3)
        let d_acc = 0.5 * (v_in + v_max) * t1;
        let d_dec = 0.5 * (v_max + v_out) * t3;

        // remaining distance for cruise (phase 2)
        let d_cruise = if distance > (d_acc + d_dec) {
            distance - (d_acc + d_dec)
        } else {
            0.0
        };

        // time at cruise speed (phase 2)
        let t2 = if v_max > 1e-6 { d_cruise / v_max } else { 0.0 };

        let t_total = t1 + t2 + t3;

        // 2) We'll do Euler steps from t=0..t_total
        let mut t = 0.0;
        let mut velocity = v_in;

        // We'll track how far we've traveled along the segment in 1D
        let mut dist_covered = 0.0;

        // For convenience, store local references
        let (sx, sy, sz) = (
            segment.start_position[0],
            segment.start_position[1],
            segment.start_position[2],
        );
        let dir = segment.direction;

        let mut accel = 0.0;
        while t < t_total {
            // figure out which phase
            match t {
                t_ph if t_ph < t1 => {
                    // accelerate
                    accel = a_max;
                }
                t_ph if t_ph < (t1 + t2) => {
                    // cruise
                    accel = 0.0;
                    velocity = v_max; // clamp to v_max
                }
                _ => {
                    // decelerate
                    accel = -d_max;
                }
            }

            // Apply acceleration for this dt
            let old_vel = velocity;
            velocity += accel * dt;

            // clamp velocity to [0, v_max] if you want
            if velocity > v_max {
                velocity = v_max;
            }
            if velocity < 0.0 {
                velocity = 0.0;
            }

            // average velocity this step
            let vel_avg = 0.5 * (old_vel + velocity);

            // distance traveled in this step
            let dist_step = vel_avg * dt;
            dist_covered += dist_step;

            // but don't exceed total distance
            if dist_covered > distance {
                dist_covered = distance;
            }

            // update machine position
            self.x = sx + dir[0] * dist_covered;
            self.y = sy + dir[1] * dist_covered;
            self.z = sz + dir[2] * dist_covered;

            // print or log
            println!(
                "t={:.3}: x={:.2}, y={:.2}, z={:.2}, dist={:.2}, vel={:.2}, accel={:.2}",
                t, self.x, self.y, self.z, dist_covered, velocity, accel
            );

            // break if we've reached the distance
            if (distance - dist_covered).abs() < 1e-6 {
                break;
            }

            // increment time
            t += dt;
            sleep(Duration::from_secs_f32(dt)).await;
        }

        // clamp final position
        self.x = segment.end_position[0];
        self.y = segment.end_position[1];
        self.z = segment.end_position[2];
    }

    pub fn queue_motion(&mut self, motions: Vec<Motion>) {
        for motion in motions {
            self.motion_buffer.queue(motion);
        }
    }

    pub async fn tick(&mut self) {
        if let Some(segment) = self.motion_buffer.dequeue() {
            self.move_to(segment).await;
        }
    }
}

fn normalize(direction: &[f32; 3]) -> [f32; 3] {
    let magnitude =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if magnitude == 0.0 {
        [0.0, 0.0, 0.0]
    } else {
        [
            direction[0] / magnitude,
            direction[1] / magnitude,
            direction[2] / magnitude,
        ]
    }
}
