use std::{sync::LazyLock, time::Duration};

use tokio::time::sleep;

use crate::{Coordinates, MotionSegment, Movement};

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

#[derive(Debug, Default)]
pub struct MotionBuffer(Vec<MotionSegment>);

impl MotionBuffer {
    pub fn push(&mut self, mut segment: MotionSegment) {
        if let Some(prev) = self.0.last_mut() {
            // Set the new segment's `v_in` to the previous segment's `v_out`
            segment.v_in = prev.v_out;

            // Temporarily calculate the previous segment's `v_out` based on the new segment
            prev.v_out = segment.v_max;
        }

        // Apply slowdown logic
        self.apply_corner_slowdown(&segment);

        // Ensure the previous segment's `v_out` is correctly updated after slowdown
        if let Some(prev) = self.0.last_mut() {
            prev.v_out = segment.v_in;
        }

        self.0.push(segment);
    }

    pub fn pop(&mut self) -> Option<MotionSegment> {
        if self.0.is_empty() {
            None
        } else {
            Some(self.0.remove(0))
        }
    }

    pub fn last(&self) -> Option<&MotionSegment> {
        self.0.last()
    }

    pub fn update_vout_and_propogate_forward(&mut self, idx: usize, v_out: f32) {
        let len = self.0.len();
        if idx >= len {
            return;
        }

        // Update the current segment's v_out
        self.0[idx].v_out = v_out;

        if idx + 1 < len {
            // Calculate the maximum allowed v_in for the next segment
            let v_max_next = self.0[idx + 1].v_max;
            let v_in = ((v_out * v_out) + (v_max_next * v_max_next))
                / (2.0 * DEFAULT_SETTINGS.max_acceleration);

            // Ensure the next segment's v_in respects the constraints
            let v_in_clamped = v_in.min(v_max_next);

            // Recursively propagate the velocity
            self.update_vout_and_propogate_forward(idx + 1, v_in_clamped);

            println!(
                "Updating idx: {}, new v_out: {:.2}, new v_in: {:.2}",
                idx, v_out, v_in
            );
        }
    }

    pub fn apply_corner_slowdown(&mut self, new_seg: &MotionSegment) {
        let mut angle_sum: f32 = 0.0;
        let mut accumulated_distance = 0.0;
        let mut last_straight = None;
        let mut last_straight_idx = 0;
        let len = self.0.len();

        for (i, seg) in self.0.iter_mut().rev().enumerate() {
            let idx = len - 1 - i; // Convert reversed index to original index

            // Compute angle between segments
            let dot = seg
                .direction
                .iter()
                .zip(new_seg.direction.iter())
                .map(|(a, b)| a * b)
                .sum::<f32>();
            let cos_angle = dot.clamp(-1.0, 1.0);
            let angle = cos_angle.acos(); // Radians

            if angle.to_degrees() < DEFAULT_SETTINGS.straight_angle_threshold {
                // "Straight" motion
                accumulated_distance += seg.distance;
                last_straight = Some(seg);
                last_straight_idx = idx;
            } else {
                // Sharp turn detected, reset distance
                accumulated_distance = 0.0;
                angle_sum += angle;
            }

            // Break if angle exceeds the threshold or distance is sufficient for slowdown
            if let Some(last_straight) = &last_straight {
                let slowdown_distance = (last_straight.v_out * last_straight.v_out)
                    / (2.0 * DEFAULT_SETTINGS.max_acceleration);

                if angle_sum.to_degrees() > DEFAULT_SETTINGS.max_angle_threshold
                    || accumulated_distance > slowdown_distance
                {
                    break;
                }
            }
        }

        // Apply slowdown at the last straight segment
        if let Some(last_straight) = last_straight {
            let v_out = last_straight.v_out * DEFAULT_SETTINGS.slowdown_factor;

            println!(
                "Segment idx: {}, angle_sum: {:.2}, accumulated_distance: {:.2}, v_out: {:.2}, v_in: {:.2}",
                last_straight_idx, angle_sum.to_degrees(), accumulated_distance, last_straight.v_out, last_straight.v_in
            );

            self.update_vout_and_propogate_forward(last_straight_idx, v_out);
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
        let mut accel = 0.0;

        // We'll track how far we've traveled along the segment in 1D
        let mut dist_covered = 0.0;

        // For convenience, store local references
        let (sx, sy, sz) = (
            segment.start_position[0],
            segment.start_position[1],
            segment.start_position[2],
        );
        let dir = segment.direction;

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

    pub fn queue_motion(&mut self, motions: Vec<Movement>) {
        for motion in motions {
            match motion {
                Movement::Linear { feedrate, coords } => {
                    let segment = self.build_linear_segment(feedrate, coords);

                    self.motion_buffer.push(segment);
                }
                Movement::Rapid(_) => {
                    // If you want to handle rapids differently, do it here
                }
            }
        }
    }

    /// Helper that builds a new MotionSegment from a feedrate + target coords
    fn build_linear_segment(&self, feedrate: f32, coords: Coordinates) -> MotionSegment {
        // Where the new segment starts:
        let (start_x, start_y, start_z) = if let Some(prev) = self.motion_buffer.last() {
            (
                prev.end_position[0],
                prev.end_position[1],
                prev.end_position[2],
            )
        } else {
            (self.x, self.y, self.z)
        };

        let start_position = [start_x, start_y, start_z];

        // Where the new segment ends:
        let end_x = coords.x.unwrap_or(self.x);
        let end_y = coords.y.unwrap_or(self.y);
        let end_z = coords.z.unwrap_or(self.z);

        let end_position = [end_x, end_y, end_z];

        // Compute distance + direction
        let dx = end_x - start_x;
        let dy = end_y - start_y;
        let dz = end_z - start_z;
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

        // Avoid divide-by-zero if coords are the same
        let direction = if distance > 1e-9 {
            [dx / distance, dy / distance, dz / distance]
        } else {
            [0.0, 0.0, 0.0]
        };

        MotionSegment {
            start_position,
            end_position,
            distance,
            direction: normalize(&direction),
            v_max: feedrate,
            v_in: feedrate,
            v_out: feedrate,
        }
    }

    pub async fn tick(&mut self) {
        if let Some(segment) = self.motion_buffer.pop() {
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
