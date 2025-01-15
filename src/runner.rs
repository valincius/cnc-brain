use std::time::Duration;

use tokio::time::sleep;

use crate::{Coordinates, MotionSegment, Movement};

#[derive(Debug)]
pub struct MachineSettings {
    pub max_acceleration: f32, // mm/s^2
    pub max_velocity: f32,     // mm/s
}

impl Default for MachineSettings {
    fn default() -> Self {
        Self {
            max_acceleration: 50.0,
            max_velocity: 100.0,
        }
    }
}

#[derive(Debug)]
pub struct MachineState {
    pub settings: MachineSettings,

    pub x: f32,
    pub y: f32,
    pub z: f32,

    pub target_v: f32,

    pub motion_buffer: Vec<MotionSegment>,
}

impl Default for MachineState {
    fn default() -> Self {
        Self {
            settings: MachineSettings::default(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
            target_v: 0.0,
            motion_buffer: Vec::new(),
        }
    }
}

impl MachineState {
    pub async fn move_to(&mut self, segment: MotionSegment) {
        let dt = 0.001; // 1 ms time step in seconds

        let a_max = self.settings.max_acceleration;
        let v_in = segment.v_in;
        let v_out = segment.v_out;
        let v_max = segment.v_max.min(self.settings.max_velocity);
        let distance = segment.distance;

        // 1) Compute times for each phase
        let t1 = (v_max - v_in) / a_max; // time to accelerate from v_in to v_max
        let t3 = (v_max - v_out) / a_max; // time to decelerate from v_max to v_out

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
                    accel = -a_max;
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
                    self.update_previous_segment_vout(feedrate);

                    let segment = self.build_linear_segment(feedrate, coords);

                    // self.apply_corner_slowdown(&segment);

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

        let v_in = if let Some(prev) = self.motion_buffer.last() {
            prev.v_out
        } else {
            0.0
        };

        let v_out = 0.0; // will be updated later

        MotionSegment {
            start_position,
            end_position,
            distance,
            direction,
            v_max: feedrate,
            v_in,
            v_out,
        }
    }

    fn update_previous_segment_vout(&mut self, new_v_in: f32) {
        if let Some(prev_segment) = self.motion_buffer.last_mut() {
            prev_segment.v_out = new_v_in;
        }
    }

    fn apply_corner_slowdown(&mut self, new_seg: &MotionSegment) {
        let mut angle_sum = 0.0;
        let mut accumulated_distance = 0.0;
        let mut last_straight = None;

        for seg in self.motion_buffer.iter_mut().rev() {
            let dot = seg.direction[0] * new_seg.direction[0]
                + seg.direction[1] * new_seg.direction[1]
                + seg.direction[2] * new_seg.direction[2];

            let cos_angle = dot.clamp(-1.0, 1.0);
            let angle = cos_angle.acos(); // radians

            if angle.to_degrees() < 10.0 {
                // "Straight" motion, accumulate distance
                accumulated_distance += seg.distance;
                last_straight = Some(seg);
            } else {
                // Reset accumulated distance on sharp turns
                accumulated_distance = 0.0;
                angle_sum += angle;
            }

            // Stop accumulating if total angle exceeds threshold
            if let Some(last_straight) = &last_straight {
                let slowdown_distance = (last_straight.v_out * last_straight.v_out)
                    / (2.0 * self.settings.max_acceleration);

                if angle_sum.to_degrees() > 45.0 || accumulated_distance > slowdown_distance {
                    break;
                }
            }
        }

        if let Some(last_straight) = last_straight {
            last_straight.v_out *= 0.5;
        }
    }

    pub async fn tick(&mut self) {
        if self.motion_buffer.is_empty() {
            return;
        }

        let segment = self.motion_buffer.remove(0);

        self.move_to(segment).await;
    }
}
