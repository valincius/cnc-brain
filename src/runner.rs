/*************************************************
 * Constants
 *************************************************/
const DEFAULT_ACCELERATION: f32 = 50.0; // mm/s^2
const DT: f32 = 0.001; // 1 ms time step (for the run loop)
const EPSILON: f32 = 1e-6; // small float tolerance

/*************************************************
 * Program & Instructions
 *************************************************/
#[derive(Debug)]
pub enum Instruction {
    Rapid(f32, MotionInputs),
    Linear(f32, MotionInputs),
    // Arcs, Wait, etc. omitted for brevity
}

#[derive(Debug, Default)]
pub struct MotionInputs {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
}

/*************************************************
 * Machine State
 *************************************************/
#[derive(Debug, Default)]
pub struct MachineState {
    pub x: f32,
    pub y: f32,
    pub z: f32,

    pub motion_buffer: Vec<MotionSegment>,

    pub current_segment: usize,
    pub current_sub_segment: usize,
    pub t_elapsed_in_sub: f32,
    pub dist_prev: f32,
}

impl MachineState {
    pub fn set_pos(&mut self, x: f32, y: f32, z: f32) {
        self.x = x;
        self.y = y;
        self.z = z;

        println!("Position: X: {:.2}mm, Y: {:.2}mm, Z: {:.2}mm", x, y, z);
    }

    pub fn push_motions(&mut self, motions: Vec<MotionSegment>) {
        self.motion_buffer.extend(motions);
    }

    pub async fn step(&mut self) {
        if self.current_segment >= self.motion_buffer.len() {
            return;
        }

        let seg = &self.motion_buffer[self.current_segment];
        let sub = &seg.sub_segments[self.current_sub_segment];

        if self.t_elapsed_in_sub < sub.t {
            let dt = DT.min(sub.t - self.t_elapsed_in_sub);
            self.t_elapsed_in_sub += dt;

            let dist_new = distance_at_time(
                sub.v_start,
                sub.accel_start,
                sub.jerk,
                self.t_elapsed_in_sub,
            );
            let delta_dist = dist_new - self.dist_prev;
            self.dist_prev = dist_new;

            self.set_pos(
                self.x + delta_dist * seg.direction[0],
                self.y + delta_dist * seg.direction[1],
                self.z + delta_dist * seg.direction[2],
            );
        } else {
            // This sub-segment is done; go to next sub-segment or next segment
            self.next_sub_segment();
        }
    }

    fn next_sub_segment(&mut self) {
        let seg = &self.motion_buffer[self.current_segment];

        if self.current_sub_segment + 1 < seg.sub_segments.len() {
            self.current_sub_segment += 1;
        } else {
            self.current_segment += 1;
            self.current_sub_segment = 0;

            self.set_pos(
                seg.end_position[0],
                seg.end_position[1],
                seg.end_position[2],
            );
        }

        // Reset integration for next sub
        self.t_elapsed_in_sub = 0.0;
        self.dist_prev = 0.0;
    }
}

fn distance_at_time(v0: f32, a0: f32, j: f32, t: f32) -> f32 {
    v0 * t + 0.5 * a0 * t * t + (j * t * t * t) / 6.0
}

/*************************************************
 * S-curve Data Structures
 *************************************************/
#[derive(Debug, Clone)]
pub struct ScurveSubSegment {
    pub jerk: f32,        // jerk, mm/s^3 (constant during this sub-interval)
    pub accel_start: f32, // acceleration at start of interval
    pub accel_end: f32,   // acceleration at end of interval
    pub v_start: f32,     // velocity at start
    pub v_end: f32,       // velocity at end
    pub distance: f32,    // distance traveled in this sub-interval (1D)
    pub t: f32,           // total time for this sub-interval
}

/// Our "coarse" segment with assigned v_in, v_out
#[derive(Debug, Clone)]
pub struct MotionSegment {
    pub start_position: [f32; 3],
    pub end_position: [f32; 3],
    pub distance: f32,       // total 3D distance
    pub direction: [f32; 3], // normalized direction vector
    pub feedrate: f32,       // max velocity for this segment
    pub v_in: f32,           // assigned by multi-pass planning
    pub v_out: f32,          // assigned by multi-pass planning
    pub sub_segments: Vec<ScurveSubSegment>,
}

impl MotionSegment {
    /// Utility for how much 1D distance we have covered (if you need it for offset calculations).
    /// For simplicity here, we'll just say it's the sum of sub-segments that have already been popped.
    pub fn distance_covered(&self) -> f32 {
        let planned_subs_distance: f32 = self.sub_segments.iter().map(|s| s.distance).sum();
        self.distance - planned_subs_distance
    }
}

/*************************************************
 * Motion Planner
 *************************************************/
pub struct MotionPlanner;

impl MotionPlanner {
    pub fn from_program(program: &Vec<Instruction>) -> Vec<MotionSegment> {
        // 1) Build coarse segments
        let mut segments = Self::build_coarse_segments(program);

        if segments.is_empty() {
            return segments;
        }

        // 2) First forward pass: accelerate from v_in=0
        segments[0].v_in = 0.0;
        for i in 0..segments.len() {
            let seg = &mut segments[i];
            // v_out^2 = v_in^2 + 2*a*distance
            let v_allowed =
                ((seg.v_in * seg.v_in) + 2.0 * DEFAULT_ACCELERATION * seg.distance).sqrt();
            let v_out = v_allowed.min(seg.feedrate);
            seg.v_out = v_out;

            // next segment's v_in = seg.v_out
            if i + 1 < segments.len() {
                segments[i + 1].v_in = v_out;
            }
        }

        // 3) Backward pass
        for i in (0..segments.len() - 1).rev() {
            let (cur, nxt) = {
                let (left, right) = segments.split_at_mut(i + 1);
                (&mut left[i], &mut right[0])
            };

            let feasible_v_out =
                ((cur.v_in * cur.v_in) + 2.0 * DEFAULT_ACCELERATION * cur.distance).sqrt();
            let new_v_out = cur.v_out.min(feasible_v_out).min(cur.feedrate);

            if new_v_out > nxt.v_in {
                cur.v_out = nxt.v_in.min(new_v_out);
            } else {
                cur.v_out = new_v_out.min(nxt.feedrate);
            }
            nxt.v_in = cur.v_out;
        }

        // 4) second forward pass (optional)
        for i in 1..segments.len() {
            segments[i].v_in = segments[i - 1].v_out;
            let v_allowed = ((segments[i].v_in * segments[i].v_in)
                + 2.0 * DEFAULT_ACCELERATION * segments[i].distance)
                .sqrt();
            let v_out = v_allowed.min(segments[i].feedrate);
            segments[i].v_out = segments[i].v_out.min(v_out);
        }

        // 5) Build S-curve sub-segments
        for seg in segments.iter_mut() {
            seg.sub_segments = Self::build_scurve_sub_segments(seg);
        }

        for (i, seg) in segments.iter().enumerate() {
            println!("Segment #{i}, total dist={:.3}", seg.distance);
            for (j, s) in seg.sub_segments.iter().enumerate() {
                println!("  Sub #{j}: t={:.3}, dist={:.3}", s.t, s.distance);
            }
        }

        segments
    }

    fn build_coarse_segments(instructions: &Vec<Instruction>) -> Vec<MotionSegment> {
        let mut segments = Vec::new();
        let mut last_pos = [0.0, 0.0, 0.0];

        for instr in instructions {
            match instr {
                Instruction::Rapid(feed, inputs) | Instruction::Linear(feed, inputs) => {
                    let end_pos = [
                        inputs.x.unwrap_or(last_pos[0]),
                        inputs.y.unwrap_or(last_pos[1]),
                        inputs.z.unwrap_or(last_pos[2]),
                    ];
                    let d = distance_3d(last_pos, end_pos);

                    let dir = if d > EPSILON {
                        [
                            (end_pos[0] - last_pos[0]) / d,
                            (end_pos[1] - last_pos[1]) / d,
                            (end_pos[2] - last_pos[2]) / d,
                        ]
                    } else {
                        [0.0, 0.0, 0.0]
                    };

                    segments.push(MotionSegment {
                        start_position: last_pos,
                        end_position: end_pos,
                        distance: d,
                        direction: dir,
                        feedrate: *feed,
                        v_in: 0.0,
                        v_out: 0.0,
                        sub_segments: Vec::new(),
                    });
                    last_pos = end_pos;
                }
            }
        }
        segments
    }

    /// Build S-curve sub-segments (3-phase approximation).
    fn build_scurve_sub_segments(seg: &MotionSegment) -> Vec<ScurveSubSegment> {
        let mut sub_segs = Vec::new();

        if seg.distance < EPSILON {
            return sub_segs;
        }

        let v_in = seg.v_in;
        let v_out = seg.v_out;
        let v_cruise = seg.feedrate;

        // Possibly won't reach feedrate if distance is short or v_out is lower
        let v_peak = v_cruise.max(v_in).max(v_out);

        let t_acc = (v_peak - v_in) / DEFAULT_ACCELERATION;
        let t_dec = (v_peak - v_out) / DEFAULT_ACCELERATION;

        let d_acc = 0.5 * (v_in + v_peak) * t_acc;
        let d_dec = 0.5 * (v_peak + v_out) * t_dec;

        let d_total_needed = d_acc + d_dec;
        let d_cruise = if d_total_needed < seg.distance {
            seg.distance - d_total_needed
        } else {
            0.0
        };

        // --- Sub #1: accelerate ---
        if t_acc > EPSILON {
            // jerk = (a_final - a_start)/time, here a_start=0, a_final=accel
            // but let’s just say a_final=+ACCEL => j=ACCEL/t_acc
            let j = (DEFAULT_ACCELERATION - 0.0) / t_acc;
            sub_segs.push(ScurveSubSegment {
                jerk: j,
                accel_start: 0.0,
                accel_end: DEFAULT_ACCELERATION,
                v_start: v_in,
                v_end: v_peak,
                distance: d_acc,
                t: t_acc,
            });
        }

        // --- Sub #2: cruise ---
        if d_cruise > EPSILON && v_peak > EPSILON {
            let t_cruise = d_cruise / v_peak;
            sub_segs.push(ScurveSubSegment {
                jerk: 0.0,
                accel_start: 0.0,
                accel_end: 0.0,
                v_start: v_peak,
                v_end: v_peak,
                distance: d_cruise,
                t: t_cruise,
            });
        }

        // --- Sub #3: decelerate ---
        if t_dec > EPSILON {
            let j = (0.0 - DEFAULT_ACCELERATION) / t_dec;
            sub_segs.push(ScurveSubSegment {
                jerk: j,
                accel_start: 0.0,
                accel_end: -DEFAULT_ACCELERATION,
                v_start: v_peak,
                v_end: v_out,
                distance: d_dec,
                t: t_dec,
            });
        }

        sub_segs
    }
}

/*************************************************
 * Utilities
 *************************************************/
fn distance_3d(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
}
