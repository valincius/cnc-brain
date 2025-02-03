#![no_std]
#![no_main]

use cnc_brain::prelude::*;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use heapless::Vec;
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

static CHANNEL: CommandChannel = Channel::new();

const DEFAULT_ACCEL_MAX: f32 = 300.0; // mm/s^2
const DEFAULT_JERK_MAX: f32 = 1000.0; // mm/s^3 (if implementing jerk control)

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let program_with_defines = pio_proc::pio_file!(
        "./src/prog.pio",
        select_program("stepper_program"), // Optional if only one program in the file
    );
    let program = program_with_defines.program;

    let mut planner = MotionPlanner::new(DEFAULT_ACCEL_MAX, DEFAULT_JERK_MAX);

    // Add a couple of linear blocks
    planner
        .add_block(MotionBlock::new(
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            1200.0, // feed rate in mm/min
            300.0,  // acceleration
            MotionType::Linear,
        ))
        .unwrap();

    planner
        .add_block(MotionBlock::new(
            [10.0, 0.0, 0.0],
            [10.0, 10.0, 0.0],
            600.0,
            300.0,
            MotionType::Linear,
        ))
        .unwrap();

    // Run lookahead to refine velocities
    planner.lookahead_pass();

    let mut executor = StepperExecutor::new();

    // Simulate some real-time updates (e.g., 100 Hz or 1 kHz loop)
    let dt = 0.01; // 10 ms
    for _ in 0..2000 {
        executor.update(&mut planner, dt);

        // In a real embedded app, you might break out when all blocks are done:
        if !executor.current_block.is_some() && !planner.has_blocks() {
            break;
        }
    }

    // At this point, we'd have moved through all blocks.
    assert!(!planner.has_blocks(), "All blocks should be executed");
}

/// Configuration: number of blocks to buffer in the planner
const PLANNER_CAPACITY: usize = 16;

//----------------------------------------------
// 1) MOTION BLOCK + TYPES
//----------------------------------------------

#[derive(Debug, Clone, Copy)]
pub enum MotionType {
    Linear,
    Arc {
        center: [f32; 2],
        radius: f32,
        clockwise: bool,
    },
    // Extend for cubic splines, etc.
}

#[derive(Debug, Clone, Copy)]
pub struct MotionBlock {
    // Basic geometry
    pub start_pos: [f32; 3],
    pub end_pos: [f32; 3],
    pub distance: f32, // Precomputed length of this move

    // G-code feed rate request (mm/min or mm/s)
    pub feed_rate: f32,

    // Limits (if different from global)
    pub accel_limit: f32,
    pub decel_limit: f32,

    // Computed velocities
    pub entry_velocity: f32,
    pub exit_velocity: f32,
    pub max_velocity: f32,

    // Motion type
    pub motion_type: MotionType,
}

impl MotionBlock {
    /// Create a new MotionBlock with default velocities = 0.
    pub fn new(
        start: [f32; 3],
        end: [f32; 3],
        feed_rate: f32,
        accel: f32,
        motion_type: MotionType,
    ) -> Self {
        // Simple distance calculation for linear; arcs would need special handling
        let dist = distance_3d(start, end);

        Self {
            start_pos: start,
            end_pos: end,
            distance: dist,
            feed_rate,
            accel_limit: accel,
            decel_limit: accel, // same for simplicity
            entry_velocity: 0.0,
            exit_velocity: 0.0,
            max_velocity: 0.0,
            motion_type,
        }
    }
}

/// Helper function for Euclidean distance in 3D.
fn distance_3d(a: [f32; 3], b: [f32; 3]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

//----------------------------------------------
// 2) MOTION PLANNER
//----------------------------------------------

pub struct MotionPlanner {
    // Our block buffer: we store upcoming segments for lookahead
    pub blocks: Vec<MotionBlock, PLANNER_CAPACITY>,

    // Machine config
    pub accel_max: f32,
    pub jerk_max: f32, // if you want jerk-limited planning
}

impl MotionPlanner {
    pub fn new(accel_max: f32, jerk_max: f32) -> Self {
        Self {
            blocks: Vec::new(),
            accel_max,
            jerk_max,
        }
    }

    /// Add a new motion block to the buffer
    pub fn add_block(&mut self, mut block: MotionBlock) -> Result<(), &'static str> {
        // If the user didn't precompute distance, do so here (for linear moves).
        if block.distance <= 1e-6 {
            // For arcs or super tiny moves, handle differently if needed.
            block.distance = distance_3d(block.start_pos, block.end_pos);
        }

        // Basic velocity initialization (trapezoid profile concept).
        // For a first pass, set entry=0, exit= feed_rate, etc.
        // We'll refine these with lookahead.
        block.entry_velocity = 0.0;
        block.exit_velocity = block.feed_rate / 60.0; // If feed_rate is mm/min, convert to mm/s
        block.max_velocity = block.exit_velocity; // This is simplistic, real logic is more complex.

        self.blocks.push(block).unwrap();
        Ok(())
    }

    /// Perform a multi-pass lookahead on the stored blocks to enforce
    /// acceleration and corner limits. (Simplified trapezoidal approach.)
    pub fn lookahead_pass(&mut self) {
        let count = self.blocks.len();
        if count < 2 {
            return; // Nothing to plan with only one block
        }

        // 1) Forward pass: limit entry velocity based on the previous exit velocity + accel
        for i in 1..count {
            let (prev, curr) = {
                // Borrow checker trick: do it in scope
                let (left, right) = self.blocks.split_at_mut(i);
                (&mut left[i - 1], &mut right[0])
            };

            // The maximum entry velocity for `curr` given `prev.exit_velocity`
            // and acceleration constraints:
            let accel = curr.accel_limit.max(self.accel_max);
            let dv = (prev.exit_velocity * prev.exit_velocity) + 2.0 * accel * curr.distance;
            let max_entry_v = dv.sqrt();

            // We clamp the `curr.entry_velocity`
            if curr.entry_velocity > max_entry_v {
                curr.entry_velocity = max_entry_v;
            }
        }

        // 2) Reverse pass: limit exit velocity based on next block's entry velocity + decel
        // (We could do multiple iterations to converge, but let's keep it simple.)
        for i in (0..count - 1).rev() {
            let (curr, next) = {
                let (left, right) = self.blocks.split_at_mut(i + 1);
                (&mut left[i], &mut right[0])
            };

            let decel = curr.decel_limit.max(self.accel_max);
            let dv = (next.entry_velocity * next.entry_velocity) + 2.0 * decel * next.distance;
            let max_exit_v = dv.sqrt();

            if curr.exit_velocity > max_exit_v {
                curr.exit_velocity = max_exit_v;
            }
        }

        // 3) Recalculate final max velocities for each block
        //    (In a real approach, you'd do a second forward pass, etc.)
        for i in 0..count {
            let block = &mut self.blocks[i];
            // Typically max_velocity is min of feed constraint and accel-limited velocity
            let feed_v = block.feed_rate / 60.0; // if feed_rate is mm/min
            block.max_velocity = feed_v.min(block.entry_velocity).min(block.exit_velocity);
        }
    }

    /// Check if we have any blocks ready to execute
    pub fn has_blocks(&self) -> bool {
        !self.blocks.is_empty()
    }

    /// Pop the next block for execution (FIFO).
    pub fn get_next_block(&mut self) -> Option<MotionBlock> {
        if !self.blocks.is_empty() {
            Some(self.blocks.remove(0))
        } else {
            None
        }
    }
}

//----------------------------------------------
// 3) STEPPER EXECUTOR
//----------------------------------------------

/// A simple stepper executor that consumes blocks from the planner and "executes" them.
/// In a real system, you’d generate precise step timings in an interrupt/timer context.
pub struct StepperExecutor {
    current_block: Option<MotionBlock>,

    // Real-time counters
    distance_covered: f32,
    current_velocity: f32,
}

impl StepperExecutor {
    pub fn new() -> Self {
        Self {
            current_block: None,
            distance_covered: 0.0,
            current_velocity: 0.0,
        }
    }

    /// In a real system, you might call this from a high-frequency timer interrupt.
    /// For simplicity, we assume `dt` seconds pass between calls.
    pub fn update(&mut self, planner: &mut MotionPlanner, dt: f32) {
        // 1) If we have no current block, fetch one
        if self.current_block.is_none() {
            self.current_block = planner.get_next_block();
            if let Some(ref block) = self.current_block {
                // Initialize
                self.current_velocity = block.entry_velocity;
                self.distance_covered = 0.0;
            }
        }

        // 2) Execute the current block
        if let Some(ref mut block) = self.current_block {
            // Simplified trapezoidal approach:
            // We'll accelerate until we reach block.max_velocity, then hold, then decelerate,
            // for demonstration. In reality, you'd do more precise math for steps.

            let target_velocity = block.max_velocity;
            let accel = block.accel_limit; // ignoring jerk-limits here
            let decel_distance = 0.5
                * (block.exit_velocity * block.exit_velocity - target_velocity * target_velocity)
                / -accel;

            // Check if we need to decelerate soon
            let distance_left = block.distance - self.distance_covered;
            if distance_left <= decel_distance {
                // Decelerate
                let dv = accel * dt;
                if self.current_velocity > block.exit_velocity {
                    self.current_velocity = (self.current_velocity - dv).max(block.exit_velocity);
                }
            } else {
                // Accelerate or hold
                if self.current_velocity < target_velocity {
                    let dv = accel * dt;
                    self.current_velocity = (self.current_velocity + dv).min(target_velocity);
                }
            }

            // Move the motor (conceptually)
            let ds = self.current_velocity * dt;
            self.distance_covered += ds;

            // In a real system, we would translate `ds` into step pulses based on steps/mm.
            // e.g., `steps = (ds * steps_per_mm)`, then toggle step pins accordingly.

            // Check if the block is done
            if self.distance_covered >= block.distance {
                // Completed this segment
                self.current_block = None;
            }
        }
    }
}
