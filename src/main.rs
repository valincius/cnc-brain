#![no_std]
#![no_main]

use core::str;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{PIO0, USB},
    pio::Pio,
    rom_data::reset_to_usb_boot,
    usb::Driver,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embassy_usb_logger::ReceiverHandler;
use heapless::Vec;
use micromath::F32Ext;
use panic_probe as _;

const CHANNEL_BUFFER_CAPACITY: usize = 64;

static CHANNEL: embassy_sync::channel::Channel<
    ThreadModeRawMutex,
    Command,
    CHANNEL_BUFFER_CAPACITY,
> = Channel::new();

enum Direction {
    CW,
    CCW,
}

impl Direction {
    pub fn from_sign(sign: f32) -> Self {
        if sign >= 0.0 {
            Self::CW
        } else {
            Self::CCW
        }
    }
}

enum AxisCommand {
    Run(u32, Direction),
    Stop,
}

enum Command {
    SetAxes([AxisCommand; 3]),
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let mut led = Output::new(p.PIN_25, Level::Low);
    led.set_high();

    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        mut sm2,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let x_step_pin = common.make_pio_pin(p.PIN_0);
    let mut x_dir_pin = Output::new(p.PIN_1, Level::Low);

    let y_step_pin = common.make_pio_pin(p.PIN_2);
    let mut y_dir_pin = Output::new(p.PIN_3, Level::Low);

    let z_step_pin = common.make_pio_pin(p.PIN_4);
    let mut z_dir_pin = Output::new(p.PIN_5, Level::Low);

    let program = pio_proc::pio_file!("./src/prog.pio");

    let mut cfg = embassy_rp::pio::Config::default();
    cfg.use_program(&common.load_program(&program.program), &[]);
    cfg.set_out_pins(&[&x_step_pin]);
    cfg.set_set_pins(&[&x_step_pin]);
    sm0.set_config(&cfg);
    sm0.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&x_step_pin]);
    sm0.set_enable(true);

    spawner.spawn(controller()).unwrap();

    loop {
        match CHANNEL.receive().await {
            Command::SetAxes(cmd) => match &cmd[0] {
                AxisCommand::Run(speed, dir) => {
                    x_dir_pin.set_level(match dir {
                        Direction::CW => Level::Low,
                        Direction::CCW => Level::High,
                    });
                    sm0.tx().wait_push(*speed).await;
                }
                AxisCommand::Stop => {
                    sm0.tx().wait_push(0).await;
                }
            },
            _ => {}
        }
    }
}

#[embassy_executor::task]
async fn controller() {
    let mut machine = MotionSystem::new();
    machine.move_to([100.0, 100.0, 0.0], 1000.0).await;
}

pub struct MotionSystem {
    current_pos: [f32; 3], // Current position of each axis
    current_speed: f32,
    accel: f32,           // Acceleration (steps/sec^2)
    target_pos: [f32; 3], // Target position
    max_speed: f32,       // Max speed along the line (steps/sec)
}

impl MotionSystem {
    pub fn new() -> Self {
        Self {
            current_pos: [0.0; 3],
            current_speed: 0.0,
            target_pos: [0.0; 3],
            max_speed: 0.0,
            accel: 1000.0,
        }
    }

    pub fn current_pos(&self) -> [f32; 3] {
        self.current_pos
    }

    pub fn current_speed(&self) -> f32 {
        self.current_speed
    }

    pub async fn move_to(&mut self, target: [f32; 3], max_speed: f32) {
        self.target_pos = target;
        self.max_speed = max_speed;

        self._move().await;
    }

    async fn _move(&mut self) {
        let start_pos = self.current_pos;

        let dx = self.target_pos[0] - start_pos[0];
        let dy = self.target_pos[1] - start_pos[1];
        let dz = self.target_pos[2] - start_pos[2];

        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        if dist <= f32::EPSILON {
            return;
        }

        let accel = self.accel;
        let max_speed = self.max_speed;
        let mut speed_along_line: f32 = 0.0;
        let mut traveled = 0.0;

        let mut last_update = Instant::now();
        let mut ticker = Ticker::every(Duration::from_millis(1));

        loop {
            let now = Instant::now();
            let dt = now.duration_since(last_update).as_micros() as f32 / 1_000_000.0;
            last_update = now;

            let distance_to_go = dist - traveled;
            if distance_to_go <= 0.0 {
                self.current_speed = 0.0;
                self.current_pos = self.target_pos;

                CHANNEL
                    .send(Command::SetAxes([const { AxisCommand::Stop }; 3]))
                    .await;
                break;
            }

            // Check deceleration distance: v^2 / (2*a)
            let decel_distance = speed_along_line.powi(2) / (2.0 * accel);
            if decel_distance >= distance_to_go {
                speed_along_line -= accel * dt;
                speed_along_line = speed_along_line.max(0.0);
            } else {
                if speed_along_line < max_speed {
                    speed_along_line += accel * dt;
                    speed_along_line = speed_along_line.min(max_speed);
                }
            }

            self.current_speed = speed_along_line;

            // Move along the line by delta_dist = speed * dt
            let delta_dist = speed_along_line * dt;
            traveled += delta_dist;

            let ratio = delta_dist / dist;
            self.current_pos[0] += dx * ratio;
            self.current_pos[1] += dy * ratio;
            self.current_pos[2] += dz * ratio;

            let speed_x = (speed_along_line * (dx / dist)).abs() as u32;
            let speed_y = (speed_along_line * (dy / dist)).abs() as u32;
            let speed_z = (speed_along_line * (dz / dist)).abs() as u32;

            let cmd = [
                AxisCommand::Run(speed_x, Direction::from_sign(dx)),
                AxisCommand::Run(speed_y, Direction::from_sign(dy)),
                AxisCommand::Run(speed_z, Direction::from_sign(dz)),
            ];
            CHANNEL.send(Command::SetAxes(cmd)).await;

            ticker.next().await;
        }
    }
}

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = str::from_utf8(data) {
            let data = data.trim();

            let mut parts = data.split_whitespace();
            if let Some(cmd) = parts.next() {
                match cmd {
                    // "speed" => {
                    //     if let Some(speed) = parts.next() {
                    //         if let Ok(speed) = speed.parse::<u32>() {
                    //             CHANNEL.send(Command::Speed(speed)).await;
                    //         }
                    //     }
                    // }
                    "q" => {
                        reset_to_usb_boot(0, 0); // Restart the chip
                    }
                    _ => {
                        log::info!("Recieved: {:?}", data);
                    }
                }
            }
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, Handler);
}

/*
// let program_with_defines = pio_proc::pio_file!(
    //     "./src/prog.pio",
    //     select_program("stepper_program"), // Optional if only one program in the file
    // );
    // let program = program_with_defines.program;

    // let mut planner = MotionPlanner::new(DEFAULT_ACCEL_MAX, DEFAULT_JERK_MAX);

    // // Add a couple of linear blocks
    // planner
    //     .add_block(MotionBlock::new(
    //         [0.0, 0.0, 0.0],
    //         [10.0, 0.0, 0.0],
    //         1200.0, // feed rate in mm/min
    //         300.0,  // acceleration
    //         MotionType::Linear,
    //     ))
    //     .unwrap();

    // planner
    //     .add_block(MotionBlock::new(
    //         [10.0, 0.0, 0.0],
    //         [10.0, 10.0, 0.0],
    //         600.0,
    //         300.0,
    //         MotionType::Linear,
    //     ))
    //     .unwrap();

    // // Run lookahead to refine velocities
    // planner.lookahead_pass();

    // let mut executor = StepperExecutor::new();

    // // Simulate some real-time updates (e.g., 100 Hz or 1 kHz loop)
    // let dt = 0.01; // 10 ms
    // for _ in 0..2000 {
    //     executor.update(&mut planner, dt);

    //     // In a real embedded app, you might break out when all blocks are done:
    //     if !executor.current_block.is_some() && !planner.has_blocks() {
    //         break;
    //     }
    // }

    // // At this point, we'd have moved through all blocks.
    // assert!(!planner.has_blocks(), "All blocks should be executed");
*/

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
