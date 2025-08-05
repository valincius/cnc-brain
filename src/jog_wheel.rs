use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};

use crate::{
    InputResources,
    // motion::{MOTION_QUEUE, MotionCommand},
};

#[embassy_executor::task]
pub async fn task(inputs: InputResources) {
    let jog_a = Input::new(inputs.jog_a, Pull::Up);
    let jog_b = Input::new(inputs.jog_b, Pull::Up);

    Output::new(inputs.jog_pwr, Level::High);
    Timer::after(Duration::from_millis(100)).await;

    // Read initial state
    let mut last_state = (jog_a.is_high() as u8) << 1 | (jog_b.is_high() as u8);

    loop {
        let a = jog_a.is_high() as u8;
        let b = jog_b.is_high() as u8;
        let state = (a << 1) | b;

        if state != last_state {
            // Quadrature decoder table: index is (last_state << 2 | state)
            // Each transition maps to a +1, -1, or 0 (invalid or no movement)
            const QUAD_TABLE: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];
            let delta = QUAD_TABLE[((last_state << 2) | state) as usize];

            if delta != 0 {
                let dir = delta > 0;
                log::info!("Jogging {}", if dir { "forward" } else { "backward" });
                // MOTION_QUEUE
                //     .send(MotionCommand::Jog([0.01 * delta as f32, 0.0, 0.0], 10000.0))
                //     .await;
            }

            last_state = state;
        }

        Timer::after(Duration::from_millis(2)).await;
    }
}
