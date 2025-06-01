use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};

use crate::InputResources;

#[embassy_executor::task]
pub async fn task(inputs: InputResources) {
    let jog_a = Input::new(inputs.jog_a, Pull::Up);
    let jog_b = Input::new(inputs.jog_b, Pull::Up);
    let jog_a2 = Input::new(inputs.jog_a2, Pull::Up);
    let jog_b2 = Input::new(inputs.jog_b2, Pull::Up);

    Output::new(inputs.jog_pwr, Level::High);
    Timer::after(Duration::from_millis(100)).await;

    let mut last_a = jog_a.is_high();
    let mut last_b = jog_b.is_high();

    let mut last_a2 = jog_a2.is_high();
    let mut last_b2 = jog_b2.is_high();

    loop {
        let current_a = jog_a.is_high();
        let current_b = jog_b.is_high();
        let current_a2 = jog_a2.is_high();
        let current_b2 = jog_b2.is_high();

        if current_a != last_a || current_b != last_b {
            if !current_a && last_b {
                // Jog wheel turned clockwise
                log::info!("Jog wheel turned clockwise");
            } else if !current_b && last_a {
                // Jog wheel turned counter-clockwise
                log::info!("Jog wheel turned counter-clockwise");
            }
            last_a = current_a;
            last_b = current_b;
        }

        if current_a2 != last_a2 || current_b2 != last_b2 {
            if !current_a2 && last_b2 {
                // Jog wheel 2 turned clockwise
                log::info!("Jog wheel 2 turned clockwise");
            } else if !current_b2 && last_a2 {
                // Jog wheel 2 turned counter-clockwise
                log::info!("Jog wheel 2 turned counter-clockwise");
            }
            last_a2 = current_a2;
            last_b2 = current_b2;
        }

        Timer::after(Duration::from_millis(50)).await; // Polling interval
    }
}
