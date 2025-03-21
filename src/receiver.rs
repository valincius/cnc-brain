use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_usb_logger::ReceiverHandler;

use crate::{CONTROLLER_CHANNEL, ControllerCommand};

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = core::str::from_utf8(data) {
            let data = data.trim();

            let mut parts = data.split_whitespace();
            if let Some(cmd) = parts.next() {
                match cmd {
                    "go" => {
                        let x = parts.next().unwrap_or("0").parse().unwrap_or(0.0);
                        let y = parts.next().unwrap_or("0").parse().unwrap_or(0.0);
                        let z = parts.next().unwrap_or("0").parse().unwrap_or(0.0);

                        let speed = parts.next().unwrap_or("1000").parse().unwrap_or(1000.0);

                        CONTROLLER_CHANNEL
                            .send(ControllerCommand::MoveTo([x, y, z], speed))
                            .await;
                    }
                    "stop" => {
                        CONTROLLER_CHANNEL.send(ControllerCommand::Stop).await;
                    }
                    "q" => {
                        reset_to_usb_boot(0, 0); // Restart the chip
                    }
                    _ => {
                        log::info!("Unknown command: {}", cmd);
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
pub async fn usb_comm_task(driver: embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Trace, driver, Handler);
}
