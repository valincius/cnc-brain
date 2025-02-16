use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_usb_logger::ReceiverHandler;

use crate::{ControllerCommand, CONTROLLER_CHANNEL};

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = core::str::from_utf8(data) {
            let data = data.trim();

            let mut parts = data.split_whitespace();
            if let Some(cmd) = parts.next() {
                match cmd {
                    "go" => {
                        CONTROLLER_CHANNEL
                            .send(ControllerCommand::MoveTo([250.0, 250.0, 250.0], 7500.0))
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
