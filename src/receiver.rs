use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_usb_logger::ReceiverHandler;

use crate::{CONTROLLER_CHANNEL, ControllerCommand};

pub struct CommandHandler;

impl CommandHandler {
    async fn process_command(&self, command_line: &str) {
        let mut parts = command_line.split_whitespace();
        if let Some(cmd) = parts.next() {
            match cmd {
                "go" => {
                    let x = parts.next().unwrap_or("0").parse().unwrap_or(0);
                    let y = parts.next().unwrap_or("0").parse().unwrap_or(0);
                    let z = parts.next().unwrap_or("0").parse().unwrap_or(0);
                    let speed = parts.next().unwrap_or("1000").parse().unwrap_or(1000.0);

                    CONTROLLER_CHANNEL
                        .send(ControllerCommand::MoveTo([x, y, z], speed))
                        .await;
                }
                "zero" => {
                    CONTROLLER_CHANNEL.send(ControllerCommand::Zero).await;
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

impl ReceiverHandler for CommandHandler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = str::from_utf8(data) {
            let data = data.trim();
            if data.is_empty() {
                return;
            }

            self.process_command(data).await;
        }
    }

    fn new() -> Self {
        CommandHandler
    }
}

#[embassy_executor::task]
pub async fn usb_comm_task(driver: embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Trace, driver, CommandHandler);
}
