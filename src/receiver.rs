use core::cell::RefCell;
use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_usb_logger::ReceiverHandler;

use crate::{CONTROLLER_CHANNEL, ControllerCommand};

const BUFFER_SIZE: usize = 128;

#[derive(Clone)]
struct CommandState {
    buffer: [u8; BUFFER_SIZE],
    buf_len: usize,
}

// Create a global mutex protecting our command state.
// Note: The ThreadModeRawMutex is appropriate if you're only using it in a single-threaded context.
static GLOBAL_COMMAND_STATE: Mutex<ThreadModeRawMutex, RefCell<CommandState>> =
    Mutex::new(RefCell::new(CommandState {
        buffer: [0; BUFFER_SIZE],
        buf_len: 0,
    }));

pub struct CommandHandler;

impl CommandHandler {
    async fn process_command(&self, command_line: &str) {
        let mut parts = command_line.split_whitespace();
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

impl ReceiverHandler for CommandHandler {
    async fn handle_data(&self, data: &[u8]) {
        // Lock the global state. This mutex is async-aware.
        let mut state = { GLOBAL_COMMAND_STATE.lock().await.borrow().clone() };

        // Calculate available space and copy new data into our fixed buffer.
        let available_space = BUFFER_SIZE - state.buf_len;
        let to_copy = data.len().min(available_space);
        state.buffer[state.buf_len..state.buf_len + to_copy].copy_from_slice(&data[..to_copy]);
        state.buf_len += to_copy;

        // Process complete commands terminated by '\n'
        while let Some(newline_pos) = state.buffer[..state.buf_len]
            .iter()
            .position(|&b| b == b'\n')
        {
            // Extract one complete line (including the newline)
            let line = &state.buffer[..=newline_pos];
            if let Ok(line_str) = core::str::from_utf8(line) {
                let command = line_str.trim(); // Remove newline and surrounding whitespace
                self.process_command(command).await;
            } else {
                log::error!("Received invalid UTF-8 data");
            }

            // Shift the remaining bytes to the beginning of the buffer
            let remaining = state.buf_len - (newline_pos + 1);
            for i in 0..remaining {
                state.buffer[i] = state.buffer[i + newline_pos + 1];
            }
            state.buf_len = remaining;

            {
                GLOBAL_COMMAND_STATE.lock().await.replace(state.clone());
            }
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
