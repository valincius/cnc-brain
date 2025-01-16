#![no_std]
#![no_main]

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let led = Output::new(p.PIN_17, Level::Low);
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(500))));
}

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

// #![no_std]
// #![no_main]

// use cortex_m::singleton;
// use cortex_m_rt::entry;
// use hal::dma::{double_buffer, single_buffer, DMAExt};
// use hal::gpio::{FunctionPio0, Pin};
// use hal::pac;
// use hal::pio::PIOExt;
// use hal::sio::Sio;
// use panic_halt as _;
// use rp2040_hal as hal;

// #[link_section = ".boot2"]
// #[used]
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// #[entry]
// fn main() -> ! {
//     let mut pac = pac::Peripherals::take().unwrap();

//     let sio = Sio::new(pac.SIO);
//     let pins = hal::gpio::Pins::new(
//         pac.IO_BANK0,
//         pac.PADS_BANK0,
//         sio.gpio_bank0,
//         &mut pac.RESETS,
//     );

//     // configure LED pin for Pio0.
//     let led: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();
//     // PIN id for use inside of PIO
//     let led_pin_id = led.id().num;

//     // HELLO WORLD in morse code:
//     // .... . .-.. .-.. --- / .-- --- .-. .-.. -..
//     #[allow(clippy::unusual_byte_groupings)]
//     let message = [
//         0b10101010_00100010_11101010_00101110,
//         0b10100011_10111011_10000000_10111011,
//         0b10001110_11101110_00101110_10001011,
//         0b10101000_11101010_00000000_00000000,
//     ];

//     // Define a PIO program which reads data from the TX FIFO bit by bit, configures the LED
//     // according to the data, and then writes the data back to the RX FIFO.
//     let program = pio_proc::pio_asm!(
//         ".wrap_target",
//         "    out x, 1",
//         "    mov pins, x",
//         "    in x, 1 [13]",
//         ".wrap"
//     );

//     // Initialize and start PIO
//     let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
//     let installed = pio.install(&program.program).unwrap();
//     let (mut sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
//         .out_pins(led_pin_id, 1)
//         .clock_divisor_fixed_point(0, 0) // as slow as possible (0 is interpreted as 65536)
//         .autopull(true)
//         .autopush(true)
//         .build(sm0);
//     // The GPIO pin needs to be configured as an output.
//     sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
//     sm.start();

//     let dma = pac.DMA.split(&mut pac.RESETS);

//     // Transfer a single message via DMA.
//     let tx_buf = singleton!(: [u32; 4] = message).unwrap();
//     let rx_buf = singleton!(: [u32; 4] = [0; 4]).unwrap();
//     let tx_transfer = single_buffer::Config::new(dma.ch0, tx_buf, tx).start();
//     let rx_transfer = single_buffer::Config::new(dma.ch1, rx, rx_buf).start();
//     let (ch0, tx_buf, tx) = tx_transfer.wait();
//     let (ch1, rx, rx_buf) = rx_transfer.wait();
//     for i in 0..rx_buf.len() {
//         if rx_buf[i] != tx_buf[i] {
//             // The data did not match, abort.
//             #[allow(clippy::empty_loop)]
//             loop {}
//         }
//     }

//     // Chain some buffers together for continuous transfers
//     let tx_buf2 = singleton!(: [u32; 4] = message).unwrap();
//     let rx_buf2 = singleton!(: [u32; 4] = [0; 4]).unwrap();
//     let tx_transfer = double_buffer::Config::new((ch0, ch1), tx_buf, tx).start();
//     let mut tx_transfer = tx_transfer.read_next(tx_buf2);
//     let rx_transfer = double_buffer::Config::new((dma.ch2, dma.ch3), rx, rx_buf).start();
//     let mut rx_transfer = rx_transfer.write_next(rx_buf2);
//     loop {
//         // When a transfer is done we immediately enqueue the buffers again.
//         if tx_transfer.is_done() {
//             let (tx_buf, next_tx_transfer) = tx_transfer.wait();
//             tx_transfer = next_tx_transfer.read_next(tx_buf);
//         }
//         if rx_transfer.is_done() {
//             let (rx_buf, next_rx_transfer) = rx_transfer.wait();
//             for i in 0..rx_buf.len() {
//                 if rx_buf[i] != message[i] {
//                     // The data did not match, abort.
//                     #[allow(clippy::empty_loop)]
//                     loop {}
//                 }
//             }
//             rx_transfer = next_rx_transfer.write_next(rx_buf);
//         }
//     }
// }

// use std::time::Duration;

// use cnc_brain::{runner::MachineState, Coordinates, Motion};
// use tokio::time::sleep;

// #[tokio::main]
// async fn main() {
//     let program = &vec![
//         Motion::Linear {
//             feedrate: 100.0,
//             coords: Coordinates {
//                 x: Some(150.0),
//                 y: Some(150.0),
//                 z: Some(0.0),
//             },
//         },
//         Motion::Linear {
//             feedrate: 100.0,
//             coords: Coordinates {
//                 x: Some(158.0),
//                 y: Some(253.0),
//                 z: Some(0.0),
//             },
//         },
//         Motion::Linear {
//             feedrate: 100.0,
//             coords: Coordinates {
//                 x: Some(200.0),
//                 y: Some(253.0),
//                 z: Some(0.0),
//             },
//         },
//         Motion::Linear {
//             feedrate: 100.0,
//             coords: Coordinates {
//                 x: Some(200.0),
//                 y: Some(260.0),
//                 z: Some(0.0),
//             },
//         },
//         Motion::Linear {
//             feedrate: 100.0,
//             coords: Coordinates {
//                 x: Some(200.0),
//                 y: Some(275.0),
//                 z: Some(0.0),
//             },
//         },
//         Motion::Linear {
//             feedrate: 100.0,
//             coords: Coordinates {
//                 x: Some(220.0),
//                 y: Some(280.0),
//                 z: Some(0.0),
//             },
//         },
//     ];

//     // println!("{:#?}", program);
//     // return;

//     let (tx, mut rx) = tokio::sync::mpsc::channel(1);

//     tx.send(Command::Movement(program.clone())).await.unwrap();

//     let mut machine = MachineState::default();
//     loop {
//         if let Ok(command) = rx.try_recv() {
//             match command {
//                 Command::Movement(movement) => {
//                     machine.queue_motion(movement);
//                 }
//             }
//         }

//         machine.tick().await;

//         sleep(Duration::from_millis(50)).await;
//     }
// }

// #[derive(Debug)]
// enum Command {
//     Movement(Vec<Motion>),
// }
