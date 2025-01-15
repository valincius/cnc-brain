use std::time::Duration;

use cnc_brain::{runner::MachineState, Coordinates, Movement};
use tokio::time::sleep;

#[tokio::main]
async fn main() {
    let program = &vec![
        Movement::Linear {
            feedrate: 100.0,
            coords: Coordinates {
                x: Some(150.0),
                y: Some(150.0),
                z: Some(0.0),
            },
        },
        Movement::Linear {
            feedrate: 100.0,
            coords: Coordinates {
                x: Some(175.0),
                y: Some(250.0),
                z: Some(0.0),
            },
        },
    ];

    // println!("{:#?}", program);
    // return;

    let (tx, mut rx) = tokio::sync::mpsc::channel(1);

    tx.send(Command::Movement(program.clone())).await.unwrap();

    let mut machine = MachineState::default();
    loop {
        if let Ok(command) = rx.try_recv() {
            match command {
                Command::Movement(movement) => {
                    machine.queue_motion(movement);
                }
            }
        }

        machine.tick().await;

        sleep(Duration::from_millis(50)).await;
    }
}

#[derive(Debug)]
enum Command {
    Movement(Vec<Movement>),
}
