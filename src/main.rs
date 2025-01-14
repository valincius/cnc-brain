use std::time::Duration;

use cnc_brain::runner::{Instruction, MachineState, MotionInputs, MotionPlanner, MotionSegment};
use tokio::time::sleep;

#[tokio::main]
async fn main() {
    let program = MotionPlanner::from_program(&vec![
        Instruction::Linear(
            50.0,
            MotionInputs {
                x: Some(100.0),
                y: Some(100.0),
                z: Some(0.0),
            },
        ),
        Instruction::Linear(
            100.0,
            MotionInputs {
                x: Some(100.0),
                y: Some(200.0),
                z: Some(0.0),
            },
        ),
        Instruction::Linear(
            20.0,
            MotionInputs {
                x: Some(0.0),
                y: Some(200.0),
                z: Some(0.0),
            },
        ),
    ]);

    return;

    let (tx, mut rx) = tokio::sync::mpsc::channel(1);

    tx.send(Command::PushMotions(program)).await.unwrap();

    let mut machine = MachineState::default();
    loop {
        if let Ok(command) = rx.try_recv() {
            match command {
                Command::PushMotions(motions) => {
                    machine.push_motions(motions);
                }
            }
        }

        machine.step().await;

        sleep(Duration::from_millis(1)).await;
    }
}

#[derive(Debug)]
enum Command {
    PushMotions(Vec<MotionSegment>),
}
