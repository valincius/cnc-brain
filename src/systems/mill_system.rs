use systems::motion_system::MotionSystem;

use crate::prelude::*;

pub struct MillSystem<X: PositionalMotor, Y: PositionalMotor, Z: PositionalMotor, S: Motor> {
    pub motion_system: MotionSystem<X, Y, Z>,
    pub spindle_motor: S,

    reciever: CommandReceiver,
}

impl<X: PositionalMotor, Y: PositionalMotor, Z: PositionalMotor, S: Motor> MillSystem<X, Y, Z, S> {
    pub fn new(
        reciever: CommandReceiver,
        motion_system: MotionSystem<X, Y, Z>,
        spindle_motor: S,
    ) -> MillSystem<X, Y, Z, S> {
        MillSystem {
            reciever,
            motion_system,
            spindle_motor,
        }
    }

    pub async fn run(&mut self) {
        loop {
            self.motion_system.tick().await;

            if let Ok(command) = self.reciever.try_receive() {
                self.process_command(command).await;
            }
        }
    }

    pub async fn process_command(&mut self, command: SystemCommand) {
        match command {
            SystemCommand::MoveTo(motion) => {
                self.motion_system.queue(motion).await;
            }
        }
    }
}
