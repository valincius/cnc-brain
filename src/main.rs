use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let mut port = MockSerialport::default();
    port.write("G0 X10 F30\n".as_bytes());
    port.write("G1 X20 Y15 F60\n".as_bytes());

    run_machine(port)?;

    Ok(())
}

fn run_machine(mut port: MockSerialport) -> Result<(), Box<dyn Error>> {
    let mut machine = MachineState::default();

    let buffer = port.read();
    let lines = String::from_utf8_lossy(&buffer);

    for line in lines.lines() {
        match machine.process_line(line) {
            Ok(_) => {
                machine.apply_target_state();
                println!("Processed: {}", line)
            },
            Err(errors) => {
                println!("Errors in line '{}':", line);
                for error in errors {
                    match error {
                        GCodeError::InvalidNumber(number) => println!("  Invalid number: '{}'", number),
                        GCodeError::UnsupportedWord(word) => println!("  Unsupported word: '{}'", word),
                        GCodeError::MissingLetter => println!("  Missing letter"),
                        GCodeError::MissingNumber => println!("  Missing number"),
                    }
                }
            }
        }
    }

    Ok(())
}

#[derive(Debug)]
enum GCodeError {
    InvalidNumber(String),        // Parsing error for the numeric part of the command
    UnsupportedWord(String),      // Unsupported command word
    MissingNumber,                // Command is missing a numeric value
    MissingLetter
}

struct MachineState {
    current_position: (f32, f32, f32), // X, Y, Z
    target_position: (f32, f32, f32),  // Target X, Y, Z
    current_feed_rate: f32,            // Current feed rate
    target_feed_rate: f32,             // Target feed rate
    current_mode: GCodeMode,           // Current motion mode (e.g., G0, G1)
    target_mode: GCodeMode,            // Target motion mode
}

impl Default for MachineState {
    fn default() -> Self {
        Self {
            current_position: (0.0, 0.0, 0.0),
            target_position: (0.0, 0.0, 0.0),
            current_feed_rate: 0.0,
            target_feed_rate: 0.0,
            current_mode: GCodeMode::Rapid, // Default to G0 (Rapid)
            target_mode: GCodeMode::Rapid,
        }
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum GCodeMode {
    Rapid,    // G0
    Linear,   // G1
}

impl MachineState {
    /// Processes a single G-code line and updates the target state
    fn process_line(&mut self, command: &str) -> Result<(), Vec<GCodeError>> {
        let mut errors = Vec::new();

        for word in command.split_whitespace() {
            // Ensure the word starts with a valid letter
            let first_char = word.chars().next();
            if let Some(first_char) = first_char {
                if !first_char.is_ascii_alphabetic() {
                    errors.push(GCodeError::MissingLetter);
                    continue;
                }
            } else {
                errors.push(GCodeError::MissingLetter);
                continue;
            }

            // Split the word into letter and numeric part
            let (letter, rest) = word.split_at(1);
            if rest.trim().is_empty() {
                errors.push(GCodeError::MissingNumber);
                continue;
            }

            let letter = letter.chars().next().unwrap();

            match rest.parse::<f32>() {
                Ok(number) => {
                    match (letter, number as i32) {
                        ('G', 0) => self.target_mode = GCodeMode::Rapid,
                        ('G', 1) => self.target_mode = GCodeMode::Linear,
                        ('G', 54..=69) => println!("Setting coordinate system to G{}", number),
                        ('X', _) => self.target_position.0 = number,
                        ('Y', _) => self.target_position.1 = number,
                        ('Z', _) => self.target_position.2 = number,
                        ('F', _) => self.target_feed_rate = number,
                        _ => errors.push(GCodeError::UnsupportedWord(word.to_string())),
                    }
                }
                Err(_) => errors.push(GCodeError::InvalidNumber(rest.to_string())),
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    /// Applies the target state to the current state
    fn apply_target_state(&mut self) {
        // Move the machine to the target position
        println!(
            "Moving from {:?} to {:?}",
            self.current_position, self.target_position
        );
        self.current_position = self.target_position;

        // Update the feed rate
        if self.current_feed_rate != self.target_feed_rate {
            println!(
                "Updating feed rate from {} to {}",
                self.current_feed_rate, self.target_feed_rate
            );
            self.current_feed_rate = self.target_feed_rate;
        }

        // Update the motion mode
        if self.current_mode != self.target_mode {
            println!(
                "Switching motion mode from {:?} to {:?}",
                self.current_mode, self.target_mode
            );
            self.current_mode = self.target_mode;
        }
    }
}

trait Serialport {
    fn read(&mut self) -> Vec<u8>;
    fn write(&mut self, data: &[u8]);
}

struct MockSerialport {
    buffer: Vec<u8>,
}

impl Default for MockSerialport {
    fn default() -> Self {
        Self {
            buffer: Vec::new(),
        }
    }
}

impl Serialport for MockSerialport {
    fn read(&mut self) -> Vec<u8> {
        let buffer = self.buffer.clone();
        self.buffer.clear();
        buffer
    }

    fn write(&mut self, data: &[u8]) {
        self.buffer.extend_from_slice(data);
    }
}
