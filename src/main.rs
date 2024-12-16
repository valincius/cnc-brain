use std::collections::HashMap;

use thiserror::Error;

use regex::Regex;

fn main() -> Result<(), anyhow::Error> {
    let mut port = MockSerialport::new();
    port.write("G0 X10 F30\n".as_bytes());
    port.write("G1 X20 Y15 F60\n".as_bytes());
    port.write("G1 X50 Y25 F60\n".as_bytes());
    port.write("G10 Y15 F60\n".as_bytes());
    port.write("G1 G28\n".as_bytes());
    port.write("G28 G1\n".as_bytes());

    run_machine(port)?;

    Ok(())
}

fn run_machine(mut port: MockSerialport) -> Result<(), anyhow::Error> {
    let mut parser = GCodeParser::new();

    let buffer = port.read();
    let lines = String::from_utf8_lossy(&buffer);

    let mut machine = MachineState::new();

    for line in lines.lines() {
        match parser.process_line(line) {
            Ok(state) => {
                machine.apply_state(state);
            }
            Err(err) => {
                println!("Errors in line '{}': {}", line, err);
            }
        }
    }

    println!("End state: {:?}", machine);

    Ok(())
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum MotionMode {
    Rapid,
    Linear,
}

#[derive(Debug, PartialEq, Copy, Clone)]
struct ParserState {
    x: Option<f32>,
    y: Option<f32>,
    z: Option<f32>,
    feed_rate: Option<f32>,
    mode: Option<MotionMode>,
    wcs: Option<u32>,
}

impl ParserState {
    fn new() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            feed_rate: None,
            mode: None,
            wcs: None,
        }
    }
}

#[derive(Debug)]
struct MachineState {
    feed_rate: f32,
    mode: MotionMode,
    active_wcs: usize,
    coordinates: Vec<(f32, f32, f32)>,
}

impl MachineState {
    fn new() -> Self {
        Self {
            feed_rate: 0.0,
            mode: MotionMode::Rapid,
            active_wcs: 0,
            coordinates: vec![(0.0, 0.0, 0.0); 10],
        }
    }

    fn apply_state(&mut self, new_state: ParserState) {
        if let Some(mode) = new_state.mode {
            self.mode = mode;
            println!("Motion mode changed: {:?}", new_state.mode);
        }

        if let Some(x) = new_state.x {
            self.coordinates[self.active_wcs].0 += x;
            println!("Moved X to {}", self.coordinates[self.active_wcs].0);
        }

        if let Some(y) = new_state.y {
            self.coordinates[self.active_wcs].1 += y;
            println!("Moved Y to {}", self.coordinates[self.active_wcs].1);
        }

        if let Some(z) = new_state.z {
            self.coordinates[self.active_wcs].2 += z;
            println!("Moved Z to {}", self.coordinates[self.active_wcs].2);
        }

        if let Some(feed_rate) = new_state.feed_rate {
            self.feed_rate = feed_rate;
            println!("Feed rate changed: {}", new_state.feed_rate.unwrap());
        }
    }
}

#[derive(Error, Debug)]
enum GCodeError {
    #[error("Unsupported letter ({0:?})")]
    UnsupportedLetter(char),
    #[error("Unsupported number ({0:?}{1:?})")]
    UnsupportedNumber(char, u32),
    #[error("Invalid letter ({0:?})")]
    InvalidLetter(String),
    #[error("Invalid number ({0:?})")]
    InvalidNumber(String),
    #[error("Invalid mantissa ({0:?})")]
    InvalidMantissa(String),
    #[error("Conflicting command encountered ({0:?} vs {1:?})")]
    ConflictingCommand(String, String),
}

struct GCodeParser {}

impl GCodeParser {
    fn new() -> Self {
        Self {}
    }

    fn parse_word(
        letter: &str,
        number: &str,
        mantissa: &str,
    ) -> Result<(char, u32, Option<f32>), GCodeError> {
        let letter_char = letter
            .chars()
            .next()
            .ok_or_else(|| GCodeError::InvalidLetter(letter.to_string()))?;

        let number_val = number
            .parse::<u32>()
            .map_err(|_| GCodeError::InvalidNumber(number.to_string()))?;

        let mantissa_val = {
            if mantissa.is_empty() {
                None
            } else {
                Some(
                    mantissa
                        .parse::<f32>()
                        .map_err(|_| GCodeError::InvalidMantissa(mantissa.to_string()))?,
                )
            }
        };

        Ok((letter_char, number_val, mantissa_val))
    }

    fn process_line(&mut self, line: &str) -> Result<ParserState, GCodeError> {
        let re = Regex::new(r"(([A-Z])(-?\d+)(\.\d+)?)\b").unwrap();

        let mut new_state = ParserState::new();

        let modal_groups = vec![
            vec!["G4", "G10", "G28", "G30", "G53", "G92", "G92"],
            vec![
                "G0", "G1", "G2", "G3", "G38", "G80", "G81", "G82", "G83", "G84", "G85", "G86",
                "G87", "G88", "G89",
            ],
            vec!["G17", "G18", "G19"],
            vec!["G90", "G91"],
            vec!["M0", "M1", "M2", "M30", "M60"],
            vec!["G93", "G94"],
            vec!["M6"],
            vec!["M3", "M4", "M5"],
            vec!["M7", "M8", "M9"],
            vec!["M48", "M49"],
            vec!["G98", "G99"],
            vec!["G54", "G55", "G56", "G57", "G58", "G59"],
            vec!["G61", "G64"],
        ];

        let find_group = |letter: char, number: u32| -> Option<usize> {
            for (i, group) in modal_groups.iter().enumerate() {
                if group.contains(&format!("{}{}", letter, number).as_str()) {
                    return Some(i);
                }
            }

            None
        };

        let mut consumed_groups = HashMap::new();

        for caps in re.captures_iter(line) {
            let _raw_word = caps.get(1).map(|m| m.as_str()).unwrap_or("");
            let raw_letter = caps.get(2).map(|m| m.as_str()).unwrap_or("");
            let raw_number = caps.get(3).map(|m| m.as_str()).unwrap_or("");
            let raw_mantissa = caps.get(4).map(|m| m.as_str()).unwrap_or("");

            let (letter, number, mantissa) =
                Self::parse_word(raw_letter, raw_number, raw_mantissa)?;

            if let Some(group) = find_group(letter, number) {
                if let Some(conflicting_number) = consumed_groups.get(&group) {
                    return Err(GCodeError::ConflictingCommand(
                        format!("{}{}", letter, conflicting_number),
                        format!("{}{}", letter, number),
                    ));
                }

                if group == 0 {
                    if let Some(g1) = consumed_groups.get(&1) {
                        return Err(GCodeError::ConflictingCommand(
                            format!("{}{}", letter, g1),
                            format!("{}{}", letter, number),
                        ));
                    }
                } else if group == 1 {
                    if let Some(g0) = consumed_groups.get(&0) {
                        return Err(GCodeError::ConflictingCommand(
                            format!("{}{}", letter, g0),
                            format!("{}{}", letter, number),
                        ));
                    }
                }

                consumed_groups.insert(group, number);
            }

            let full_number = number as f32 + mantissa.unwrap_or(0.0);

            match (letter, number) {
                ('G', _) => match number {
                    0 => {
                        new_state.mode = Some(MotionMode::Rapid);
                    }
                    1 => {
                        new_state.mode = Some(MotionMode::Linear);
                    }
                    10 | 28 | 30 => {}
                    54..=59 => {
                        new_state.wcs = Some(number - 54);
                    }
                    _ => return Err(GCodeError::UnsupportedNumber(letter, number)),
                },
                ('A' | 'B' | 'C' | 'X' | 'Y' | 'Z', _) => match letter {
                    'X' => new_state.x = Some(full_number),
                    'Y' => new_state.y = Some(full_number),
                    'Z' => new_state.z = Some(full_number),
                    _ => return Err(GCodeError::UnsupportedLetter(letter)),
                },
                ('F', _) => {
                    new_state.feed_rate = Some(full_number);
                }
                _ => return Err(GCodeError::UnsupportedLetter(letter)),
            }
        }

        Ok(new_state)
    }
}

trait Serialport {
    fn read(&mut self) -> Vec<u8>;
    fn write(&mut self, data: &[u8]);
}

struct MockSerialport {
    buffer: Vec<u8>,
}

impl MockSerialport {
    fn new() -> Self {
        Self { buffer: Vec::new() }
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
