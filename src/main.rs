use std::collections::HashSet;

use thiserror::Error;

use regex::Regex;

fn main() -> Result<(), anyhow::Error> {
    let mut port = MockSerialport::new();
    port.write("G0 X10 F30\n".as_bytes());
    port.write("G1 X20 Y15 F60\n".as_bytes());
    port.write("G1 X50 Y25 F60\n".as_bytes());
    port.write("G10 L2 P1 Y15 F60\n".as_bytes());
    port.write("G55 Y99 X99\n".as_bytes());
    port.write("G0 G1 X99\n".as_bytes());

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
enum LineMode {
    MoveRapid,                             //G0
    MoveLinear,                            //G1
    MoveCircularCW,                        //G2
    MoveCircularCCW,                       //G3
    Probe,                                 //G38
    Dwell,                                 //G4
    SetCoordinateSystem,                   //G10
    Home,                                  //G28
    Home2,                                 //G30
    MoveMachine,                           //G53
    CoordinateSystemOffset,                //G92
    DisableCoordinateSystemOffset,         //G92.1
    DisableAndClearCoordinateSystemOffset, //G92.2
    RestoreCoordinateSystemOffset,         //G92.3
                                           // No canned cycles - G8x
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum Plane {
    XY, //G17
    XZ, //G18
    YZ, //G19
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum Units {
    Inches,
    Millimeters,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum DistanceMode {
    Absolute,
    Incremental,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum StopMode {
    ProgramStop,
    OptionalStop,
    ProgramEnd,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum FeedRateMode {
    InverseTime,
    UnitsPerMinute,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum SpindleMode {
    Clockwise,
    CounterClockwise,
    Stop,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum CoolantMode {
    Mist,
    Flood,
    None,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum OverrideMode {
    Enable,
    Disable,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum ToolLengthMode {
    Set(f32),
    Add(f32),
    Cancel,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum ParserState {
    X(f32),
    Y(f32),
    Z(f32),
    L(u32),
    P(u32),
    SpindleSpeed(f32),
    FeedRate(f32),
    Mode(LineMode),
    Wcs(usize),
    Plane(Plane),
    Units(Units),
    DistanceMode(DistanceMode),
    FeedRateMode(FeedRateMode),
    Stop(StopMode),
    SpindleMode(SpindleMode),
    CoolantMode(CoolantMode),
    OverrideMode(OverrideMode),
    SelectTool(usize),
    ToolChange,
    ToolLengthMode(ToolLengthMode),
}

#[derive(Debug)]
struct MachineState {
    position: (f32, f32, f32),
    feed_rate: f32,
    spindle_speed: f32,
    active_wcs: usize,
    wcs_offsets: Vec<(f32, f32, f32)>,
    plane: Plane,
    units: Units,
    distance_mode: DistanceMode,
    feed_rate_mode: FeedRateMode,
    spindle_mode: SpindleMode,
    coolant_mode: CoolantMode,
    override_mode: OverrideMode,
    tool: usize,
}

impl MachineState {
    fn new() -> Self {
        Self {
            position: (0.0, 0.0, 0.0),
            feed_rate: 0.0,
            spindle_speed: 0.0,
            tool: 0,
            active_wcs: 0,
            wcs_offsets: vec![(0.0, 0.0, 0.0); 10],
            plane: Plane::XY,
            units: Units::Inches,
            distance_mode: DistanceMode::Absolute,
            feed_rate_mode: FeedRateMode::UnitsPerMinute,
            spindle_mode: SpindleMode::Stop,
            coolant_mode: CoolantMode::None,
            override_mode: OverrideMode::Disable,
        }
    }

    fn apply_state(&mut self, state: Vec<ParserState>) {
        let mut sorted_state = state;

        sorted_state.sort_by_key(|s| match s {
            ParserState::Mode(_) => 0,
            ParserState::Wcs(_) => 1,
            ParserState::FeedRate(_) => 2,
            _ => 3,
        });

        let word_x = sorted_state
            .iter()
            .find(|s| matches!(s, ParserState::X(_)))
            .map(|s| match s {
                ParserState::X(x) => *x,
                _ => 0.0,
            });

        let word_y = sorted_state
            .iter()
            .find(|s| matches!(s, ParserState::Y(_)))
            .map(|s| match s {
                ParserState::Y(y) => *y,
                _ => 0.0,
            });

        let word_z = sorted_state
            .iter()
            .find(|s| matches!(s, ParserState::Z(_)))
            .map(|s| match s {
                ParserState::Z(z) => *z,
                _ => 0.0,
            });

        let word_l = sorted_state
            .iter()
            .find(|s| matches!(s, ParserState::L(_)))
            .map(|s| match s {
                ParserState::L(l) => *l,
                _ => 0,
            });

        let word_p = sorted_state
            .iter()
            .find(|s| matches!(s, ParserState::P(_)))
            .map(|s| match s {
                ParserState::P(p) => *p,
                _ => 0,
            });

        for s in sorted_state {
            match s {
                ParserState::FeedRate(f) => {
                    self.feed_rate = f;
                }
                ParserState::SpindleSpeed(s) => {
                    self.spindle_speed = s;
                }
                ParserState::Wcs(wcs) => {
                    self.active_wcs = wcs;
                }
                ParserState::Plane(p) => {
                    self.plane = p;
                }
                ParserState::Units(u) => {
                    self.units = u;
                }
                ParserState::DistanceMode(m) => {
                    self.distance_mode = m;
                }
                ParserState::FeedRateMode(m) => {
                    self.feed_rate_mode = m;
                }
                ParserState::Stop(m) => match m {
                    StopMode::ProgramStop => {
                        todo!("Program stop");
                    }
                    StopMode::OptionalStop => {
                        todo!("Optional stop");
                    }
                    StopMode::ProgramEnd => {
                        todo!("Program end");
                    }
                },
                ParserState::SpindleMode(m) => {
                    self.spindle_mode = m;
                }
                ParserState::CoolantMode(m) => {
                    self.coolant_mode = m;
                }
                ParserState::OverrideMode(m) => {
                    self.override_mode = m;
                }
                ParserState::SelectTool(t) => {
                    self.tool = t;
                }
                ParserState::ToolChange => {
                    todo!("Tool change");
                }
                ParserState::Mode(mode) => match mode {
                    LineMode::SetCoordinateSystem => match word_l {
                        Some(2) => {
                            if let Some(x) = word_x {
                                self.wcs_offsets[self.active_wcs].0 = x;
                            }
                            if let Some(y) = word_y {
                                self.wcs_offsets[self.active_wcs].1 = y;
                            }
                            if let Some(z) = word_z {
                                self.wcs_offsets[self.active_wcs].2 = z;
                            }
                        }
                        Some(20) => {
                            // G10 L20 is similar to G10 L2 except that instead of setting the offset/entry to the given value, it is set to a calculated value that makes the current coordinates become the given value.
                            todo!("G10 L20");
                        }
                        _ => {
                            todo!("Unsupported coordinate system");
                        }
                    },
                    LineMode::CoordinateSystemOffset => {
                        todo!("Coordinate system offset");
                    }
                    _ => {}
                },
                _ => {}
            }
        }
    }
}

#[derive(Error, Debug)]
enum GCodeError {
    #[error("Unsupported letter ({0:?})")]
    UnsupportedLetter(char),
    #[error("Unsupported number ({0:?}{1:?})")]
    UnsupportedNumber(char, u32),
    #[error("Unsupported mantissa ({0:?})")]
    UnsupportedMantissa(Option<u32>),
    #[error("Invalid letter ({0:?})")]
    InvalidLetter(String),
    #[error("Invalid number ({0:?})")]
    InvalidNumber(String),
    #[error("Invalid mantissa ({0:?})")]
    InvalidMantissa(String),
    #[error("Conflicting command encountered ({0:?} is already set to {1:?})")]
    ConflictingCommand(String, String),
    #[error("Missing required word ({0:?})")]
    MissingRequiredWord(String),
}

struct GCodeParser {
    line_regex: Regex,
}

impl GCodeParser {
    fn new() -> Self {
        Self {
            line_regex: Regex::new(r"(([A-Z])((-?\d+)\.?(\d+)?))\b").unwrap(),
        }
    }

    fn parse_word(caps: regex::Captures) -> Result<(char, f32, u32, Option<u32>), GCodeError> {
        let letter = caps.get(2).unwrap().as_str();
        let number = caps.get(3).unwrap().as_str();
        let whole_number = caps.get(4).unwrap().as_str();
        let fractional_number = caps.get(5).map(|m| m.as_str()).unwrap_or("");

        let letter_char = letter
            .chars()
            .next()
            .ok_or_else(|| GCodeError::InvalidLetter(letter.to_string()))?;

        let number_val = number
            .parse::<f32>()
            .map_err(|_| GCodeError::InvalidNumber(number.to_string()))?;

        let whole_number_val = whole_number
            .parse::<u32>()
            .map_err(|_| GCodeError::InvalidNumber(whole_number.to_string()))?;

        let mantissa_val = {
            if fractional_number.is_empty() {
                None
            } else {
                Some(
                    fractional_number
                        .parse::<u32>()
                        .map_err(|_| GCodeError::InvalidMantissa(fractional_number.to_string()))?,
                )
            }
        };

        Ok((letter_char, number_val, whole_number_val, mantissa_val))
    }

    fn variant_kind(state: &ParserState) -> &'static str {
        match state {
            ParserState::Mode(_) => "Mode",
            ParserState::Wcs(_) => "Wcs",
            ParserState::FeedRate(_) => "FeedRate",
            ParserState::X(_) => "X",
            ParserState::Y(_) => "Y",
            ParserState::Z(_) => "Z",
            ParserState::Plane(_) => "Plane",
            ParserState::Units(_) => "Units",
            ParserState::DistanceMode(_) => "DistanceMode",
            ParserState::FeedRateMode(_) => "FeedRateMode",
            ParserState::Stop(_) => "Stop",
            ParserState::SpindleMode(_) => "SpindleMode",
            ParserState::CoolantMode(_) => "CoolantMode",
            ParserState::OverrideMode(_) => "OverrideMode",
            ParserState::SelectTool(_) => "SelectTool",
            ParserState::ToolChange => "ToolChange",
            ParserState::SpindleSpeed(_) => "SpindleSpeed",
            ParserState::L(_) => "L",
            ParserState::P(_) => "P",
            ParserState::ToolLengthMode(_) => "ToolLengthMode",
        }
    }

    fn process_line(&mut self, line: &str) -> Result<Vec<ParserState>, GCodeError> {
        let mut state = Vec::new();
        let mut seen_variants = HashSet::new();

        let mut required_variants = HashSet::new();

        for caps in self.line_regex.captures_iter(line) {
            let (letter, full_number, number, mantissa) = Self::parse_word(caps)?;

            let new_state = match (letter, number) {
                ('G', _) => match number {
                    0 => ParserState::Mode(LineMode::MoveRapid),
                    1 => ParserState::Mode(LineMode::MoveLinear),
                    2 => ParserState::Mode(LineMode::MoveCircularCW),
                    3 => ParserState::Mode(LineMode::MoveCircularCCW),
                    4 => ParserState::Mode(LineMode::Dwell),
                    10 => {
                        required_variants.insert(Self::variant_kind(&ParserState::L(0)));
                        required_variants.insert(Self::variant_kind(&ParserState::P(0)));
                        ParserState::Mode(LineMode::SetCoordinateSystem)
                    }
                    17 => ParserState::Plane(Plane::XY),
                    18 => ParserState::Plane(Plane::XZ),
                    19 => ParserState::Plane(Plane::YZ),
                    20 => ParserState::Units(Units::Inches),
                    21 => ParserState::Units(Units::Millimeters),
                    28 => ParserState::Mode(LineMode::Home),
                    30 => ParserState::Mode(LineMode::Home2),
                    38 => ParserState::Mode(LineMode::Probe),
                    43 => match mantissa {
                        Some(1) => ParserState::ToolLengthMode(ToolLengthMode::Set(full_number)),
                        Some(2) => ParserState::ToolLengthMode(ToolLengthMode::Add(full_number)),
                        Some(3) => ParserState::ToolLengthMode(ToolLengthMode::Cancel),
                        _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                    },
                    53 => ParserState::Mode(LineMode::MoveMachine),
                    54..=59 => ParserState::Wcs(number as usize - 54),
                    90 => ParserState::DistanceMode(DistanceMode::Absolute),
                    91 => ParserState::DistanceMode(DistanceMode::Incremental),
                    92 => match mantissa {
                        None => ParserState::Mode(LineMode::CoordinateSystemOffset),
                        Some(1) => ParserState::Mode(LineMode::DisableCoordinateSystemOffset),
                        Some(2) => {
                            ParserState::Mode(LineMode::DisableAndClearCoordinateSystemOffset)
                        }
                        Some(3) => ParserState::Mode(LineMode::RestoreCoordinateSystemOffset),
                        _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                    },
                    93 => ParserState::FeedRateMode(FeedRateMode::InverseTime),
                    94 => ParserState::FeedRateMode(FeedRateMode::UnitsPerMinute),
                    _ => return Err(GCodeError::UnsupportedNumber(letter, number)),
                },
                ('M', _) => match number {
                    0 => ParserState::Stop(StopMode::ProgramStop),
                    1 => ParserState::Stop(StopMode::OptionalStop),
                    2 => ParserState::Stop(StopMode::ProgramEnd),
                    3 => ParserState::SpindleMode(SpindleMode::Clockwise),
                    4 => ParserState::SpindleMode(SpindleMode::CounterClockwise),
                    5 => ParserState::SpindleMode(SpindleMode::Stop),
                    6 => ParserState::ToolChange,
                    7 => ParserState::CoolantMode(CoolantMode::Mist),
                    8 => ParserState::CoolantMode(CoolantMode::Flood),
                    9 => ParserState::CoolantMode(CoolantMode::None),
                    48 => ParserState::OverrideMode(OverrideMode::Enable),
                    49 => ParserState::OverrideMode(OverrideMode::Disable),
                    _ => return Err(GCodeError::UnsupportedNumber(letter, number)),
                },
                ('X', _) => ParserState::X(full_number),
                ('Y', _) => ParserState::Y(full_number),
                ('Z', _) => ParserState::Z(full_number),
                ('F', _) => ParserState::FeedRate(full_number),
                ('S', _) => ParserState::SpindleSpeed(full_number),
                ('T', _) => ParserState::SelectTool(number as usize),
                ('L', _) => ParserState::L(number),
                ('P', _) => ParserState::P(number),
                _ => return Err(GCodeError::UnsupportedLetter(letter)),
            };

            let kind = Self::variant_kind(&new_state);
            if seen_variants.contains(kind) {
                return Err(GCodeError::ConflictingCommand(
                    kind.to_string(),
                    format!("{:?}", new_state),
                ));
            } else {
                seen_variants.insert(kind);
            }

            state.push(new_state);
        }

        for required in required_variants.iter() {
            if !seen_variants.contains(required) {
                return Err(GCodeError::MissingRequiredWord(required.to_string()));
            }
        }

        Ok(state)
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
