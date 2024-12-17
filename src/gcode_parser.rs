use regex::Regex;
use std::collections::HashSet;
use thiserror::Error;

use super::machine_state;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum LineMode {
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
pub enum ParserState {
    X(f32),
    Y(f32),
    Z(f32),
    L(u32),
    P(u32),
    SpindleSpeed(f32),
    FeedRate(f32),
    Mode(LineMode),
    Wcs(usize),
    Plane(machine_state::Plane),
    Units(machine_state::Units),
    DistanceMode(machine_state::DistanceMode),
    FeedRateMode(machine_state::FeedRateMode),
    Stop(machine_state::StopMode),
    SpindleMode(machine_state::SpindleMode),
    CoolantMode(machine_state::CoolantMode),
    OverrideMode(machine_state::OverrideMode),
    SelectTool(usize),
    ToolChange,
    ToolLengthMode(machine_state::ToolLengthMode),
}

impl ParserState {
    pub fn kind(self) -> &'static str {
        match self {
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
}

#[derive(Error, Debug)]
pub enum GCodeError {
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

pub struct GCodeParser {
    pub line_regex: Regex,
}

impl GCodeParser {
    pub fn new() -> Self {
        Self {
            line_regex: Regex::new(r"(([A-Z])((-?\d+)\.?(\d+)?))\b").unwrap(),
        }
    }

    pub fn parse_word(caps: regex::Captures) -> Result<(char, f32, u32, Option<u32>), GCodeError> {
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

    pub fn process_line(&mut self, line: &str) -> Result<Vec<ParserState>, GCodeError> {
        let mut state = Vec::new();
        let mut seen_variants = HashSet::new();

        let mut required_variants = Vec::new();

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
                        required_variants = vec!["L", "P"];
                        ParserState::Mode(LineMode::SetCoordinateSystem)
                    }
                    17 => ParserState::Plane(machine_state::Plane::XY),
                    18 => ParserState::Plane(machine_state::Plane::XZ),
                    19 => ParserState::Plane(machine_state::Plane::YZ),
                    20 => ParserState::Units(machine_state::Units::Inches),
                    21 => ParserState::Units(machine_state::Units::Millimeters),
                    28 => ParserState::Mode(LineMode::Home),
                    30 => ParserState::Mode(LineMode::Home2),
                    38 => ParserState::Mode(LineMode::Probe),
                    43 => match mantissa {
                        Some(1) => ParserState::ToolLengthMode(machine_state::ToolLengthMode::Set(
                            full_number,
                        )),
                        Some(2) => ParserState::ToolLengthMode(machine_state::ToolLengthMode::Add(
                            full_number,
                        )),
                        Some(3) => {
                            ParserState::ToolLengthMode(machine_state::ToolLengthMode::Cancel)
                        }
                        _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                    },
                    53 => ParserState::Mode(LineMode::MoveMachine),
                    54..=59 => ParserState::Wcs(number as usize - 54),
                    90 => ParserState::DistanceMode(machine_state::DistanceMode::Absolute),
                    91 => ParserState::DistanceMode(machine_state::DistanceMode::Incremental),
                    92 => match mantissa {
                        None => ParserState::Mode(LineMode::CoordinateSystemOffset),
                        Some(1) => ParserState::Mode(LineMode::DisableCoordinateSystemOffset),
                        Some(2) => {
                            ParserState::Mode(LineMode::DisableAndClearCoordinateSystemOffset)
                        }
                        Some(3) => ParserState::Mode(LineMode::RestoreCoordinateSystemOffset),
                        _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                    },
                    93 => ParserState::FeedRateMode(machine_state::FeedRateMode::InverseTime),
                    94 => ParserState::FeedRateMode(machine_state::FeedRateMode::UnitsPerMinute),
                    _ => return Err(GCodeError::UnsupportedNumber(letter, number)),
                },
                ('M', _) => match number {
                    0 => ParserState::Stop(machine_state::StopMode::ProgramStop),
                    1 => ParserState::Stop(machine_state::StopMode::OptionalStop),
                    2 => ParserState::Stop(machine_state::StopMode::ProgramEnd),
                    3 => ParserState::SpindleMode(machine_state::SpindleMode::Clockwise),
                    4 => ParserState::SpindleMode(machine_state::SpindleMode::CounterClockwise),
                    5 => ParserState::SpindleMode(machine_state::SpindleMode::Stop),
                    6 => ParserState::ToolChange,
                    7 => ParserState::CoolantMode(machine_state::CoolantMode::Mist),
                    8 => ParserState::CoolantMode(machine_state::CoolantMode::Flood),
                    9 => ParserState::CoolantMode(machine_state::CoolantMode::None),
                    48 => ParserState::OverrideMode(machine_state::OverrideMode::Enable),
                    49 => ParserState::OverrideMode(machine_state::OverrideMode::Disable),
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

            let kind = new_state.kind();
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
