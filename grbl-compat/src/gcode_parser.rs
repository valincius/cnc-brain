use regex::Regex;
use thiserror::Error;

use crate::grbl_state::{
    self, CircularDirection, CoordinateSystemOffsetMode, HomeMode, OptionalAxes, OptionalOffsets,
    ProbeMode,
};

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Function {
    MoveRapid(OptionalAxes),  //G0
    MoveLinear(OptionalAxes), //G1
    MoveCircular(
        CircularDirection,
        OptionalAxes,
        OptionalOffsets,
        Option<f32>,
    ), //G2
    Probe(ProbeMode, OptionalAxes), //G38
    Dwell(f32),               //G4
    SetCoordinateSystem(u32, u32), //G10 L P
    Home(HomeMode),           //G28
    Home2(HomeMode),          //G30
    MoveMachine(OptionalAxes), //G53
    CoordinateSystemOffset(CoordinateSystemOffsetMode), //G92
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Word {
    A(f32),              // Axis
    B(f32),              // Axis
    C(f32),              // Axis
    D(f32),              // Tool radius offset
    F(f32),              // Feed rate
    G(u32, Option<u32>), // General function
    H(u32),              // Tool length offset index
    I(f32),              // X-axis arc offset
    J(f32),              // Y-axis arc offset
    K(f32),              // Z-axis arc offset
    L(u32),              // G10 key
    M(u32),              // Miscellaneous function
    N(u32),              // Line number
    P(f32),              // Dwell time or G10 key
    R(f32),              // Arc radius
    S(f32),              // Spindle speed
    T(u32),              // Tool selection
    X(f32),              // X-axis
    Y(f32),              // Y-axis
    Z(f32),              // Z-axis
}

#[macro_export]
macro_rules! extract_word {
    ($state:expr, $variant:path) => {
        $state.iter().find_map(|s| {
            if let $variant(inner) = s {
                Some(*inner)
            } else {
                None
            }
        })
    };
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Action {
    Function(Function),
    SetSpindleSpeed(f32),
    SetFeedRate(f32),
    SetWCS(usize),
    SetPlane(grbl_state::Plane),
    SetUnits(grbl_state::Units),
    SetDistanceMode(grbl_state::DistanceMode),
    SetFeedRateMode(grbl_state::FeedRateMode),
    SetSpindleMode(grbl_state::SpindleMode),
    SetCoolantMode(grbl_state::CoolantMode),
    SetOverrideMode(grbl_state::OverrideMode),
    SetSelectedTool(usize),
    SetToolLengthMode(grbl_state::ToolLengthMode),
    Stop(grbl_state::StopMode),
    ToolChange,
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
    #[error("Unsupported word ({0:?})")]
    UnsupportedWord(Word),
    #[error("Invalid pattern ({0:?})")]
    InvalidPattern(String),
}

macro_rules! select_match {
    ($value:expr, $($pattern:pat => $result:expr),+ $(,)?) => {
        match $value {
            $($pattern => $result,)+
            _ => return Err(GCodeError::InvalidPattern($value.to_string())),
        }
    };
}

pub struct GCodeParser {
    pub line_regex: Regex,
}

impl Default for GCodeParser {
    fn default() -> Self {
        Self {
            line_regex: Regex::new(r"(([A-Z])((-?\d+)\.?(\d+)?))\b").unwrap(),
        }
    }
}

impl GCodeParser {
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

    pub fn parse_line(&self, line: &str) -> Result<Vec<Word>, GCodeError> {
        let mut words = Vec::new();

        for caps in self.line_regex.captures_iter(line) {
            let (letter, full_number, number, mantissa) = Self::parse_word(caps)?;

            match letter {
                'A' => words.push(Word::A(full_number)),
                'B' => words.push(Word::B(full_number)),
                'C' => words.push(Word::C(full_number)),
                'D' => words.push(Word::D(full_number)),
                'F' => words.push(Word::F(full_number)),
                'G' => words.push(Word::G(number, mantissa)),
                'H' => words.push(Word::H(number)),
                'I' => words.push(Word::I(full_number)),
                'J' => words.push(Word::J(full_number)),
                'K' => words.push(Word::K(full_number)),
                'L' => words.push(Word::L(number)),
                'M' => words.push(Word::M(number)),
                'N' => words.push(Word::N(number)),
                'P' => words.push(Word::P(full_number)),
                'S' => words.push(Word::S(full_number)),
                'T' => words.push(Word::T(number)),
                'X' => words.push(Word::X(full_number)),
                'Y' => words.push(Word::Y(full_number)),
                'Z' => words.push(Word::Z(full_number)),
                _ => return Err(GCodeError::UnsupportedLetter(letter)),
            }
        }

        Ok(words)
    }

    pub fn build_command(&mut self, words: Vec<Word>) -> Result<Vec<Action>, GCodeError> {
        let mut commands = Vec::new();

        let optional_axes = OptionalAxes::from_words(&words);

        let g_words = words.iter().filter_map(|w| match w {
            Word::G(number, mantissa) => Some((*number, *mantissa)),
            _ => None,
        });

        let m_words = words.iter().filter_map(|w| match w {
            Word::M(number) => Some(*number),
            _ => None,
        });

        for (number, mantissa) in g_words {
            let command = match number {
                0 => Action::Function(Function::MoveRapid(optional_axes)),
                1 => Action::Function(Function::MoveLinear(optional_axes)),
                2 | 3 => Action::Function(Function::MoveCircular(
                    select_match!(
                        number,
                        2 => CircularDirection::Clockwise,
                        3 => CircularDirection::CounterClockwise
                    ),
                    optional_axes,
                    OptionalOffsets::from_words(&words),
                    extract_word!(words, Word::R),
                )),
                4 => Action::Function(Function::Dwell(extract_word!(words, Word::P).unwrap())),
                10 => Action::Function(Function::SetCoordinateSystem(
                    extract_word!(words, Word::L).unwrap(),
                    extract_word!(words, Word::P).unwrap() as u32,
                )),
                17 => Action::SetPlane(grbl_state::Plane::XY),
                18 => Action::SetPlane(grbl_state::Plane::XZ),
                19 => Action::SetPlane(grbl_state::Plane::YZ),
                20 => Action::SetUnits(grbl_state::Units::Inches),
                21 => Action::SetUnits(grbl_state::Units::Millimeters),
                28 => Action::Function(Function::Home(match mantissa {
                    None => HomeMode::Go,
                    Some(1) => HomeMode::Set(optional_axes),
                    _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                })),
                30 => Action::Function(Function::Home2(match mantissa {
                    None => HomeMode::Go,
                    Some(1) => HomeMode::Set(optional_axes),
                    _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                })),
                38 => Action::Function(Function::Probe(
                    match mantissa {
                        Some(2) => ProbeMode::TowardWorkpieceErroring,
                        Some(3) => ProbeMode::TowardWorkpieceNonErroring,
                        Some(4) => ProbeMode::AwayFromWorkpieceErroring,
                        Some(5) => ProbeMode::AwayFromWorkpieceNonErroring,
                        _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                    },
                    optional_axes,
                )),
                43 => match mantissa {
                    Some(1) => {
                        Action::SetToolLengthMode(grbl_state::ToolLengthMode::Set(optional_axes))
                    }
                    Some(2) => {
                        Action::SetToolLengthMode(grbl_state::ToolLengthMode::Add(optional_axes))
                    }
                    Some(3) => Action::SetToolLengthMode(grbl_state::ToolLengthMode::Cancel),
                    _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                },
                53 => Action::Function(Function::MoveMachine(optional_axes)),
                54..=59 => Action::SetWCS(number as usize - 54),
                90 => Action::SetDistanceMode(grbl_state::DistanceMode::Absolute),
                91 => Action::SetDistanceMode(grbl_state::DistanceMode::Incremental),
                92 => Action::Function(Function::CoordinateSystemOffset(match mantissa {
                    None => CoordinateSystemOffsetMode::Set(OptionalOffsets::from_words(&words)),
                    Some(1) => CoordinateSystemOffsetMode::DisableAndZero,
                    Some(2) => CoordinateSystemOffsetMode::Disable,
                    Some(3) => CoordinateSystemOffsetMode::Restore,
                    _ => return Err(GCodeError::UnsupportedMantissa(mantissa)),
                })),
                93 => Action::SetFeedRateMode(grbl_state::FeedRateMode::InverseTime),
                94 => Action::SetFeedRateMode(grbl_state::FeedRateMode::UnitsPerMinute),
                _ => return Err(GCodeError::UnsupportedNumber('G', number)),
            };
            commands.push(command);
        }

        for number in m_words {
            let command = match number {
                0 => Action::Stop(grbl_state::StopMode::ProgramStop),
                1 => Action::Stop(grbl_state::StopMode::OptionalStop),
                2 => Action::Stop(grbl_state::StopMode::ProgramEnd),
                3 | 4 => Action::SetSpindleMode(grbl_state::SpindleMode::Direction(select_match!(
                    number,
                    3 => CircularDirection::Clockwise,
                    4 => CircularDirection::CounterClockwise
                ))),
                5 => Action::SetSpindleMode(grbl_state::SpindleMode::Stop),
                6 => Action::ToolChange,
                7 => Action::SetCoolantMode(grbl_state::CoolantMode::Mist),
                8 => Action::SetCoolantMode(grbl_state::CoolantMode::Flood),
                9 => Action::SetCoolantMode(grbl_state::CoolantMode::None),
                48 => Action::SetOverrideMode(grbl_state::OverrideMode::Enable),
                49 => Action::SetOverrideMode(grbl_state::OverrideMode::Disable),
                _ => return Err(GCodeError::UnsupportedNumber('M', number)),
            };

            commands.push(command);
        }

        Ok(commands)
    }
}
