use crate::{
    extract_word,
    gcode_parser::{Action, Word},
};

use super::gcode_parser::Function;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Plane {
    XY, //G17
    XZ, //G18
    YZ, //G19
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Units {
    Inches,
    Millimeters,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum DistanceMode {
    Absolute,
    Incremental,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum StopMode {
    ProgramStop,
    OptionalStop,
    ProgramEnd,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum FeedRateMode {
    InverseTime,
    UnitsPerMinute,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum CircularDirection {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum SpindleMode {
    Direction(CircularDirection),
    Stop,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum CoolantMode {
    Mist,
    Flood,
    None,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum OverrideMode {
    Enable,
    Disable,
}

#[derive(Debug, PartialEq, Copy, Clone, Default)]
pub struct OptionalAxes {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,

    pub a: Option<f32>,
    pub b: Option<f32>,
    pub c: Option<f32>,
}

impl OptionalAxes {
    pub fn from_words(words: &[Word]) -> Self {
        Self {
            x: extract_word!(words, Word::X),
            y: extract_word!(words, Word::Y),
            z: extract_word!(words, Word::Z),
            a: extract_word!(words, Word::A),
            b: extract_word!(words, Word::B),
            c: extract_word!(words, Word::C),
        }
    }
}

#[derive(Debug, PartialEq, Copy, Clone, Default)]
pub struct OptionalOffsets {
    pub i: Option<f32>,
    pub j: Option<f32>,
    pub k: Option<f32>,
}

impl OptionalOffsets {
    pub fn from_words(words: &[Word]) -> Self {
        Self {
            i: extract_word!(words, Word::I),
            j: extract_word!(words, Word::J),
            k: extract_word!(words, Word::K),
        }
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct Axes {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl Default for Axes {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            a: 0.0,
            b: 0.0,
            c: 0.0,
        }
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum ToolLengthMode {
    Set(OptionalAxes),
    Add(OptionalAxes),
    Cancel,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum ProbeMode {
    TowardWorkpieceErroring,
    TowardWorkpieceNonErroring,
    AwayFromWorkpieceErroring,
    AwayFromWorkpieceNonErroring,
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum HomeMode {
    Go,
    Set(OptionalAxes),
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum CoordinateSystemOffsetMode {
    Set(OptionalOffsets),
    DisableAndZero,
    Disable,
    Restore,
}

#[derive(Debug)]
pub struct GrblState {
    pub position: Axes,
    pub feed_rate: f32,
    pub spindle_speed: f32,
    pub active_wcs: usize,
    pub wcs_offsets: Vec<Axes>,
    pub plane: Plane,
    pub units: Units,
    pub distance_mode: DistanceMode,
    pub feed_rate_mode: FeedRateMode,
    pub spindle_mode: SpindleMode,
    pub coolant_mode: CoolantMode,
    pub override_mode: OverrideMode,
    pub tool: usize,
}

impl Default for GrblState {
    fn default() -> Self {
        Self {
            position: Axes::default(),
            feed_rate: 0.0,
            spindle_speed: 0.0,
            tool: 0,
            active_wcs: 0,
            wcs_offsets: vec![Axes::default(); 10],
            plane: Plane::XY,
            units: Units::Inches,
            distance_mode: DistanceMode::Absolute,
            feed_rate_mode: FeedRateMode::UnitsPerMinute,
            spindle_mode: SpindleMode::Stop,
            coolant_mode: CoolantMode::None,
            override_mode: OverrideMode::Disable,
        }
    }
}

impl GrblState {
    pub fn apply_state(&mut self, state: Vec<Action>) {
        for s in state.clone() {
            match s {
                Action::Function(mode) => match mode {
                    Function::SetCoordinateSystem(2, _p) => {
                        // let wcs = self.wcs_offsets[p as usize].borrow_mut();
                        // if let Some(x) = extract_word!(&state, Word::X) {
                        //     wcs.0 = x;
                        // }
                        // if let Some(y) = extract_word!(&state, Word::Y) {
                        //     wcs.1 = y;
                        // }
                        // if let Some(z) = extract_word!(&state, Word::Z) {
                        //     wcs.2 = z;
                        // }
                    }
                    _ => {
                        println!("Skipping mode: {:?}", mode);
                    }
                },
                Action::SetFeedRate(f) => {
                    self.feed_rate = f;
                }
                Action::SetSpindleSpeed(s) => {
                    self.spindle_speed = s;
                }
                Action::SetWCS(wcs) => {
                    self.active_wcs = wcs;
                }
                Action::SetPlane(p) => {
                    self.plane = p;
                }
                Action::SetUnits(u) => {
                    self.units = u;
                }
                Action::SetDistanceMode(m) => {
                    self.distance_mode = m;
                }
                Action::SetFeedRateMode(m) => {
                    self.feed_rate_mode = m;
                }
                Action::Stop(m) => match m {
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
                Action::SetSpindleMode(m) => {
                    self.spindle_mode = m;
                }
                Action::SetCoolantMode(m) => {
                    self.coolant_mode = m;
                }
                Action::SetOverrideMode(m) => {
                    self.override_mode = m;
                }
                Action::SetSelectedTool(t) => {
                    self.tool = t;
                }
                Action::ToolChange => {
                    println!("Tool change");
                }
                Action::SetToolLengthMode(mode) => match mode {
                    ToolLengthMode::Set(_axes) => {
                        println!("Tool length mode set");
                    }
                    ToolLengthMode::Add(_axes) => {
                        println!("Tool length mode add");
                    }
                    ToolLengthMode::Cancel => {
                        todo!("Tool length mode cancel");
                    }
                },
            }
        }
    }
}
