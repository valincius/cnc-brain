use super::gcode_parser::{LineMode, ParserState};

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
pub enum SpindleMode {
    Clockwise,
    CounterClockwise,
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

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum ToolLengthMode {
    Set(f32),
    Add(f32),
    Cancel,
}

#[derive(Debug)]
pub struct MachineState {
    pub position: (f32, f32, f32),
    pub feed_rate: f32,
    pub spindle_speed: f32,
    pub active_wcs: usize,
    pub wcs_offsets: Vec<(f32, f32, f32)>,
    pub plane: Plane,
    pub units: Units,
    pub distance_mode: DistanceMode,
    pub feed_rate_mode: FeedRateMode,
    pub spindle_mode: SpindleMode,
    pub coolant_mode: CoolantMode,
    pub override_mode: OverrideMode,
    pub tool: usize,
}

impl MachineState {
    pub fn new() -> Self {
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

    pub fn apply_state(&mut self, state: Vec<ParserState>) {
        let word_x = state.iter().find_map(|s| match s {
            ParserState::X(x) => Some(*x),
            _ => None,
        });
        let word_y = state.iter().find_map(|s| match s {
            ParserState::Y(y) => Some(*y),
            _ => None,
        });
        let word_z = state.iter().find_map(|s| match s {
            ParserState::Z(z) => Some(*z),
            _ => None,
        });
        let word_l = state.iter().find_map(|s| match s {
            ParserState::L(l) => Some(*l),
            _ => None,
        });
        let word_p = state.iter().find_map(|s| match s {
            ParserState::P(p) => Some(*p),
            _ => None,
        });

        for s in state {
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
