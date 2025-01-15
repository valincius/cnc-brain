pub mod runner;

#[derive(Debug, Clone)]
pub struct MotionSegment {
    pub start_position: [f32; 3],
    pub end_position: [f32; 3],
    pub distance: f32,
    pub direction: [f32; 3],
    pub v_max: f32,
    pub v_in: f32,
    pub v_out: f32,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Coordinates {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
}

#[derive(Debug, Copy, Clone)]
pub enum Movement {
    Rapid(Coordinates),
    Linear { feedrate: f32, coords: Coordinates },
}
