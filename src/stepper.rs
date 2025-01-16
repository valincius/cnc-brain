pub struct Stepper {
    pub pin: u8,
}

impl Stepper {
    pub fn new(pin: u8) -> Self {
        Stepper { pin }
    }
}

impl Motor for Stepper {
    fn on(&self, speed: Option<f32>, direction: Option<Direction>) {
        // println!("Stepper on");
    }

    fn on_time(&self, time: f32, speed: Option<f32>, direction: Option<Direction>) {
        // println!("Stepper on for {} seconds", time);
    }

    fn on_cw(&self, speed: Option<f32>) {
        // println!("Stepper on clockwise");
    }

    fn on_ccw(&self, speed: Option<f32>) {
        // println!("Stepper on counter-clockwise");
    }

    fn on_time_cw(&self, time: f32, speed: Option<f32>) {
        // println!("Stepper on clockwise for {} seconds", time);
    }

    fn on_time_ccw(&self, time: f32, speed: Option<f32>) {
        // println!("Stepper on counter-clockwise for {} seconds", time);
    }

    fn off(&self) {
        // println!("Stepper off");
    }
}

pub trait Motor {
    fn on(&self, speed: Option<f32>, direction: Option<Direction>);
    fn on_time(&self, time: f32, speed: Option<f32>, direction: Option<Direction>);
    fn on_cw(&self, speed: Option<f32>);
    fn on_ccw(&self, speed: Option<f32>);
    fn on_time_cw(&self, time: f32, speed: Option<f32>);
    fn on_time_ccw(&self, time: f32, speed: Option<f32>);
    fn off(&self);
}

pub enum Direction {
    Forward,
    Backward,
}
