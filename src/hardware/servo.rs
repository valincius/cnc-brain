use super::Motor;

pub struct ServoDrive {
    pub pin: u8,
    pub feedback_pin: u8,
}

impl ServoDrive {
    pub fn new(pin: u8, feedback_pin: u8) -> Self {
        ServoDrive { pin, feedback_pin }
    }
}

impl Motor for ServoDrive {}
