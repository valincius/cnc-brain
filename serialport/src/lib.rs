pub trait Serialport {
    fn read(&mut self) -> Vec<u8>;
    fn write(&mut self, data: &[u8]);
}

#[derive(Default)]
pub struct MockSerialport {
    pub buffer: Vec<u8>,
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
