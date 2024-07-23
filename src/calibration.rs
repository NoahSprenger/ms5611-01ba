/// Oversampling Ratio
#[derive(Clone)]
pub enum OversamplingRatio {
    OSR256,
    OSR512,
    OSR1024,
    OSR2048,
    OSR4096,
}

impl OversamplingRatio {
    pub fn value(&self) -> u8 {
        match *self {
            OversamplingRatio::OSR256 => 0x00,
            OversamplingRatio::OSR512 => 0x02,
            OversamplingRatio::OSR1024 => 0x04,
            OversamplingRatio::OSR2048 => 0x06,
            OversamplingRatio::OSR4096 => 0x08,
        }
    }
    pub fn delay(&self) -> u8 {
        // 0.5 / 1.1 / 2.1 / 4.1 / 8.22 ms return in ns
        match *self {
            OversamplingRatio::OSR256 => 1,
            OversamplingRatio::OSR512 => 2,
            OversamplingRatio::OSR1024 => 3,
            OversamplingRatio::OSR2048 => 5,
            OversamplingRatio::OSR4096 => 9,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Calibration {
    pub sens: u16,
    pub off: u16,
    pub tcs: u16, // temperature coefficient of sensitivity
    pub tco: u16, // temperature coefficient of offset
    pub t_ref: u16,
    pub temp_sens: u16,
}

impl Calibration {
    pub fn new(buf: &[u16; 8]) -> Calibration {
        Calibration {
            sens: buf[1],
            off: buf[2],
            tcs: buf[3],
            tco: buf[4],
            t_ref: buf[5],
            temp_sens: buf[6],
        }
    }
}
