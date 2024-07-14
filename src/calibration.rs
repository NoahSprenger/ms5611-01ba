


/// Oversampling Ratio
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
}

pub struct Calibration {
    pub sens: u16,
    pub off: u16,
    tcs: u16, // temperature coefficient of sensitivity
    tco: u16, // temperature coefficient of offset
    t_ref: u16,
    temp_sens: u16,
}
