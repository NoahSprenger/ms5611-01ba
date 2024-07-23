use crate::calibration::OversamplingRatio;

pub enum Command {
    Reset,
    ReadPROM(u8, u8, u8), // (ad2, ad1, ad0) 128 bit calibration words not a fan since adx can only
    // be 0 or 1.
    D1Conversion(OversamplingRatio),
    D2Conversion(OversamplingRatio),
    ReadADC, // 24 bit pressure / temperature
}

impl Command {
    pub fn value(&self) -> u16 {
        match self {
            Command::Reset => 0x1E,
            Command::ReadPROM(ad2, ad1, ad0) => {
                (0xA0 | (ad2 << 1) | (ad1 << 2) | (ad0 << 3)) as u16
            }
            Command::D1Conversion(ratio) => (0x40 | ratio.value()) as u16,
            Command::D2Conversion(ratio) => (0x50 | ratio.value()) as u16,
            Command::ReadADC => 0x00,
        }
    }
}
