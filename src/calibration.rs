/// Oversampling Ratio
#[derive(Clone, Debug, PartialEq, Eq)]
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

    /// Gets the required typical conversion delay in MILLISECONDS.
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

/// Factory calibration data read from PROM
#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    /// C1: Pressure sensitivity | SENST1
    pub sens_t1: u16,
    /// C2: Pressure offset | OFFT1
    pub off_t1: u16,
    /// C3: Temperature coefficient of pressure sensitivity | TCS
    pub tcs: u16,
    /// C4: Temperature coefficient of pressure offset | TCO
    pub tco: u16,
    /// C5: Reference temperature | TREF
    pub t_ref: u16,
    /// C6: Temperature coefficient of the temperature | TEMPSENS
    pub temp_sens: u16,
    // We don't store PROM[0] (manufacturer info) or PROM[7] (Serial/CRC) here
}

impl Calibration {
    pub fn new(buf: &[u16; 8]) -> Calibration {
        Calibration {
            sens_t1: buf[1],
            off_t1: buf[2],
            tcs: buf[3],
            tco: buf[4],
            t_ref: buf[5],
            temp_sens: buf[6],
        }
    }
}
