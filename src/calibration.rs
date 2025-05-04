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
    /// Gets the command bits for the OSR setting
    pub fn value(&self) -> u8 {
        match *self {
            OversamplingRatio::OSR256 => 0x00,
            OversamplingRatio::OSR512 => 0x02,
            OversamplingRatio::OSR1024 => 0x04,
            OversamplingRatio::OSR2048 => 0x06,
            OversamplingRatio::OSR4096 => 0x08,
        }
    }

    /// Gets the required conversion delay in nanoseconds
    pub fn delay_ns(&self) -> u32 {
        // Values from datasheet pg 3 (typ column) converted to ns
        match *self {
            OversamplingRatio::OSR256 => 540_000,    // 0.54 ms
            OversamplingRatio::OSR512 => 1_060_000,  // 1.06 ms
            OversamplingRatio::OSR1024 => 2_080_000, // 2.08 ms
            OversamplingRatio::OSR2048 => 4_130_000, // 4.13 ms
            OversamplingRatio::OSR4096 => 8_220_000, // 8.22 ms
        }
    }

    /// Gets the required conversion delay in microseconds
    pub fn delay_us(&self) -> u32 {
        self.delay_ns() / 1_000
    }

    /// Gets the required conversion delay in milliseconds
    pub fn delay_ms(&self) -> u32 {
        self.delay_ns() / 1_000_000
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
    /// Creates a new Calibration struct from the raw PROM data buffer.
    /// Assumes the buffer contains data read from addresses 0 through 7,
    /// and that the CRC has already been validated.
    pub fn new(buf: &[u16; 8]) -> Calibration {
        Calibration {
            sens_t1: buf[1],   // C1 from PROM addr 1
            off_t1: buf[2],    // C2 from PROM addr 2
            tcs: buf[3],       // C3 from PROM addr 3
            tco: buf[4],       // C4 from PROM addr 4
            t_ref: buf[5],     // C5 from PROM addr 5
            temp_sens: buf[6], // C6 from PROM addr 6
        }
    }
}
