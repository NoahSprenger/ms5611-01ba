#![no_std]
use embedded_hal::spi::SpiDevice;
mod error;
use error::Error;

pub struct MS5611_01BA<SPI> {
    spi: SPI,
}

impl<SPI> MS5611_01BA<SPI> where SPI: SpiDevice {
    /// Create a new instance of the MS5611_01BA03
    /// Clock speed must not exceed 20MHz
    /// Accept mode 0 or 3. 
    pub fn new(spi: SPI) -> Self {
        // Should we calibrate in the constructor? No because this is beyond the scope of the constructor



        Self { spi }
    }
    /// Every module is individually factory calibrated at two temperatures and two pressures. As a result, 6 coefficients
    /// necessary to compensate for process variations and temperature variations are calculated and stored in the 128-
    /// bit PROM of each module. These bits (partitioned into 6 coefficients) must be read by the microcontroller software
    /// and used in the program converting D1 and D2 into compensated pressure and temperature values.
    /// This could just be a function of the device struct 
    fn calibrate(&mut self) {
        // Read PROM
        // Calculate calibration values
    }

    /// Second Order Temperature Compensation
    fn temp_compensate(mut temp: i32, d_t: i32, mut off: i64, mut sens: i64) -> (i32, i64, i64) {
        // We assume that we are in high temperature unless sensed otherwise. 
        // I don't really like how this is just mutables galore. 
        let t2: i32; 
        let mut off2: i64;
        let mut sens2: i64;

        // temperature compensation 
        if temp < 20 {
            t2 = d_t.pow(2) / (2 as i32).pow(31);
            off2 = 5 * (temp as i64 - 2000).pow(2) / 2; // datasheet is / 2^1. 
            sens2 = 5 * (temp as i64 - 2000).pow(2) / 4; // datasheet is / 2^2. 
            if temp < -15 {
                off2 = off2 + 7 * (temp as i64 + 1500).pow(2); 
                sens2 = sens2 + 11 * (temp as i64 + 1500).pow(2) / 2; 
            }
            temp = temp - t2; 
            off = off - off2;
            sens = sens - sens2;
        } 
        (temp, off, sens)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_temp_compensation() {
        
    }
}
