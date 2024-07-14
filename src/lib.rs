#![no_std]
use embedded_hal::spi::SpiDevice;
mod calibration;
mod command;
mod error;
use calibration::{Calibration, OversamplingRatio};
use command::Command;
use error::DeviceError;

pub struct MS5611_01BA<SPI> {
    spi: SPI,
    calibration: Option<Calibration>, // a user can choose to calibrate the device
    oversampling_ratio: OversamplingRatio,
}

impl<SPI> MS5611_01BA<SPI>
where
    SPI: SpiDevice,
{
    /// Create a new instance of the MS5611_01BA03
    /// Clock speed must not exceed 20MHz
    /// Accept mode 0 or 3.
    /// Default to OSR1024
    pub fn new(spi: SPI) -> Self {
        // Should we calibrate in the constructor? No because this is beyond the scope of the constructor

        Self {
            spi,
            calibration: None,
            oversampling_ratio: OversamplingRatio::OSR1024,
        }
    }
    /// Every module is individually factory calibrated at two temperatures and two pressures. As a result, 6 coefficients
    /// necessary to compensate for process variations and temperature variations are calculated and stored in the 128-
    /// bit PROM of each module. These bits (partitioned into 6 coefficients) must be read by the microcontroller software
    /// and used in the program converting D1 and D2 into compensated pressure and temperature values.
    /// This could just be a function of the device struct
    /// Only needs to be called once.
    pub fn calibrate(&mut self) -> Result<(), DeviceError<SPI::Error>> {
        // Read PROM
        // Calculate calibration values

        Ok(())
    }

    fn set_oversampling_ratio(&mut self, ratio: OversamplingRatio) {
        self.oversampling_ratio = ratio;
    }

    fn read_digital_temp(&mut self) -> Result<i32, DeviceError<SPI::Error>> {
        Ok(0)
    }

    fn read_digital_pressure(&mut self) -> Result<i32, DeviceError<SPI::Error>> {
        Ok(0)
    }

    pub fn get_temperature_uncompensated(&mut self) -> Result<f32, DeviceError<SPI::Error>> {
        // Read digital temperature
        // Calculate temperature
        Ok(0.0)
    }

    pub fn get_pressure_uncompensated(&mut self) -> Result<f32, DeviceError<SPI::Error>> {
        // Read digital pressure
        // Calculate pressure
        Ok(0.0)
    }

    /// Second Order Temperature Compensation
    fn temp_compensate(&self, temp: i32, d_t: i32) -> Result<(i32, i64, i64), DeviceError<SPI>> {
        // We assume that we are in high temperature unless sensed otherwise.
        // I don't really like how this is just mutables galore.
        if let Some(ref calibration) = self.calibration {
            let mut t2: i32 = 0;
            let mut off2: i64 = 0;
            let mut sens2: i64 = 0;

            // temperature compensation
            if temp < 20 {
                t2 = d_t.pow(2) / (2_i32).pow(31);
                off2 = 5 * (temp as i64 - 2000).pow(2) / 2; // datasheet is / 2^1.
                sens2 = 5 * (temp as i64 - 2000).pow(2) / 4; // datasheet is / 2^2.
                if temp < -15 {
                    off2 += 7 * (temp as i64 + 1500).pow(2);
                    sens2 += 11 * (temp as i64 + 1500).pow(2) / 2;
                }
            }
            Ok((
                temp - t2,
                calibration.off as i64 - off2,
                calibration.sens as i64 - sens2,
            ))
        } else {
            Err(DeviceError::Uncalibrated)
        }
    }

    pub fn get_temperature(&mut self) -> Result<f32, DeviceError<SPI::Error>> {
        // Get uncompensated temperature
        // Get digital temperature
        // Get uncompensated pressure
        // Get digital pressure
        // Get temperature compensated pressure
        // Get pressure compensated temperature
        Ok(0.0)
    }

    pub fn get_pressure(&mut self) -> Result<f32, DeviceError<SPI::Error>> {
        // Get uncompensated temperature
        // Get digital temperature
        // Get uncompensated pressure
        // Get digital pressure
        // Get temperature compensated pressure
        // Get pressure compensated temperature
        Ok(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_command_d1_conversion() {
        let mut command = Command::D1Conversion(OversamplingRatio::OSR1024);
        assert_eq!(command.value(), 0x44);
        command = Command::D1Conversion(OversamplingRatio::OSR256);
        assert_eq!(command.value(), 0x40);
        command = Command::D1Conversion(OversamplingRatio::OSR512);
        assert_eq!(command.value(), 0x42);
    }

    #[test]
    fn check_command_d2_conversion() {
        let mut command = Command::D2Conversion(OversamplingRatio::OSR1024);
        assert_eq!(command.value(), 0x54);
        command = Command::D2Conversion(OversamplingRatio::OSR256);
        assert_eq!(command.value(), 0x50);
        command = Command::D2Conversion(OversamplingRatio::OSR512);
        assert_eq!(command.value(), 0x52);
    }

    #[test]
    fn check_command_read_prom() {
        let mut command = Command::ReadPROM(0, 1, 0);
        assert_eq!(command.value(), 0xA4);
        command = Command::ReadPROM(1, 1, 1);
        assert_eq!(command.value(), 0xAE);
    }

    #[test]
    fn check_temp_compensation() {}
}
