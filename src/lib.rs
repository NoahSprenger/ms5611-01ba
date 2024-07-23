#![no_std]
use embedded_hal::spi::FullDuplex;
pub mod calibration;
pub mod command;
pub mod error;
use calibration::{Calibration, OversamplingRatio};
use command::Command;
use core::fmt::Debug;
use embedded_hal::blocking::delay::DelayMs;
use error::DeviceError;

pub struct MS5611_01BA<SPI>
where
    SPI: FullDuplex<u8> + Debug,
{
    spi: SPI,
    calibration: Result<Calibration, DeviceError>, // a user can choose to calibrate the device
    oversampling_ratio: OversamplingRatio,
}

impl<SPI> MS5611_01BA<SPI>
where
    SPI: FullDuplex<u8> + Debug,
    <SPI as FullDuplex<u8>>::Error: Debug,
{
    /// Create a new instance of the MS5611_01BA03
    /// Clock speed must not exceed 20MHz
    /// Accept mode 0 or 3.
    /// Default to OSR1024
    pub fn new(mut spi: SPI, oversampling_ratio: OversamplingRatio) -> Self {
        // Should we calibrate in the constructor? No because this is beyond the scope of the constructor

        let calibration = Self::calibrate(&mut spi);

        Self {
            spi,
            calibration,
            oversampling_ratio,
        }
    }
    /// Every module is individually factory calibrated at two temperatures and two pressures. As a result, 6 coefficients
    /// necessary to compensate for process variations and temperature variations are calculated and stored in the 128-
    /// bit PROM of each module. These bits (partitioned into 6 coefficients) must be read by the microcontroller software
    /// and used in the program converting D1 and D2 into compensated pressure and temperature values.
    /// This could just be a function of the device struct
    /// Only needs to be called once.
    fn calibrate(spi: &mut SPI) -> Result<Calibration, DeviceError> {
        // Read PROM
        // Calculate calibration values
        //  Address 0 contains factory data and the setup, addresses 1-6 calibration coefficients and address 7 contains
        // the serial code and CRC.
        let mut buf = [0u16; 8]; // 16 bits

        for i in 0..8 {
            spi.send(Command::ReadPROM(i / 4, (i / 2) % 2, i % 2).value())
                .map_err(|_| DeviceError::Io)?;
            let temp_val = {
                (spi.read().map_err(|_| DeviceError::Io)? as u16) << 8
                    | spi.read().map_err(|_| DeviceError::Io)? as u16
            };
            buf[i as usize] = temp_val;
        }

        // before setting calibration values, check CRC.
        // 4 bit CRC
        if !crc4(&mut buf) {
            return Err(DeviceError::InvalidCRC);
        }

        let calibration = Calibration::new(&buf);

        Ok(calibration)
    }

    pub fn set_oversampling_ratio(&mut self, ratio: OversamplingRatio) {
        self.oversampling_ratio = ratio;
    }

    pub fn reset(&mut self) -> Result<(), DeviceError> {
        self.spi
            .send(Command::Reset.value())
            .map_err(|_| DeviceError::Io)
    }

    fn read_digital_temp<Delay: DelayMs<u8>>(
        &mut self,
        delay: &mut Delay,
    ) -> Result<u32, DeviceError> {
        // send d2 conversion command
        // wait for conversion
        // read digital temperature
        self.spi
            .send(Command::D2Conversion(self.oversampling_ratio.clone()).value())
            .map_err(|_| DeviceError::Io)?;

        delay.delay_ms(self.oversampling_ratio.delay());

        let mut temp_buf = [0u8; 4];

        self.spi
            .send(Command::ReadADC.value())
            .map_err(|_| DeviceError::Io)?;

        for i in 0..temp_buf.len() {
            temp_buf[i] = self.spi.read().map_err(|_| DeviceError::Io)?;
        }
        Ok(u32::from_be_bytes(temp_buf))
    }

    fn read_digital_pressure<Delay: DelayMs<u8>>(
        &mut self,
        delay: &mut Delay,
    ) -> Result<u32, DeviceError> {
        let mut temp_buf = [0u8; 4];

        self.spi
            .send(Command::D1Conversion(self.oversampling_ratio.clone()).value())
            .map_err(|_| DeviceError::Io)?;

        delay.delay_ms(self.oversampling_ratio.delay());

        self.spi
            .send(Command::ReadADC.value())
            .map_err(|_| DeviceError::Io)?;

        for i in 0..temp_buf.len() {
            temp_buf[i] = self.spi.read().map_err(|_| DeviceError::Io)?;
        }

        Ok(u32::from_be_bytes(temp_buf))
    }

    fn get_temperature_uncompensated<Delay: DelayMs<u8>>(
        &mut self,
        delay: &mut Delay,
    ) -> Result<(i32, i32), DeviceError> {
        // Read digital temperature
        // Calculate temperature
        let digital_temp = self.read_digital_temp(delay)?;

        if let Ok(ref calibration) = self.calibration {
            let d_t: i32 = digital_temp as i32 - calibration.t_ref as i32;
            let temp = 2000 + d_t * calibration.temp_sens as i32 / 2_i32.pow(23);
            return Ok((temp, d_t));
        }
        Err(DeviceError::Uncalibrated)
    }

    /// Second Order Temperature Compensation
    fn temp_compensate(&self, temp: i32, d_t: i32) -> Result<(i32, i64, i64), DeviceError> {
        // We assume that we are in high temperature unless sensed otherwise.
        // I don't really like how this is just mutables galore.
        if let Ok(ref calibration) = self.calibration {
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

    /// Returns (pressure mbar, temperature celcius)
    pub fn get_data<Delay: DelayMs<u8>>(
        &mut self,
        delay: &mut Delay,
    ) -> Result<(i32, i32), DeviceError> {
        let (temp, d_t) = self.get_temperature_uncompensated(delay)?;
        let (temp, mut off, mut sens) = self.temp_compensate(temp, d_t)?;
        let d1 = self.read_digital_pressure(delay).unwrap();
        if let Ok(ref calibration) = self.calibration {
            off += calibration.tco as i64 * d_t as i64;
            sens += calibration.tcs as i64 * d_t as i64;
            let p = d1 as i64 * sens - off;
            return Ok((p as i32, temp));
        }
        Err(DeviceError::Uncalibrated)
    }
}

/// CRC4 is calculated from the 16 bits of the PROM. The CRC is calculated by the polynomial x^4 + x^3 + x^2 + 1.
/// This should live elsewhere since it's not really a method of the device.
fn crc4(prom: &mut [u16; 8]) -> bool {
    let mut n_rem = 0_u16;
    let crc_read = prom[7] & 0xF; // last 4 bits
    prom[7] = prom[7] & 0xFF00; // clear last 4 bits
    for i in 0..16 {
        if i % 2 == 1 {
            n_rem ^= prom[i >> 1] & 0x00FF;
        } else {
            n_rem ^= prom[i >> 1] >> 8;
        }
        for _ in 0..8 {
            if n_rem & 0x8000 != 0 {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = n_rem << 1;
            }
        }
    }
    n_rem = n_rem >> 12;
    n_rem ^= 0x00;
    crc_read == n_rem
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
