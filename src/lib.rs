#![no_std]
#![allow(clippy::needless_range_loop)] // Allow for i in 0..16 loop in CRC check

use embedded_hal_nb::{nb, spi::FullDuplex};
use nb::block;

pub mod calibration;
pub mod command;
pub mod error;

use calibration::{Calibration, OversamplingRatio};
use command::Command;
use error::DeviceError;

// NOTE: Reset delay (~3ms) must be handled externally by the caller

/// Driver for the MS5611-01BA03 pressure sensor using embedded-hal-nb.
///
/// This driver uses the non-blocking `FullDuplex` SPI trait and does *not*
/// manage delays internally. The caller is responsible for inserting appropriate
/// delays after reset and between starting a conversion and reading the result.
pub struct MS5611_01BA<SPI>
where
    SPI: FullDuplex<u8>,
    SPI::Error: Clone,
{
    spi: SPI,
    calibration: Result<Calibration, DeviceError<SPI::Error>>,
    oversampling_ratio: OversamplingRatio,
}

impl<SPI> MS5611_01BA<SPI>
where
    SPI: FullDuplex<u8>,
    SPI::Error: Clone,
{
    /// Creates a new driver instance and sends the Reset command.
    ///
    /// **IMPORTANT:** The caller MUST wait approximately 3ms after this function
    /// returns successfully before calling `calibrate()` or any other command,
    /// to allow the sensor to complete its reset sequence.
    ///
    /// The driver starts in an uncalibrated state. Call `calibrate()` after the reset delay.
    ///
    /// # Arguments
    /// * `spi` - An SPI peripheral implementing `embedded_hal_nb::spi::FullDuplex<u8>`.
    /// * `oversampling_ratio` - The desired default OSR for measurements.
    pub fn new(mut spi: SPI, oversampling_ratio: OversamplingRatio) -> Self {
        // Send Reset command, ignore potential error during init
        let _reset_res = block!(spi.write(Command::Reset.value())).map_err(DeviceError::Spi);
        // NOTE: Caller MUST wait ~3ms externally NOW.

        Self {
            spi,
            calibration: Err(DeviceError::Uncalibrated), // Start uncalibrated
            oversampling_ratio,
        }
    }

    /// Reads calibration data from the sensor's PROM and validates the CRC.
    ///
    /// This function **MUST** be called after `new()` and after the external ~3ms
    /// reset delay has elapsed. It updates the driver's internal calibration state.
    pub fn calibrate(&mut self) -> Result<(), DeviceError<SPI::Error>> {
        match Self::read_calibration_data(&mut self.spi) {
            Ok(cal_data) => {
                self.calibration = Ok(cal_data);
                Ok(())
            }
            Err(e) => {
                self.calibration = Err(e.clone()); // Store the error
                Err(e) // Return the error
            }
        }
    }

    /// Static method to read all 8 PROM words and validate CRC. Called by `calibrate`.
    fn read_calibration_data(spi: &mut SPI) -> Result<Calibration, DeviceError<SPI::Error>> {
        let mut prom_data = [0u16; 8];
        let dummy_byte = 0x00;
        for i in 0..8 {
            block!(spi.write(Command::ReadPROM(i as u8).value())).map_err(DeviceError::Spi)?;
            block!(spi.write(dummy_byte)).map_err(DeviceError::Spi)?;
            let msb = block!(spi.read()).map_err(DeviceError::Spi)?;
            block!(spi.write(dummy_byte)).map_err(DeviceError::Spi)?;
            let lsb = block!(spi.read()).map_err(DeviceError::Spi)?;
            prom_data[i] = u16::from_be_bytes([msb, lsb]);
        }
        let mut prom_data_for_crc = prom_data;
        if !Self::validate_crc4(&mut prom_data_for_crc) {
            return Err(DeviceError::InvalidCRC);
        }
        Ok(Calibration::new(&prom_data))
    }

    /// Sets the oversampling ratio for subsequent conversions.
    pub fn set_oversampling_ratio(&mut self, ratio: OversamplingRatio) {
        self.oversampling_ratio = ratio;
    }

    /// Gets the currently configured oversampling ratio.
    pub fn get_oversampling_ratio(&self) -> OversamplingRatio {
        self.oversampling_ratio.clone()
    }

    /// Sends the reset command to the sensor.
    ///
    /// **IMPORTANT:** The caller MUST wait approximately 3ms after this function
    /// returns successfully before calling `calibrate()` or any other command.
    pub fn reset(&mut self) -> Result<(), DeviceError<SPI::Error>> {
        block!(self.spi.write(Command::Reset.value())).map_err(DeviceError::Spi)
    }

    /// Sends the command to start a D2 (Temperature) conversion.
    ///
    /// **IMPORTANT:** After calling this function, the caller MUST wait for the
    /// conversion to complete before calling `read_adc_result()`.
    /// The required delay depends on the current OSR setting. Use
    /// `get_oversampling_ratio().delay_ms()` to find the typical delay needed (in milliseconds).
    pub fn start_temp_conversion(&mut self) -> Result<(), DeviceError<SPI::Error>> {
        let conversion_cmd = Command::D2Conversion(self.oversampling_ratio.clone()).value();
        block!(self.spi.write(conversion_cmd)).map_err(DeviceError::Spi)?;
        Ok(())
    }

    /// Sends the command to start a D1 (Pressure) conversion.
    ///
    /// **IMPORTANT:** After calling this function, the caller MUST wait for the
    /// conversion to complete before calling `read_adc_result()`.
    /// The required delay depends on the current OSR setting. Use
    /// `get_oversampling_ratio().delay_ms()` to find the typical delay needed (in milliseconds).
    pub fn start_pressure_conversion(&mut self) -> Result<(), DeviceError<SPI::Error>> {
        let conversion_cmd = Command::D1Conversion(self.oversampling_ratio.clone()).value();
        block!(self.spi.write(conversion_cmd)).map_err(DeviceError::Spi)?;
        Ok(())
    }

    /// Reads the 24-bit result from the sensor's ADC.
    ///
    /// **IMPORTANT:** This function should only be called *after* a conversion
    /// (`start_temp_conversion` or `start_pressure_conversion`) has been initiated
    /// AND the required conversion delay has elapsed (handled externally by the caller).
    /// Calling this too early will result in an invalid reading.
    ///
    /// # Returns
    /// The raw 24-bit ADC value (padded to u32).
    pub fn read_adc_result(&mut self) -> Result<u32, DeviceError<SPI::Error>> {
        let dummy_byte = 0x00;
        let mut adc_read_buf = [0u8; 3];
        block!(self.spi.write(Command::ReadADC.value())).map_err(DeviceError::Spi)?;
        for byte_val in adc_read_buf.iter_mut() {
            block!(self.spi.write(dummy_byte)).map_err(DeviceError::Spi)?;
            *byte_val = block!(self.spi.read()).map_err(DeviceError::Spi)?;
        }
        Ok(u32::from_be_bytes([
            0,
            adc_read_buf[0],
            adc_read_buf[1],
            adc_read_buf[2],
        ]))
    }

    /// Calculates the compensated pressure and temperature using the provided
    /// raw D1 (pressure) and D2 (temperature) ADC values and the stored calibration data.
    ///
    /// This function performs the 1st and 2nd order compensation calculations.
    /// It requires that `calibrate()` was called successfully beforehand.
    ///
    /// # Arguments
    /// * `d1_raw` - The raw 24-bit pressure ADC reading (obtained via `read_adc_result` after `start_pressure_conversion` and delay).
    /// * `d2_raw` - The raw 24-bit temperature ADC reading (obtained via `read_adc_result` after `start_temp_conversion` and delay).
    ///
    /// # Returns
    /// A tuple `(pressure, temperature)` on success:
    /// * `pressure`: Pressure in Pascals (Pa). (100 Pa = 1 mbar).
    /// * `temperature`: Temperature in hundredths of degrees Celsius (e.g., 2007 means 20.07 °C).
    ///
    /// # Errors
    /// Returns `DeviceError::Uncalibrated` if the driver hasn't been successfully calibrated.
    /// May return `DeviceError::{Under/Over}{Temperature/Pressure}` if the final
    /// calculated values fall outside the sensor's operating range.
    pub fn calculate_compensated_data(
        &self, // Takes &self because it only reads calibration
        d1_raw: u32,
        d2_raw: u32,
    ) -> Result<(i32, i32), DeviceError<SPI::Error>> {
        // Ensure calibration data is valid
        let cal = match self.calibration {
            Ok(c) => c,
            Err(ref e) => return Err(e.clone()), // Return stored calibration error
        };

        // --- Start Calculation (using i64 for intermediate values) ---
        let dt: i64 = d2_raw as i64 - (cal.t_ref as i64 * (1 << 8));
        let mut temp: i64 = 2000 + (dt * cal.temp_sens as i64 / (1 << 23));
        let mut off: i64 = (cal.off_t1 as i64 * (1 << 16)) + (cal.tco as i64 * dt / (1 << 7));
        let mut sens: i64 = (cal.sens_t1 as i64 * (1 << 15)) + (cal.tcs as i64 * dt / (1 << 8));

        // --- Second Order Compensation ---
        let mut t2: i64 = 0;
        let mut off2: i64 = 0;
        let mut sens2: i64 = 0;
        if temp < 2000 {
            t2 = dt.saturating_mul(dt) / (1i64 << 31);
            off2 = 5 * (temp - 2000).saturating_pow(2) / 2;
            sens2 = 5 * (temp - 2000).saturating_pow(2) / 4;
            if temp < -1500 {
                off2 += 7 * (temp + 1500).saturating_pow(2);
                sens2 += 11 * (temp + 1500).saturating_pow(2) / 2;
            }
        }
        temp -= t2;
        off -= off2;
        sens -= sens2;

        let p: i64 = ((d1_raw as i64 * sens / (1 << 21)) - off) / (1 << 15);

        // --- Range Checks ---
        if temp < -4000 {
            return Err(DeviceError::UnderTemperature);
        }
        if temp > 8500 {
            return Err(DeviceError::OverTemperature);
        }
        if p < 1000 {
            return Err(DeviceError::UnderPressure);
        }
        if p > 120000 {
            return Err(DeviceError::OverPressure);
        }

        Ok((p as i32, temp as i32))
    }

    /// Validates CRC4. Internal helper function.
    fn validate_crc4(prom_data: &mut [u16; 8]) -> bool {
        let mut n_rem = 0u16;
        let crc_read = prom_data[7] & 0x000F;
        prom_data[7] &= 0xFF00;
        for i in 0..16 {
            if i % 2 == 0 {
                n_rem ^= prom_data[i / 2] >> 8;
            } else {
                n_rem ^= prom_data[i / 2] & 0x00FF;
            }
            for _bit in 0..8 {
                if n_rem & 0x8000 != 0 {
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    n_rem <<= 1;
                }
            }
        }
        let crc_calculated = (n_rem >> 12) & 0x000F;
        crc_calculated == crc_read
    }
}
