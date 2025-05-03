//! A platform-agnostic Rust driver for the MS5611-01BA03 barometric pressure sensor,
//! using the `embedded-hal` traits.

#![no_std]
#![allow(clippy::needless_range_loop)] // Allow for i in 0..16 loop in CRC check

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};

pub mod calibration;
pub mod command;
pub mod error;

use calibration::{Calibration, OversamplingRatio};
use command::Command;
use error::DeviceError;

/// Reset command needs ~2.8ms, use 3ms to be safe.
const RESET_DELAY_MS: u32 = 3;

/// Represents the MS5611-01BA sensor and holds its configuration and state.
///
/// `SPI` is the type of the `SpiDevice`.
/// `Delay` is the type of the `DelayNs` provider.
/// `SpiE` is the error type associated with the `SpiDevice`.
pub struct MS5611_01BA<SPI, Delay>
where
    SPI: SpiDevice, // No need to specify data width here, handled by methods
    SPI::Error: Clone,
    Delay: DelayNs,
{
    spi: SPI,
    delay: Delay,
    /// Stores the result of the initial calibration attempt.
    /// If Ok, contains the calibration coefficients.
    /// If Err, contains the reason calibration failed (SPI error or Invalid CRC).
    calibration: Result<Calibration, DeviceError<SPI::Error>>,
    /// The currently selected oversampling ratio for conversions.
    oversampling_ratio: OversamplingRatio,
}

impl<SPI, Delay, SpiE> MS5611_01BA<SPI, Delay>
where
    SPI: SpiDevice<u8, Error = SpiE>, // Specify u8 data width for SPI operations
    SpiE: Clone,
    Delay: DelayNs,
{
    /// Creates a new driver instance, performs an initial reset, and reads
    /// the factory calibration data from the sensor's PROM.
    ///
    /// # Arguments
    ///
    /// * `spi` - An SPI peripheral implementing `embedded_hal::spi::SpiDevice<u8>`.
    ///           Clock speed should not exceed 20MHz. Mode 0 or 3 supported.
    /// * `delay` - A delay provider implementing `embedded_hal::delay::DelayNs`.
    /// * `oversampling_ratio` - The desired default oversampling ratio for measurements.
    ///
    /// # Errors
    ///
    /// Returns `DeviceError::Spi` if SPI communication fails during reset or calibration.
    /// Returns `DeviceError::InvalidCRC` if the PROM data checksum is incorrect,
    /// indicating potentially corrupted calibration data.
    pub fn new(
        mut spi: SPI,
        mut delay: Delay,
        oversampling_ratio: OversamplingRatio,
    ) -> Result<Self, DeviceError<SpiE>> {
        // 1. Reset the device to ensure a known state
        spi.write(&[Command::Reset.value()])
            .map_err(DeviceError::Spi)?;

        // 2. Wait for the reset sequence and PROM reload to complete.
        delay.delay_ms(RESET_DELAY_MS);

        // 3. Read and validate calibration data.
        let calibration_result = Self::read_calibration_data(&mut spi)?; // Propagate SPI errors from here

        // 4. Check CRC (read_calibration_data includes this check now)
        // Note: calibration_result is already checked for CRC within read_calibration_data

        // 5. Construct the driver instance
        Ok(Self {
            spi,
            delay,
            calibration: Ok(calibration_result), // Store the successful Calibration data
            oversampling_ratio,
        })
    }

    /// Reads the 8 PROM words (16 bytes total) containing factory calibration
    /// coefficients and performs a CRC check.
    ///
    /// # Returns
    ///
    /// `Ok(Calibration)` if PROM read is successful and CRC check passes.
    /// `Err(DeviceError::Spi)` if communication fails.
    /// `Err(DeviceError::InvalidCRC)` if the CRC check fails.
    fn read_calibration_data(spi: &mut SPI) -> Result<Calibration, DeviceError<SpiE>> {
        let mut prom_data = [0u16; 8]; // Buffer for 8x 16-bit words
        let mut read_buf = [0u8; 2]; // Temp buffer for SPI read

        // Read PROM words 0 through 7
        for i in 0..8 {
            let read_prom_cmd = Command::ReadPROM(i as u8).value();
            spi.transaction(&mut [
                Operation::Write(&[read_prom_cmd]),
                Operation::Read(&mut read_buf),
            ])
            .map_err(DeviceError::Spi)?;
            prom_data[i] = u16::from_be_bytes(read_buf);
        }

        // Make a mutable copy to pass to the CRC validation function,
        // as it modifies the array (clears CRC bits in word 7).
        let mut prom_data_for_crc = prom_data;

        // Validate CRC using the read data
        if !Self::validate_crc4(&mut prom_data_for_crc) {
            return Err(DeviceError::InvalidCRC);
        }

        // If CRC is valid, create and return the Calibration struct
        // using the original, unmodified prom_data array.
        Ok(Calibration::new(&prom_data))
    }

    /// Sets a new oversampling ratio for subsequent measurements.
    pub fn set_oversampling_ratio(&mut self, ratio: OversamplingRatio) {
        self.oversampling_ratio = ratio;
    }

    /// Gets the currently configured oversampling ratio.
    pub fn get_oversampling_ratio(&self) -> OversamplingRatio {
        self.oversampling_ratio.clone() // Clone since OSR might not be Copy
    }

    /// Sends the reset command to the sensor and waits for it to initialize.
    /// This can be used if the sensor enters an unknown state.
    pub fn reset(&mut self) -> Result<(), DeviceError<SpiE>> {
        self.spi
            .write(&[Command::Reset.value()])
            .map_err(DeviceError::Spi)?;
        self.delay.delay_ms(RESET_DELAY_MS);
        Ok(())
    }

    /// Performs a temperature conversion based on the current OSR setting
    /// and reads the resulting 24-bit raw digital value (D2).
    /// This is a low-level function; use `read_temperature` or `read_compensated_data`
    /// for compensated values.
    fn read_digital_temp(&mut self) -> Result<u32, DeviceError<SpiE>> {
        let conversion_cmd = Command::D2Conversion(self.oversampling_ratio.clone()).value();
        self.spi
            .write(&[conversion_cmd])
            .map_err(DeviceError::Spi)?;

        // Wait for the conversion to complete based on the OSR
        self.delay.delay_ns(self.oversampling_ratio.delay_ns());

        let mut adc_read_buf = [0u8; 3]; // Buffer for the 3-byte ADC result
        self.spi
            .transaction(&mut [
                Operation::Write(&[Command::ReadADC.value()]),
                Operation::Read(&mut adc_read_buf),
            ])
            .map_err(DeviceError::Spi)?;

        // Construct the 32-bit unsigned integer from the 3 bytes read (MSB first)
        Ok(u32::from_be_bytes([
            0,               // Pad with 0 for the most significant byte
            adc_read_buf[0], // MSB of the 24-bit result
            adc_read_buf[1], // Middle byte
            adc_read_buf[2], // LSB
        ]))
    }

    /// Performs a pressure conversion based on the current OSR setting
    /// and reads the resulting 24-bit raw digital value (D1).
    /// This is a low-level function; use `read_compensated_data` for the
    /// compensated pressure value.
    fn read_digital_pressure(&mut self) -> Result<u32, DeviceError<SpiE>> {
        let conversion_cmd = Command::D1Conversion(self.oversampling_ratio.clone()).value();
        self.spi
            .write(&[conversion_cmd])
            .map_err(DeviceError::Spi)?;

        // Wait for the conversion to complete
        self.delay.delay_ns(self.oversampling_ratio.delay_ns());

        let mut adc_read_buf = [0u8; 3]; // Buffer for the 3-byte ADC result
        self.spi
            .transaction(&mut [
                Operation::Write(&[Command::ReadADC.value()]),
                Operation::Read(&mut adc_read_buf),
            ])
            .map_err(DeviceError::Spi)?;

        // Construct the 32-bit unsigned integer from the 3 bytes read (MSB first)
        Ok(u32::from_be_bytes([
            0,               // Pad with 0
            adc_read_buf[0], // MSB
            adc_read_buf[1], // Middle byte
            adc_read_buf[2], // LSB
        ]))
    }

    /// Reads the current temperature, applying first-order compensation.
    /// For potentially higher accuracy (especially at temperature extremes),
    /// use `read_compensated_data` which includes second-order compensation.
    ///
    /// # Returns
    ///
    /// Temperature in hundredths of degrees Celsius (e.g., 2007 means 20.07 °C).
    ///
    /// # Errors
    ///
    /// Returns `DeviceError::Uncalibrated` if calibration data is missing/invalid.
    /// Returns `DeviceError::Spi` if communication fails.
    pub fn read_temperature(&mut self) -> Result<i32, DeviceError<SpiE>> {
        // Ensure calibration data is valid before proceeding
        let cal = match self.calibration {
            Ok(c) => c,
            Err(ref e) => return Err(e.clone()),
        };

        // Perform temperature reading and get raw D2
        let d2 = self.read_digital_temp()?;

        // --- Calculate 1st Order Temperature ---
        // Use i64 for intermediate calculations to prevent overflow

        // dT = D2 - C5 * 2^8
        let dt: i64 = d2 as i64 - (cal.t_ref as i64 * (1 << 8)); // 1<<8 = 256

        // TEMP = 2000 + dT * C6 / 2^23
        let temp: i64 = 2000i64 + (dt * cal.temp_sens as i64 / (1 << 23));

        // Optional: Check if temp is within operating range before returning
        if temp < -4000 {
            return Err(DeviceError::UnderTemperature);
        }
        if temp > 8500 {
            return Err(DeviceError::OverTemperature);
        }

        Ok(temp as i32) // Final temperature should fit in i32
    }

    /// Reads both pressure and temperature, applying second-order compensation
    /// for the highest accuracy according to the datasheet formulas.
    ///
    /// # Returns
    ///
    /// A tuple `(pressure, temperature)` on success:
    /// * `pressure`: Pressure in Pascals (Pa). Note: 100 Pa = 1 mbar. (e.g., 100009 Pa = 1000.09 mbar).
    /// * `temperature`: Temperature in hundredths of degrees Celsius (e.g., 2007 means 20.07 °C).
    ///
    /// # Errors
    ///
    /// Returns `DeviceError::Uncalibrated` if calibration data is missing/invalid.
    /// Returns `DeviceError::Spi` if communication fails during readings.
    /// May return `DeviceError::{Under/Over}{Temperature/Pressure}` if the final
    /// calculated values fall outside the sensor's operating range.
    pub fn read_compensated_data(&mut self) -> Result<(i32, i32), DeviceError<SpiE>> {
        // Ensure calibration data is valid
        let cal = match self.calibration {
            Ok(c) => c,
            Err(ref e) => return Err(e.clone()),
        };

        // 1. Read Raw Sensor Data (Temperature D2 first, then Pressure D1)
        let d2 = self.read_digital_temp()?;
        let d1 = self.read_digital_pressure()?;

        // --- Start Calculation (using i64 for intermediate values) ---

        // 2. Calculate 1st order temperature & difference dT
        // dT = D2 - C5 * 2^8
        let dt: i64 = d2 as i64 - (cal.t_ref as i64 * (1 << 8));

        // TEMP = 2000 + dT * C6 / 2^23
        // This is the first-order compensated temperature
        let mut temp: i64 = 2000 + (dt * cal.temp_sens as i64 / (1 << 23));

        // 3. Calculate 1st order Offset and Sensitivity (using first-order temp `dt`)
        // OFF = C2 * 2^16 + (C4 * dT) / 2^7
        let mut off: i64 = (cal.off_t1 as i64 * (1 << 16)) + (cal.tco as i64 * dt / (1 << 7));

        // SENS = C1 * 2^15 + (C3 * dT) / 2^8
        let mut sens: i64 = (cal.sens_t1 as i64 * (1 << 15)) + (cal.tcs as i64 * dt / (1 << 8));

        // --- Second Order Temperature Compensation ---
        // These adjustments are applied if the first-order temperature (temp)
        // is outside the optimal range (below 20°C).
        let mut t2: i64 = 0;
        let mut off2: i64 = 0;
        let mut sens2: i64 = 0;

        // Check if temperature is below 20°C (2000 hundredths)
        if temp < 2000 {
            // Calculate T2 = dT^2 / 2^31
            // Ensure intermediate squaring doesn't overflow i64, though unlikely with dT range
            t2 = dt.saturating_mul(dt) / (1i64 << 31);

            // Calculate OFF2 = 5 * (TEMP - 2000)^2 / 2^1
            off2 = 5 * (temp - 2000).saturating_pow(2) / 2; // or >> 1

            // Calculate SENS2 = 5 * (TEMP - 2000)^2 / 2^2
            sens2 = 5 * (temp - 2000).saturating_pow(2) / 4; // or >> 2

            // Additional adjustments if temperature is very low (below -15°C)
            if temp < -1500 {
                // OFF2 = OFF2 + 7 * (TEMP + 1500)^2
                off2 += 7 * (temp + 1500).saturating_pow(2);
                // SENS2 = SENS2 + 11 * (TEMP + 1500)^2 / 2^1
                sens2 += 11 * (temp + 1500).saturating_pow(2) / 2;
            }
        }

        // Apply the second-order adjustments to the temperature, offset, and sensitivity
        temp -= t2;
        off -= off2;
        sens -= sens2;

        // 4. Calculate Final Compensated Pressure
        // P = (D1 * SENS / 2^21 - OFF) / 2^15
        let p: i64 = ((d1 as i64 * sens / (1 << 21)) - off) / (1 << 15);

        // --- Check Final Values Against Operating Ranges ---
        // Temperature: -40.00°C to +85.00°C
        if temp < -4000 {
            return Err(DeviceError::UnderTemperature);
        }
        if temp > 8500 {
            return Err(DeviceError::OverTemperature);
        }
        // Pressure: 10.00 mbar to 1200.00 mbar (1000 Pa to 120000 Pa)
        if p < 1000 {
            return Err(DeviceError::UnderPressure);
        }
        if p > 120000 {
            return Err(DeviceError::OverPressure);
        }

        // Cast final results to i32 (should fit within i32 range) and return
        Ok((p as i32, temp as i32))
    }

    /// Validates the 4-bit CRC checksum of the PROM data.
    ///
    /// The algorithm is based on Application Note AN520.
    ///
    /// # Arguments
    ///
    /// * `prom_data` - A mutable slice containing the 8x 16-bit words read from PROM.
    ///                 This function **will modify** word 7 by clearing the CRC bits
    ///                 during calculation. Pass a copy if the original is needed later.
    ///
    /// # Returns
    ///
    /// `true` if the calculated CRC matches the one stored in PROM word 7, `false` otherwise.
    fn validate_crc4(prom_data: &mut [u16; 8]) -> bool {
        let mut n_rem = 0u16; // CRC remainder, initialized to 0

        // Extract the original 4-bit CRC read from the sensor
        // (lowest 4 bits of the 7th word, address 7)
        let crc_read = prom_data[7] & 0x000F;

        // Clear the CRC bits in the copy of PROM data word 7 for calculation
        prom_data[7] &= 0xFF00;

        // Iterate through all 16 bytes (8 words) of the PROM data
        for i in 0..16 {
            // XOR the next byte into the remainder
            if i % 2 == 0 {
                // High byte of prom_data[i/2]
                n_rem ^= prom_data[i / 2] >> 8;
            } else {
                // Low byte of prom_data[i/2]
                n_rem ^= prom_data[i / 2] & 0x00FF;
            }

            // Process 8 bits for the CRC polynomial division
            // Polynomial: x^4 + x^3 + x^2 + 1 => 10011 => 0x3000 (shifted left by 12)
            for _bit in 0..8 {
                if n_rem & 0x8000 != 0 {
                    // If MSB (bit 15) is set, XOR with polynomial coefficient
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    // Otherwise, just shift left
                    n_rem <<= 1;
                }
            }
        }

        // The final 4-bit CRC remainder is in bits 15-12 of n_rem
        let crc_calculated = (n_rem >> 12) & 0x000F;

        // Compare calculated CRC with the CRC read from PROM
        crc_calculated == crc_read
    }
}
