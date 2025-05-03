use crate::calibration::OversamplingRatio;

/// Commands for the MS5611 sensor
#[derive(Debug)] // Added Debug
pub enum Command {
    /// Resets the sensor
    Reset,
    /// Reads a 16-bit word from the internal PROM at the given address (0-7)
    ReadPROM(u8), // Changed: Takes address 0-7 directly
    /// Starts a pressure conversion with the specified Oversampling Ratio
    D1Conversion(OversamplingRatio),
    /// Starts a temperature conversion with the specified Oversampling Ratio
    D2Conversion(OversamplingRatio),
    /// Reads the 24-bit result from the ADC after a conversion
    ReadADC,
}

impl Command {
    /// Gets the 8-bit byte value for the command
    pub fn value(&self) -> u8 {
        match self {
            Command::Reset => 0x1E,
            // Corrected logic: 0xA0 ORed with (address shifted left by 1)
            Command::ReadPROM(address) => {
                // Ensure address is within the valid range 0-7
                let addr = address & 0x07;
                0xA0 | (addr << 1)
            }
            Command::D1Conversion(ratio) => 0x40 | ratio.value(),
            Command::D2Conversion(ratio) => 0x50 | ratio.value(),
            Command::ReadADC => 0x00,
        }
    }
}

// --- Tests ---
#[cfg(test)]
mod tests {
    use super::*; // Imports Command and OversamplingRatio

    #[test]
    fn check_command_reset() {
        let command = Command::Reset;
        assert_eq!(command.value(), 0x1E, "Reset command mismatch");
    }

    #[test]
    fn check_command_read_adc() {
        let command = Command::ReadADC;
        assert_eq!(command.value(), 0x00, "ReadADC command mismatch");
    }

    #[test]
    fn check_command_d1_conversion_all_osr() {
        let test_cases = [
            (OversamplingRatio::OSR256, 0x40),
            (OversamplingRatio::OSR512, 0x42),
            (OversamplingRatio::OSR1024, 0x44),
            (OversamplingRatio::OSR2048, 0x46),
            (OversamplingRatio::OSR4096, 0x48),
        ];

        for (osr, expected_value) in test_cases {
            let command = Command::D1Conversion(osr.clone());
            assert_eq!(
                command.value(),
                expected_value,
                "D1 Conversion mismatch for OSR {:?}",
                osr
            );
        }
    }

    #[test]
    fn check_command_d2_conversion_all_osr() {
        let test_cases = [
            (OversamplingRatio::OSR256, 0x50),
            (OversamplingRatio::OSR512, 0x52),
            (OversamplingRatio::OSR1024, 0x54),
            (OversamplingRatio::OSR2048, 0x56),
            (OversamplingRatio::OSR4096, 0x58),
        ];

        for (osr, expected_value) in test_cases {
            let command = Command::D2Conversion(osr.clone());
            assert_eq!(
                command.value(),
                expected_value,
                "D2 Conversion mismatch for OSR {:?}",
                osr
            );
        }
    }

    #[test]
    fn check_command_read_prom_all_addresses() {
        // Test all 8 PROM addresses (0 to 7)
        for address in 0..8 {
            // Calculate the CORRECT expected command byte according to the datasheet:
            let expected_value = 0xA0 | (address << 1);

            // Generate the command using the corrected implementation
            let command = Command::ReadPROM(address);
            let actual_value = command.value();

            // Assert that the generated value matches the datasheet specification
            assert_eq!(
                actual_value, expected_value,
                "ReadPROM command mismatch for address {}. Expected {:#04X}, got {:#04X}",
                address, expected_value, actual_value
            );
        }
    }

    #[test]
    fn check_read_prom_address_clamping() {
        // Test that addresses outside 0-7 get clamped correctly
        let command_high = Command::ReadPROM(10); // 10 = 1010b, should clamp to 010b = 2
        let expected_high = 0xA0 | (2 << 1); // 0xA4
        assert_eq!(
            command_high.value(),
            expected_high,
            "ReadPROM address clamping failed for high value"
        );

        let command_exact = Command::ReadPROM(7);
        let expected_exact = 0xA0 | (7 << 1); // 0xAE
        assert_eq!(
            command_exact.value(),
            expected_exact,
            "ReadPROM address clamping failed for exact value 7"
        );
    }

    // Keep the empty test for temp compensation as a placeholder
    #[test]
    fn check_temp_compensation() {
        // TODO: Add tests for calculation logic using known inputs/outputs
    }
}
