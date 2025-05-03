#[derive(Copy, Clone, Debug)]
pub enum DeviceError<SPI> {
    /// SPI communication error
    Spi(SPI),
    /// Temperature reading was below the sensor's specified minimum (-40 C)
    /// Calculation might be inaccurate.
    UnderTemperature,
    /// Temperature reading was above the sensor's specified maximum (+85 C)
    /// Calculation might be inaccurate.
    OverTemperature,
    /// Pressure reading was below the sensor's specified minimum (10 mbar)
    /// Calculation might be inaccurate.
    UnderPressure,
    /// Pressure reading was above the sensor's specified maximum (1200 mbar)
    /// Calculation might be inaccurate.
    OverPressure,
    /// Attempted to read data before calibration coefficients were successfully read.
    Uncalibrated,
    /// The CRC check on the PROM data failed, calibration data is suspect.
    InvalidCRC,
}
