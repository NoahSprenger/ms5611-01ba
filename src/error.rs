#[derive(Copy, Clone, Debug)]
pub enum DeviceError<SPI> {
    Spi(SPI),
    UnderTemperature,
    OverTemperature,
    UnderPressure,
    OverPressure,
    Uncalibrated,
    InvalidCRC,
}
