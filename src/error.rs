#[derive(Copy, Clone, Debug)]
pub enum DeviceError {
    Io,
    UnderTemperature,
    OverTemperature,
    UnderPressure,
    OverPressure,
    Uncalibrated,
    InvalidCRC,
}
