#[derive(Copy, Clone, Debug)]
pub enum Error {
    UnderTemperature,
    OverTemperature,
    UnderPressure,
    OverPressure,
}