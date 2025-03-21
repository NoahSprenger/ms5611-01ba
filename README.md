# MS5611-01BA Embedded-HAL Driver
This is a blocking driver which takes advantage of the Embedded-HAL Rust crate to enable platform independent usage. 
## Notes
- This is untested on hardware at the moment.
- Only supports SPI.
- Does not validate conditions like the SPI SCLK speed of less than 20MHz.
- SPI can use Mode 0 or 3 for phase and polarity.
