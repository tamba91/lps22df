 This crate provides a platform-agnostic driver for the ST LPS22DF pressure-temperature sensor driver.
 The datasheet and other documentation is available at <https://www.st.com/en/mems-and-sensors/lps22df.html>.
 This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) traits.
 Ensure that the hardware abstraction layer of your microcontroller implements the embedded-hal traits.

 ## Instantiating

 Create an instance of the driver with the `new_i2c` or `new_spi` associated function, by passing i2c and address instances
 or an spi (SpiDevice) instance.
 
 ### I2C:

```rust
 use lps22df::{Lps22df, I2CAddress}
-
 let mut sensor = Lps22df::new_i2c(i2c, I2CAddress::Address1).unwrap();
 ```
 
 i2c instance must implement the I2c trait of embedded-hal. The address is an enum variant of I2CAddress enum.
 There are two addresses available: 0x5C and 0x5D, which correspond to enum variants Address0 and Address1.
 Check the datasheet for I2C address configuration.

 If multiple sensors are used on the same I2C bus, create an instance of i2c that implements bus sharing:
 (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
```rust
 use core::cell::RefCell;
 use embedded_hal_bus::i2c;

 use lps22df::{Lps22df, I2CAddress as Lps22dfAddress};
 use stts22h::{Stts22h, I2CAddress as Stts22hAddress}; // the STTS22H is another sensor of ST MEMS family

 let i2c_bus = RefCell::new(i2c);
```
 and then create an instance of i2c for each sensor connected. 
```rust
 let mut lps22df = Lps22df::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Lps22dfAddress::Address1).unwrap();
 let mut stts22h = Stts22h::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Stts22hAddress::Address0).unwrap();
```
 
 In this example sharing is implemented with a RefCell, so it only allows sharing within a single thread.
 If you need to share a bus across several threads, use CriticalSectionDevice instead.

 ### SPI:

 HALs normally implement SpiBus trait. In order to obtain an SpiDevice from an SpiBus embedded-hal-bus crate can be used.
 (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
 ```rust
 use embedded_hal_bus::spi::RefCellDevice;
 use lps22df::Lps22df;
 let spi = RefCellDevice::new_no_delay(&spi, cs).unwrap(); // cs is the chip select pin
 let mut sensor = Lps22df::new_spi(spi).unwrap();
 sensor.disable_i2c_interface().unwrap(); // when using spi, the I2C interface can be disabled
 ```
 
 ## Setting ODR and AVG:
 Set the output data rate and the resolution with the `set_odr` and the `set_avg` methods.
 ```rust
 sensor.set_odr(0).unwrap(); // by passing '0' we put the sensor in power-down/one-shot mode
 sensor.set_avg(512).unwrap(); // 512 gives the best resolution
 ```
 
 Start a pressure and temperature sample measurement with `one_shot` method.
 Read the measured pressure and temperature with `get_temp` method.
 ```rust
 loop {
     sensor.one_shot().unwrap();
     let values = sensor.get_values().unwrap();
     writeln!(tx, "press: {}, temp: {}", values.0, values.1).unwrap();
     delay.delay_ms(5000);
    }
 ```
 
 the values are then printed on a generic uart interface
