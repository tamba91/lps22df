//! This crate provides a platform-agnostic driver for the ST LPS22DF pressure-temperature sensor.
//! The datasheet and other documentation is available at <https://www.st.com/en/mems-and-sensors/lps22df.html>.
//! This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) traits.
//! Ensure that the hardware abstraction layer of your microcontroller implements the embedded-hal traits.
//!
//! ## Instantiating
//!
//! Create an instance of the driver with the `new_i2c` or `new_spi` associated function, by passing i2c and address instances
//! or an spi (SpiDevice) instance.
//! 
//! ### I2C:
//!
//!```rust
//! use lps22df::{Lps22df, I2CAddress}
//!-
//! let mut sensor = Lps22df::new_i2c(i2c, I2CAddress::Address1).unwrap();
//! ```
//! 
//! i2c instance must implement the I2c trait of embedded-hal. The address is an enum variant of I2CAddress enum.
//! There are two addresses available: 0x5C and 0x5D, which correspond to enum variants Address0 and Address1.
//! Check the datasheet for I2C address configuration.
//!
//! If multiple sensors are used on the same I2C bus, create an instance of i2c that implements bus sharing:
//! (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
//!```rust
//! use core::cell::RefCell;
//! use embedded_hal_bus::i2c;
//!
//! use lps22df::{Lps22df, I2CAddress as Lps22dfAddress};
//! use stts22h::{Stts22h, I2CAddress as Stts22hAddress}; // the STTS22H is another sensor of ST MEMS family
//!
//! let i2c_bus = RefCell::new(i2c);
//!```
//! and then create an instance of i2c for each sensor connected. 
//!```rust
//! let mut lps22df = Lps22df::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Lps22dfAddress::Address1).unwrap();
//! let mut stts22h = Stts22h::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Stts22hAddress::Address0).unwrap();
//!```
//! 
//! In this example sharing is implemented with a RefCell, so it only allows sharing within a single thread.
//! If you need to share a bus across several threads, use CriticalSectionDevice instead.
//!
//! ### SPI:
//!
//! HALs normally implement SpiBus trait. In order to obtain an SpiDevice from an SpiBus embedded-hal-bus crate can be used.
//! (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
//! ```rust
//! use embedded_hal_bus::spi::RefCellDevice;
//! use lps22df::Lps22df;
//! let spi = RefCellDevice::new_no_delay(&spi, cs).unwrap(); // cs is the chip select pin
//! let mut sensor = Lps22df::new_spi(spi).unwrap();
//! sensor.disable_i2c_interface().unwrap(); // when using spi, the I2C interface can be disabled
//! ```
//! 
//! ## Setting ODR and AVG:
//! Set the output data rate and the resolution with the `set_odr` and the `set_avg` methods.
//! ```rust
//! sensor.set_odr(0).unwrap(); // by passing '0' we put the sensor in power-down/one-shot mode
//! sensor.set_avg(512).unwrap(); // 512 gives the best resolution
//! ```
//! 
//! Start a pressure and temperature sample measurement with the `one_shot` method.
//! Read the measured pressure and temperature with the `get_temp` method.
//! ```rust
//! loop {
//!     sensor.one_shot().unwrap();
//!     let values = sensor.get_values().unwrap();
//!     writeln!(tx, "press: {}, temp: {}", values.0, values.1).unwrap();
//!     delay.delay_ms(5000);
//!    }
//! ```
//! 
//! the values are then printed on a generic uart interface
//!

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

mod lps22df_reg;

///
/// The LPS22DF driver struct.
///
pub struct Lps22df<B: lps22df_reg::BusOperation> {
    bus: B,
}

///
/// Available I2C addresses for the LPS22DF sensor. Check the datasheet for I2C address configuration.
///
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum I2CAddress {
    Address0 = 0x5C,
    Address1 = 0x5D,
}

///
/// Signal Type for INT_DRDY PIN and interrupt status. This enum is used to specify the behavior for the INT_DRDY PIN,
/// when the press data-ready signal to pin is enabled through the method `enable_press_drdy_to_pin`, and to specify the behavior
/// of the interrupt status (and the INT_DRDY PIN, if the pressure interrupt event is propagated to INT_DRDY PIN) through the method
/// `set_press_thr_interrupt_mode`.
///
pub enum SignalMode {
    /// When used in data ready to pin configuration the INT_DRDY PIN is asserted for approximately 5 μs (pulse) when a new pressure 
    /// value is available. After this time, the pin clears itself.
    ///
    /// When used in differential pressure interrupt configuration the differential pressure interrupt remains asserted until 
    /// the condition that triggered the interrupt remains true, or the method `get_press_int_status` is called.
    /// This behavior propagates to the INT_DRDY PIN if the differential pressure interrupt event to pin is enabled through the method
    /// `enable_press_thr_interrupt_to_pin`.
    ///
    Pulsed,
    /// When used in data ready to pin configuration the pin remains asserted until the new pressure value is read 
    /// with the method `get_press` or `get_press_raw` or `get_values` or `get_values_raw`.
    ///
    /// When used in differential pressure interrupt configuration the differential pressure interrupt remains asserted even if 
    /// the condition that triggered the interrupt is no longer true, until the method `get_press_int_status` is called
    /// This behavior propagates to the INT_DRDY PIN if the pressure interrupt signal to pin is enabled through the method
    /// `enable_press_thr_interrupt_to_pin`.
    Latched,
}

///
/// This enum specify the logic of the INT_DRDY PIN
///
pub enum PinLogic {
    /// The INT_DRDY PIN is asserted high
    ActiveHigh,
    /// The INT_DRDY PIN is asserted low
    ActiveLow,
}

///
/// This enum is used as parameter for the method `engage_differential_mode`
///
pub enum DifferentialMode {
    /// Autozero mode, when this variant is used in the method `engage_differential_mode` the measured pressure value is
    /// used as the reference, and stored. From this point on, the output pressure value obtained with the methods
    /// `get_press`, `get_press_raw`, `get_values`, `get_values_raw` is the difference between the measured pressure
    /// and the stored pressure.
    Autozero,
    /// Autorefp mode, when this variant is used in the method `engage_differential_mode` the measured pressure value is
    /// used as the reference, and stored. With this variant the output pressure value obtained with the methods
    /// `get_press`, `get_press_raw`, `get_values`, `get_values_raw` is not affected
    Autorefp,
}

///
/// A variant of this enum is returned by the method `get_press_int_status`.
///
pub enum DifferentialPressEvent {
    /// No pressure interrupt occurred.
    NoInterrupt,
    /// A pressure low interrupt occurred (the measured pressure is lower than the threshold pressure).
    PressureLow,
    /// A pressure high interrupt occurred (the measured pressure is higher than the threshold pressure).
    PressureHigh,
    /// A pressure low interrupt and a pressure high interrupt occurred.
    /// (the measuered pressure rised above the threshold and felt below the threshold before the method `get_press_int_status` was called)
    BothInterrupt,
}

///
/// A variant of this enum is passed as parameter to the function `set_fifo_mode` to configure the FIFO
///
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum FifoMode {
    /// The FIFO is not operational and it remains empty, switching to bypass mode is also used to reset the FIFO.
    /// Passing through bypass mode is mandatory when switching between different FIFO buffer operating modes.
    Bypass = 0b000,
    /// In FIFOMode pressure data are stored in the FIFO until it is full. When the FIFO is full in order to restart the FIFO the
    /// FIFO must be set in FifoMode::Bypass and then in FifoMode::FIFOMode again.
    FIFOMode = 0b001,
    /// In Continuous mode when the FIFO is full the oldest samples are overwritten when new pressure samples are produced.
    Continuous = 0b010,
    /// The FIFO is in Bypass mode and switches to FIFOMode when a pressure interrupt event occurs.
    BypassToFifo = 0b101,
    /// The FIFO is in Bypass mode and switches to Continuous when a pressure interrupt event occurs.
    BypassToContinuous = 0b110,
    /// The FIFO is in Continuous mode and switches to FIFOMode when a pressure interrupt event occurs.
    ContinuousToFifo = 0b111,
}

///
/// A variant of this enum is passed as parameter to the function `enable_lpf1_filter` to configure the low pass filter
///
#[repr(u8)]
pub enum Lpf1Conf {
    //LPF1 Filter disabled
    OdrDiv2Lpf0Only = 0b00,
    //LPF1 Filter configuration ODR/4
    OdrDiv4 = 0b10,
    //LPF1 Filter configuration ODR/9
    OdrDiv9 = 0b11,
}

///
/// Driver errors.
///
#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    /// An error occurred at the bus level. Any methods that access the I2C/SPI bus to interact with the sensor may return this error
    /// if the bus operation fails.
    /// The generic type B represents the specific error generated by the HAL of the microcontroller in use.
    Bus(B),
    /// The attempt to write to a register failed,
    /// resulting in a discrepancy between the intended value and the actual value stored in the register.
    WriteFailure,
    /// The `who_am_i` method returned an incorrect sensor identifier (the LPS22DF identifier is 0xB4).
    WhoAmIError(u8),
    /// A register contains an invalid value.
    InvalidRegisterValue,
    /// A one-shot measurement is requested through the `one_shot` method, but the sensor is not in power-down/one-shot mode.
    PowerDownOneShotModeNotEnabled,
    /// A read action on the FIFO or the FIFO status was requested, but the FIFO is in Bypass mode (turned off).
    BypassModeEnabled,
    /// A configuration action on the FIFO was attempted, but the FIFO is not in Bypass mode.
    BypassModeNotEnabled,
    /// The method `is_fifo_full` was called, but the FIFO is resized to the watermark level.
    /// Use the method `is_watermark_full` instead.
    WatermarkEnabled,
    /// An invalid watermark level was passed to the `method set_fifo_watermark`.
    InvalidWatermarkLevel,
}

impl<P> Lps22df<lps22df_reg::Lps22dfI2C<P>>
where
    P: I2c,
{
    ///
    /// Constructor method (associated function) for using the I2C bus. This method checks for the presence of the sensor on the bus
    /// and returns a new driver instance if the sensor responds with the correct identifier.
    ///
    /// # Arguments
    ///
    /// * `i2c` - an I2C peripheral instance.
    /// * `address` - an I2C address enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * Self: The sensor driver instance.
    ///     * Error: If a wrong identifier is received (!= 0xB4) an Error::WhoAmIError(u8) is returned.
    ///              The error contains the wrong number received.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Result<Self, Error<P::Error>> {
        let bus = lps22df_reg::Lps22dfI2C::new(i2c, address as SevenBitAddress);
        let mut instance = Self { bus };
        let who = instance.who_am_i_get()?;
        if who != 0xB4 {
            return Err(Error::WhoAmIError(who));
        }
        instance.ctrl_reg2_set_boot()?;
        while instance.int_source_get_boot_on()? != 0 {}
        instance.ctrl_reg2_set_swreset()?;
        while instance.ctrl_reg2_get_swreset()? != 0 {}

        Ok(instance)
    }
}

impl<P> Lps22df<lps22df_reg::Lps22dfSPI<P>>
where
    P: SpiDevice,
{
    ///
    /// Constructor method (associated function) for using the SPI bus. This method checks for the presence of the sensor on the bus
    /// and returns a new driver instance if the sensor responds with the correct identifier.
    ///
    /// # Arguments
    ///
    /// * `spi` - an SPI peripheral instance.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * Self: The sensor driver instance.
    ///     * Error: If a wrong identifier is received (!= 0xB4) an Error::WhoAmIError(u8) is returned.
    ///              The error contains the wrong number received.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn new_spi(spi: P) -> Result<Self, Error<P::Error>> {
        let bus = lps22df_reg::Lps22dfSPI::new(spi);
        let mut instance = Self { bus };
        let who = instance.who_am_i_get()?;
        if who != 0xB4 {
            return Err(Error::WhoAmIError(who));
        }
        instance.ctrl_reg2_set_boot()?;
        while instance.int_source_get_boot_on()? != 0 {}
        instance.ctrl_reg2_set_swreset()?;
        while instance.ctrl_reg2_get_swreset()? != 0 {}

        Ok(instance)
    }

    ///
    /// Method that disables the I2C interface. It's avaialable only when the sensor is connected through SPI bus.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_i2c_interface(&mut self) -> Result<(), Error<P::Error>> {
        self.if_ctrl_set_i2c_i3c_dis(true as u8)?;

        Ok(())
    }

    ///
    /// Method that enables the I2C interface. It's avaialable only when the sensor is connected through SPI bus.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_i2c_interface(&mut self) -> Result<(), Error<P::Error>> {
        self.if_ctrl_set_i2c_i3c_dis(false as u8)?;

        Ok(())
    }
}

impl<B: lps22df_reg::BusOperation> Lps22df<B> {
    ///
    /// Method that returns the sensor identifier (0xB4).
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * u8: The sensor identifier number (0xA0).
    ///     * Error: If a wrong identifier is received (!= 0xB4) an Error::WhoAmIError(u8) is returned.
    ///              The error contains the wrong number received.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    ///
    /// ```rust
    ///  let who = sensor.who_am_i().unwrap();
    ///  writeln!(tx, "who am i: {}", who).unwrap();
    /// ```
    /// 
    pub fn who_am_i(&mut self) -> Result<u8, Error<B::Error>> {
        let res = self.who_am_i_get()?;

        if res != 0xB4 {
            return Err(Error::WhoAmIError(res));
        }

        Ok(res)
    }

    ///
    /// Method that returns the current ODR (output data rate).
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * u32: The sensor current ODR (output data rate). If 0 is returned the sensor is in power-down/one-shot mode.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///              
    pub fn get_odr(&mut self) -> Result<u32, Error<B::Error>> {
        let odr: lps22df_reg::CtrlReg1Odr = self.ctrl_reg1_get_odr()?.into();

        Ok(odr.into())
    }

    ///
    /// Method that sets the ODR (output data rate).
    ///     
    /// # Arguments
    ///
    /// * odr: an u32 unsigned integer number representing the desired odr.
    ///        The available ODRs are:
    ///        0: sets the sensor in power-down/one-shot mode (see the `one_shot` method documentation), 1Hz 4Hz, 10Hz, 25Hz,
    ///        50Hz, 75Hz (max avg in continuous mode: 128), 100Hz (max avg in continuous mode: 64), 
    ///        200Hz (max avg in continuous mode: 32).
    ///                           
    /// # Note
    ///
    /// Passing an ODR value from the list will set the sensor to this exact ODR.
    /// If an ODR value outside the list is passed, it will be rounded to the next greater value.
    /// If a value greater than 200 is passed, the ODR will be rounded to 200.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 1
    ///   
    /// ```rust
    /// sensor.set_odr(10).unwrap(); // the odr is set at 10 Hz
    /// let current_odr = sensor.get_odr().unwrap(); // current_odr is 10
    /// ```
    ///
    /// # Example 2
    ///   
    /// ```rust
    /// sensor.set_odr(22).unwrap(); // 22 is not a value in the list, the odr is set at 25 Hz
    /// let current_odr = sensor.get_odr().unwrap(); // current_odr is 25
    /// ```
    ///
    /// # Example 3
    ///   
    /// ```rust
    /// sensor.set_odr(0).unwrap(); // the sensor is put in power-down/one-shot mode
    /// let current_odr = sensor.get_odr().unwrap(); // current_odr is 0
    /// ```
    /// 
    pub fn set_odr(&mut self, odr: u32) -> Result<(), Error<B::Error>> {
        let odr: lps22df_reg::CtrlReg1Odr = odr.into();
        self.ctrl_reg2_set_bdu(false as u8)?;
        self.ctrl_reg1_set_odr(odr as u8)?;

        Ok(())
    }

    ///
    /// Method that returns the current AVG (resolution).
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * u32: The sensor current AVG (resolution).
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///              
    pub fn get_avg(&mut self) -> Result<u32, Error<B::Error>> {
        let avg: lps22df_reg::CtrlReg1Avg = self.ctrl_reg1_get_avg()?.try_into()?;

        Ok(avg.into())
    }

    ///
    /// Method that sets the AVG (resolution).
    ///     
    /// # Arguments
    ///
    /// * avg: an u32 unsigned integer number representing the desired resolution (averages).
    ///        The available AVGs are: 4, 8, 16, 32, 64 (max odr in continuous mode: 100 Hz), 128 (max odr in continuous mode: 75 Hz),
    ///        512 (max odr in continuous mode: 25 Hz)
    ///                 
    /// # Note
    ///
    /// Passing an AVG value from the list will set the sensor to this exact AVG.
    /// If an AVG value outside the list is passed, it will be rounded to the next greater value.
    /// If a value greater than 512 is passed, the AVG will be rounded to 512.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 1
    ///   
    /// ```rust
    /// sensor.set_avg(16).unwrap(); // the avg is set at 16 averages
    /// let current_avg = sensor.get_avg().unwrap(); // current_avg is 16
    /// ```
    ///
    /// # Example 2
    ///   
    /// ```rust
    /// sensor.set_avg(30).unwrap(); // 30 is not a value in the list, the avg is set at 32 averages
    /// let current_avg = sensor.get_avg().unwrap(); // current_avg is 32
    /// ```
    /// 
    pub fn set_avg(&mut self, avg: u32) -> Result<(), Error<B::Error>> {
        let avg: lps22df_reg::CtrlReg1Avg = avg.into();
        self.ctrl_reg1_set_avg(avg as u8)?;

        Ok(())
    }

    ///
    /// Method that returns true if an unread pressure sample is available.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: if true an unread pressure sample is available, if false no unread pressure sample is available.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    ///   
    /// ```rust
    /// sensor.set_odr(10).unwrap();
    /// sensor.set_avg(512).unwrap();
    /// loop {
    ///    if sensor.is_press_data_avail().unwrap() == true {
    ///        let press: f32 = sensor.get_press().unwrap();
    ///        let press_avail: bool = sensor.is_press_data_avail().unwrap(); // the available sample has been read, press_avail is false
    ///    }
    /// }
    /// ```  
    ///            
    pub fn is_press_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_get_p_da()?;

        Ok(val != 0)
    }

    ///
    /// Method that returns true if an unread temperature sample is available.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: if true an unread temperature sample is available, if false no unread temperature sample is available.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    ///   
    /// ```rust
    /// sensor.set_odr(10).unwrap();
    /// sensor.set_avg(512).unwrap();
    /// loop {
    ///     if sensor.is_temp_data_avail().unwrap() == true {
    ///         let temp: f32 = sensor.get_temp().unwrap();
    ///         let temp_avail: bool = sensor.is_temp_data_avail().unwrap(); // the available sample has been read, temp_avail is false
    ///     }
    /// }
    /// ```  
    ///  
    pub fn is_temp_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_get_t_da()?;

        Ok(val != 0)
    }

    ///
    /// This method turns on the measurement chain.
    /// When the measurement is completed, the device is put in power-down condition, and the data can be read
    /// with the `get_press`, `get_press_raw`, `get_temp`, `get_temp_raw`, `get_values` and `get_values_raw`  methods.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()          
    ///     * Error: If the sensor is not in one-shot mode an Error::PowerDownOneShotModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    /// 
    /// * AVG/ODR max values in one-shot mode:
    ///        * avg 512: max odr 25 Hz
    ///        * avg 128: max odr 75 Hz
    ///        * avg 64: max odr: 100 Hz
    ///        * avg 32: max odr: 200 Hz
    ///        * avg 16: max odr: 300 Hz
    ///        * avg 8: max odr: 400 Hz
    ///        * avg 4: max odr: 500 Hz  
    /// 
    /// # Note:
    ///   In one-shot mode, the ODR (output data rate) corresponds to the frequency at which the `one_shot` method is called.
    /// 
    /// # Example
    ///   
    /// ```rust
    /// sensor.set_odr(0).unwrap();
    /// sensor.set_avg(512).unwrap();
    /// loop {
    ///     sensor.one_shot().unwrap();
    ///     let press: f32 = sensor.get_press().unwrap();
    ///     delay.delay_ms(5000);
    /// }
    /// ``` 
    ///     
    pub fn one_shot(&mut self) -> Result<(), Error<B::Error>> {
        if let lps22df_reg::CtrlReg1Odr::PowerDownOneShot = self.ctrl_reg1_get_odr()?.into() {
            self.press_out_h_get()?;
            self.ctrl_reg2_set_oneshot()?;
            while self.status_get_p_da()? == 0 {}

            return Ok(());
        } else {
            return Err(Error::PowerDownOneShotModeNotEnabled);
        }
    }

    ///
    /// Method that returns the temperature value.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: temperature in °C. The value is expressed as 32-bit floating point.           
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn get_temp(&mut self) -> Result<f32, Error<B::Error>> {
        let raw_temp = self.temp_out_l_h_get()?;
        let temp = raw_temp as f32 / TEMP_SENSITIVITY;

        Ok(temp)
    }

    ///
    /// Method that returns the raw temperature value.
    /// This method avoids floating point division.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw temperature. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real temperature in °C divide by 100.0 (or the TEMP_SENSITIVITY const),
    ///            e.g. 2500 is 25.00 °C, -1000 is -10.00 °C.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///   
    pub fn get_temp_raw(&mut self) -> Result<i16, Error<B::Error>> {
        let raw_temp = self.temp_out_l_h_get()?;

        Ok(raw_temp)
    }

    ///
    /// Method that returns the pressure value.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: pressure in hPa. The value is expressed as 32-bit floating point.           
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///    
    pub fn get_press(&mut self) -> Result<f32, Error<B::Error>> {
        let raw_press = self.press_out_xl_l_h_get()?;
        let press = raw_press as f32 / PRESS_SENSITIVITY;

        Ok(press)
    }

    ///
    /// Method that returns the raw pressure value.
    /// This method avoids floating point division.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i32: raw pressure. The value is expressed as two’s complement 32-bit integer.
    ///            To obtain the real pressure in hPa divide by 4096.0 (or the PRESS_SENSITIVITY const).
    ///            e.g. 4191629 is 1023.3 hPa.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    /// 
    pub fn get_press_raw(&mut self) -> Result<i32, Error<B::Error>> {
        let raw_press = self.press_out_xl_l_h_get()?;

        Ok(raw_press)
    }

    ///
    /// Method that returns the pressure and temperature values.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * (f32, f32): a tuple containing pressure and temperaure. The values are expressed as 32-bit floating point.           
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    /// 
    /// ```rust
    /// sensor.set_odr(0).unwrap();
    /// sensor.set_avg(512).unwrap();
    /// loop {
    ///     sensor.one_shot().unwrap();
    ///     let values: (f32, f32) = sensor.get_values().unwrap();
    ///     let press: f32 = values.0;
    ///     let temp: f32 = values.1;
    ///     delay.delay_ms(5000);
    /// }
    /// ```
    ///   
    pub fn get_values(&mut self) -> Result<(f32, f32), Error<B::Error>> {
        let raw_values = self.press_out_xl_l_h_temp_out_l_h_get()?;
        let press = raw_values.0 as f32 / PRESS_SENSITIVITY;
        let temp = raw_values.1 as f32 / TEMP_SENSITIVITY;

        Ok((press, temp))
    }

    ///
    /// Method that returns the raw pressure and temperature values.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * (i32, i16): a tuple containing raw pressure and raw temperaure. The values expressed as 32-bit 16-bit signed integers;
    ///       see the methods `get_press_raw` and `get_temp_raw`.          
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    /// 
    /// ```rust
    /// sensor.set_odr(0).unwrap();
    /// sensor.set_avg(512).unwrap();
    /// loop {
    ///     sensor.one_shot().unwrap();
    ///     let raw_values: (i32, i16) = sensor.get_values_raw().unwrap();
    ///     let raw_press: i32 = raw_values.0;
    ///     let raw_temp: i16 = raw_values.1;
    ///     delay.delay_ms(5000);
    /// }
    /// ```
    ///
    pub fn get_values_raw(&mut self) -> Result<(i32, i16), Error<B::Error>> {
        let raw_values = self.press_out_xl_l_h_temp_out_l_h_get()?;

        Ok(raw_values)
    }

    ///
    /// Method that selects the INT_DRDY PIN logic.
    ///     
    /// # Arguments
    ///
    /// * pin_logic: a PinLogic enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()         
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn set_pin_logic(&mut self, pin_logic: PinLogic) -> Result<(), Error<B::Error>> {
        match pin_logic {
            PinLogic::ActiveHigh => self.ctrl_reg3_set_int_h_l(false as u8)?,
            PinLogic::ActiveLow => self.ctrl_reg3_set_int_h_l(true as u8)?,
        }

        Ok(())
    }
    
    ///
    /// Method that enables the data ready signal on the INT_DRDY pin.
    ///     
    /// # Arguments
    ///
    /// * signal_mode: a SignalMode enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: If true an unread pressure sample is already available when the data ready signal to INT_DRDY PIN is enabled.
    ///             If false no unread pressure sample is available when the data ready signal to INT_DRDY PIN is enabled.    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    /// 
    /// ```rust
    /// sensor.set_odr(10).unwrap();
    /// sensor.set_avg(512).unwrap();
    /// sensor.set_pin_logic(PinLogic::ActiveHigh).unwrap();    // INT_DRDY PIN asserted high
    /// sensor.enable_press_drdy_to_pin(SignalMode::Pulsed).unwrap();   // INT_DRDY PIN in pulsed mode, pulse width around 5 μs 
    /// ```
    ///   
    pub fn enable_press_drdy_to_pin(
        &mut self,
        signal_mode: SignalMode,
    ) -> Result<bool, Error<B::Error>> {
        let val = self.press_out_h_get()?;
        match signal_mode {
            SignalMode::Pulsed => self.ctrl_reg4_set_drdy_pulsed(true as u8)?,
            SignalMode::Latched => self.ctrl_reg4_set_drdy_pulsed(false as u8)?,
        }
        self.ctrl_reg4_set_drdy(true as u8)?;

        Ok(val != 0)
    }

    ///
    /// Method that disables the data ready signal on the INT_DRDY pin.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()     
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_press_drdy_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_drdy(false as u8)?;
        self.ctrl_reg4_set_drdy_pulsed(false as u8)?;

        Ok(())
    }

    ///
    /// Method that enables the pressure interrupt signal on the INT_DRDY pin.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * DifferentialPressEvent: enum variant the represents the differential pressure event interrupt status
    ///       when the method is called. 
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example
    /// This example demonstrates how to enable a differential pressure event interrupt and propagate it to the INT_DRDY PIN, 
    /// generating a signal on the pin when the measured pressure falls below the stored reference pressure minus a threshold.
    /// In the example, the current pressure is sampled and stored using the `engage_differential_mode` method.
    /// A pressure threshold of 10.0 hPa is set using the `set_press_threshold` method.
    /// The pressure low event is enabled using the `enable_press_low_event` method.
    /// When the measured pressure falls below the stored pressure value minus the threshold 
    /// (measured_pressure < stored_pressure_reference - threshold), the interrupt is triggered.
    /// 
    /// ```rust
    /// sensor.set_pin_logic(PinLogic::ActiveLow).unwrap(); // INT_DRDY PIN is asserted low
    /// sensor.set_odr(10).unwrap(); 
    /// sensor.set_avg(512).unwrap(); 
    /// sensor.engage_differential_mode(DifferentialMode::Autorefp).unwrap(); // sampling and storing the current pressure value (pressure reference), Autorefp mode
    /// sensor.set_press_threshold(10.0).unwrap(); // threshold value for pressure interrupt generation of 10.0 hPa
    /// sensor.set_press_thr_interrupt_mode(SignalMode::Pulsed).unwrap(); // differential pressure event interrupt pulsed
    /// sensor.enable_press_low_event().unwrap(); // pressure low event enabled
    /// sensor.enable_press_thr_interrupt_to_pin().unwrap(); // differential pressure event interrupt propagated to INT_DRDY PIN
    /// ```
    /// 
    pub fn enable_press_thr_interrupt_to_pin(&mut self) -> Result<DifferentialPressEvent, Error<B::Error>> {
        let val = self.int_source_get_pl_ph()?;
        self.ctrl_reg4_set_int_en(true as u8)?;

        Ok(val.into())
    }

    ///
    /// Method that disables the pressure interrupt signal on the INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_press_thr_interrupt_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_en(false as u8)?;

        Ok(())
    }

    ///
    /// Method that configure the pressure interrupt status behavior.
    ///     
    /// # Arguments
    ///
    /// * signal_mode: a SignalMode enum variant; see SignalMode enum documentation.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn set_press_thr_interrupt_mode(
        &mut self,
        signal_mode: SignalMode,
    ) -> Result<(), Error<B::Error>> {
        match signal_mode {
            SignalMode::Pulsed => self.interrupt_cfg_set_lir(false as u8)?,
            SignalMode::Latched => self.interrupt_cfg_set_lir(true as u8)?,
        }

        Ok(())
    }

    ///
    /// Method that enables the pressure low event interrupt.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_press_low_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_ple(true as u8)?;

        Ok(())
    }

    ///
    /// Method that disables the pressure low event interrupt.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_press_low_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_ple(false as u8)?;

        Ok(())
    }

    ///
    /// Method that enables the pressure high event interrupt.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_press_high_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_phe(true as u8)?;

        Ok(())
    }

    ///
    /// Method that disables the pressure high event interrupt.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_press_high_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_phe(false as u8)?;

        Ok(())
    }

    ///
    /// This method stores the current pressure value to be used together with the threshold in order to generate differential
    /// pressure interrupts.
    ///     
    /// # Arguments
    ///
    /// * differential_mode: a DifferentialMode enum variant; see the DifferentialMode enum documentation.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn engage_differential_mode(
        &mut self,
        differential_mode: DifferentialMode,
    ) -> Result<(), Error<B::Error>> {
        match differential_mode {
            DifferentialMode::Autozero => {
                self.interrupt_cfg_set_reset_arp()?;
                self.interrupt_cfg_set_autozero()?;
            }
            DifferentialMode::Autorefp => {
                self.interrupt_cfg_set_reset_az()?;
                self.interrupt_cfg_set_autorefp()?;
            }
        }

        Ok(())
    }

    ///
    /// This method disengages the differential mode, deleting the stored pressure value. 
    /// If Autozero mode was enabled, it is disabled.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disengage_differential_mode(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_reset_arp()?;
        self.interrupt_cfg_set_reset_az()?;

        Ok(())
    }

    ///
    /// This method sets the pressure threshold value to be used together with the stored pressure value
    /// in order to generate differential pressure interrupts.
    ///     
    /// # Arguments
    ///
    /// * h_pa: a f32 pressure value in hPa.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()   
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn set_press_threshold(&mut self, h_pa: f32) -> Result<(), Error<B::Error>> {
        let abs_h_pa: f32 = f32::from_bits(h_pa.to_bits() & (i32::MAX as u32));
        let threshold: [u8; 2] = ((abs_h_pa * 16.0) as u16).to_le_bytes();
        self.ths_p_l_set(threshold[0])?;
        self.ths_p_h_set(threshold[1] & 0x7F)?;

        Ok(())
    }

    ///
    /// This method returns the differential pressure interrupt status.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * DifferentialPressEvent: The current differentail pressure interrupt status; see DifferentialPressEvent enum documentation.    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_press_int_status(&mut self) -> Result<DifferentialPressEvent, Error<B::Error>> {
        let val = self.int_source_get_pl_ph()?;

        Ok(val.into())
    }

    ///
    /// This method enables the FIFO FULL signal to INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result    
    ///     * Error: If the FIFO is not in bypass mode Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_fifo_full_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_full(true as u8)?;
            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method disables the FIFO FULL signal to INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result    
    ///     * Error: If the FIFO is not in bypass mode Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_fifo_full_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_full(false as u8)?;
            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method enables the FIFO WATERMARK signal to INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result    
    ///     * Error: If the FIFO is not in bypass mode Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_fifo_watermark_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_wtm(true as u8)?;
            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method disables the FIFO WATERMARK signal to INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result    
    ///     * Error: If the FIFO is not in bypass mode Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_fifo_watermark_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_wtm(false as u8)?;
            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method enables the FIFO OVERRUN signal to INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result    
    ///     * Error: If the FIFO is not in bypass mode Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_fifo_overrun_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_ovr(true as u8)?;
            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method disables the FIFO OVERRUN signal to INT_DRDY PIN.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result    
    ///     * Error: If the FIFO is not in bypass mode Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_fifo_overrun_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_ovr(false as u8)?;
            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method sets the FIFO mode.
    ///     
    /// # Arguments
    ///
    /// * mode: a FifoMode enum variant; see the FifoMode enum documentation.
    /// 
    /// # Returns
    ///
    /// * Result 
    ///     * ()
    ///     * Error: If a transition from non Bypass mode to another non Bypass mode is attempted 
    ///              an Error::BypassModeNotEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    /// 
    /// # Example
    /// In this example the FIFO is configured to start collecting data when a low pressure interrupt event is triggered.
    /// ```rust
    /// sensor.set_pin_logic(PinLogic::ActiveLow).unwrap(); // INT_DRDY PIN asserted low
    /// sensor.set_odr(10).unwrap();
    /// sensor.set_avg(512).unwrap(); 
    /// sensor.set_press_threshold(10.0).unwrap(); // pressure threshold of 10.0 hPa
    /// sensor.engage_differential_mode(DifferentialMode::Autorefp).unwrap(); //sampling and storing the current pressure as reference 
    /// sensor.set_press_thr_interrupt_mode(SignalMode::Pulsed).unwrap(); // dfferential pressure interrupt in pulsed mode
    /// sensor.set_fifo_watermark(true, 50).unwrap(); // trimming the FIFO to 50 samples max
    /// sensor.enable_fifo_watermark_to_pin().unwrap(); // when watermark threshold is reached INT_DRDY PIN is asserted
    /// sensor.set_fifo_mode(FifoMode::BypassToFifo).unwrap(); // when the interrupt is triggered the FIFO starts storing samples
    /// sensor.enable_press_low_event().unwrap(); // enable press low event, if pressure < (stored_pressure - threshold) the interrupt is triggered
    /// let mut buf: [Option<f32>; 50] = [None; 50]; // buffer array 
    /// /* ...  */ // waiting for the watermark threshold event on INT_DRDY PIN ...
    /// if sensor.is_watermark_full().unwrap() { // checking that the watermark threshold event occurred
    ///     sensor.read_fifo(&mut buf).unwrap(); // reading and (emptying) the FIFO
    ///     sensor.set_fifo_mode(FifoMode::Bypass).unwrap(); // resetting the FIFO
    ///     sensor.set_fifo_mode(FifoMode::BypassToFifo).unwrap(); // setting the FIFO in BypassToFifo mode again
    /// }
    /// ```
    /// 
    pub fn set_fifo_mode(&mut self, mode: FifoMode) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = mode {
            self.fifo_ctrl_set_trig_modes_f_mode(mode as u8)?;

            return Ok(());
        }
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.fifo_ctrl_set_trig_modes_f_mode(mode as u8)?;

            return Ok(());
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method sets the watermark threshold value .
    ///     
    /// # Arguments
    ///
    /// * enable_size_fifo_to_watermark: if true the FIFO size is trimmed to the watermark level, if false the size of the FIFO
    ///                                  is kept at 128 samples.
    /// * watermark_level: number of samples for the watermark level threshold. If 0 and enable_size_fifo_to_watermark is false
    ///                    the watermark threshold is disabled, if 0 and enable_size_fifo_to_watermark is true an error is returned.
    /// 
    /// # Returns
    ///
    /// * Result 
    ///     * ()
    ///     * Error: An invalid watermark configuration returns Err(Error::InvalidWatermarkLevel).
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 1
    /// In this example the FIFO size is trimmed to the watermark level (50 samples). The method `is_watermark_full`
    /// will return true when 50 samples are stored in the FIFO and the FIFO will stop storing data.
    /// ```rust
    /// sensor.set_fifo_watermark(true, 50).unwrap();
    /// sensor.set_fifo_mode(FifoMode::FIFOMode).unwrap();
    /// ```
    /// 
    /// # Example 2
    /// In this example the FIFO size is not trimmed to the watermark level (120 samples). The method `is_watermark_full`
    /// will return true when 120 samples are stored in the FIFO but the FIFO will keep storing data up to 128 samples.
    /// ```rust
    /// sensor.set_fifo_watermark(false, 120).unwrap();
    /// sensor.set_fifo_mode(FifoMode::FIFOMode).unwrap();
    /// ```
    ///
    /// # Example 3
    /// In this example the watermark level is disabled
    /// ```rust
    /// sensor.set_fifo_watermark(false, 0).unwrap();
    /// ```
    ///
    pub fn set_fifo_watermark(
        &mut self,
        enable_size_fifo_to_watermark: bool,
        watermark_level: u8,
    ) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            match enable_size_fifo_to_watermark {
                true => match watermark_level {
                    0 => return Err(Error::InvalidWatermarkLevel),
                    1..=127 => {
                        self.fifo_ctrl_set_stop_on_wtm(true as u8)?;
                        self.fifo_wtm_set(watermark_level)?;
                        return Ok(());
                    }
                    128.. => return Err(Error::InvalidWatermarkLevel),
                },
                false => match watermark_level {
                    0 => {
                        self.fifo_ctrl_set_stop_on_wtm(false as u8)?;
                        self.fifo_wtm_set(0)?;
                        return Ok(());
                    }
                    1..=127 => {
                        self.fifo_ctrl_set_stop_on_wtm(false as u8)?;
                        self.fifo_wtm_set(watermark_level)?;
                        return Ok(());
                    }
                    128.. => return Err(Error::InvalidWatermarkLevel),
                },
            }
        } else {
            return Err(Error::BypassModeNotEnabled);
        }
    }

    ///
    /// This method returns the number of unread data samples stored in the FIFO.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result 
    ///     * u32: the number of unread data samples stored in the FIFO
    ///     * Error: If the FIFO is in Bypass mode an Error::BypassModeEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_fifo_data_length(&mut self) -> Result<u32, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::BypassModeEnabled);
        }
        let val: u32 = self.fifo_status1_get()? as u32;

        Ok(val)
    }

    ///
    /// This method returns the number of unread data samples stored in the FIFO is equal or greater than the watermark threshold.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result 
    ///     * bool: If true the number of unread data samples stored in the FIFO is equal or greater than the watermark threshold.
    ///     * Error: If the FIFO is in Bypass mode an Error::BypassModeEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn is_watermark_full(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::BypassModeEnabled);
        }
        let res = self.fifo_status2_get_fifo_wtm_ia()?;

        Ok(res != 0)
    }

    ///
    /// This method returns true if the fifo is full (128 unread samples)
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result 
    ///     * bool: If true the fifo is full
    ///     * Error: If the FIFO is in Bypass mode an Error::BypassModeEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn is_fifo_full(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::BypassModeEnabled);
        }
        if let true = self.fifo_ctrl_get_stop_on_wtm()? != 0 {
            return Err(Error::WatermarkEnabled);
        }
        let res = self.fifo_status2_get_fifo_full_ia()?;

        Ok(res != 0)
    }

    ///
    /// This method returns true if at least one data sample in FIFO has been overwritten (in Continuous mode).
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result 
    ///     * bool: If true at least one data sample in FIFO has been overwritten
    ///     * Error: If the FIFO is in Bypass mode an Error::BypassModeEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn is_fifo_overrun(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::BypassModeEnabled);
        }
        let res = self.fifo_status2_get_fifo_ovr_ia()?;

        Ok(res != 0)
    }

    ///
    /// This method reads the data samples stored in the FIFO into the provided buffer array. If the array is larger then the current
    /// number of samples stored in FIFO will be filled with None .
    ///     
    /// # Arguments
    ///
    /// * buffer: a mutable reference to an array of `Option<i32>`.
    ///
    /// # Returns
    ///
    /// * Result 
    ///     * ()
    ///     * Error: If the FIFO is in bypass mode Error::BypassModeEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    /// 
    /// # Example
    /// ```rust
    /// let mut buf: [Option<f32>; 128] = [None; 128]; // buffer array, slightly larger than watermark level, FIFO size
    /// sensor.set_fifo_watermark(false, 125).unwrap(); // enabling watermark event when FIFO reaches 125 samples (FIFO size not reduced)
    /// sensor.enable_fifo_watermark_to_pin().unwrap(); // watermark signal to INT_DRDY PIN enabled
    /// sensor.set_fifo_mode(FifoMode::Continuous).unwrap(); // FIFO running in Continuous mode
    /// /* ...  */ // waiting for the watermark threshold event
    /// if sensor.is_watermark_full().unwrap() {    // checking that the watermark threshold event occurred
    ///     sensor.read_fifo(&mut buf).unwrap(); // reading and (emptying) the FIFO
    /// }
    /// ``` 
    ///             
    pub fn read_fifo(&mut self, buffer: &mut [Option<f32>]) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::BypassModeEnabled);
        }

        let min_length = core::cmp::min(self.fifo_status1_get()? as usize, buffer.len());

        for i in 0..min_length {
            let raw = self.fifo_data_out_press_xl_l_h_get()?;
            buffer[i] = Some(raw as f32 / PRESS_SENSITIVITY);
        }

        for i in min_length..buffer.len() {
            buffer[i] = None;
        }

        Ok(())
    }

    ///
    /// This method reads (as raw data) the data samples stored in the FIFO into the provided buffer array. 
    /// If the array is larger than the current number of samples stored in the FIFO, the extra elements will be filled with `None`.
    ///     
    /// # Arguments
    ///
    /// * buffer: a mutable reference to an array of `Option<i32>`.
    ///
    /// # Returns
    ///
    /// * Result 
    ///     * ()
    ///     * Error: If the FIFO is in bypass mode Error::BypassModeEnabled is returned.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn read_fifo_raw(&mut self, buffer: &mut [Option<i32>]) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::BypassModeEnabled);
        }

        let min_length = core::cmp::min(self.fifo_status1_get()? as usize, buffer.len());

        for i in 0..min_length {
            let raw = self.fifo_data_out_press_xl_l_h_get()?;
            buffer[i] = Some(raw);
        }

        for i in min_length..buffer.len() {
            buffer[i] = None;
        }

        Ok(())
    }

    ///
    /// This method enables the low pass filter 1.
    ///     
    /// # Arguments
    ///
    /// * lpf1_conf: a Lpf1Conf enum variant; see the Lpf1Conf enum documentation.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn enable_lpf1_filter(&mut self, lpf1_conf: Lpf1Conf) -> Result<(), Error<B::Error>> {
        self.ctrl_reg2_set_lfpf_cfg_en_lpfp(lpf1_conf as u8)?;

        Ok(())
    }

    ///
    /// This method disables the low pass filter 1 (default).
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_lpf1_filter(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg2_set_lfpf_cfg_en_lpfp(Lpf1Conf::OdrDiv2Lpf0Only as u8)?;

        Ok(())
    }

    ///
    /// This method resets all the sensor registers and memory content.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn reset(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg2_set_boot()?;
        while self.int_source_get_boot_on()? != 0 {}
        self.ctrl_reg2_set_swreset()?;
        while self.ctrl_reg2_get_swreset()? != 0 {}

        Ok(())
    }
}

/// The constant 4096.0 serves as the divisor when converting a two's complement integer number which represents a raw pressure
/// to obtain the pressure in hPa as a float.
const PRESS_SENSITIVITY: f32 = 4096.0;
/// The constant 100.0 serves as the divisor when converting a two's complement integer number which represents a raw temperature
/// to obtain the temperature in Celsius as a float.
const TEMP_SENSITIVITY: f32 = 100.0;
