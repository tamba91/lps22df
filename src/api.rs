use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

mod lps22df_internal;

pub struct Lps22df<T> {
    bus: T,
}

impl<P: I2c> Lps22df<lps22df_internal::Lps22dfI2C<P>> {
    pub fn new_i2c(i2c: P, address: SevenBitAddress) -> Self {
        let bus = lps22df_internal::Lps22dfI2C::new(i2c, address);
        Self { bus }
    }
}

impl<P: SpiDevice> Lps22df<lps22df_internal::Lps22dfSPI<P>> {
    pub fn new_spi(spi: P) -> Self {
        let bus = lps22df_internal::Lps22dfSPI::new(spi);
        Self { bus }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Lps22dfError<P> {
    I2C(P),
    SPI(P),
    WhoAmIError(u8),
    WriteFailure,
    ContinuosModeEnabled,
}

pub enum SignalMode {
    Pulsed,
    Latched,
}

impl<T: lps22df_internal::BusOperation> Lps22df<T> {
    pub fn who_am_i(&mut self) -> Result<u8, Lps22dfError<T::Error>> {
        let res = self.who_am_i_get()?;

        if res != 0xB4 {
            return Err(Lps22dfError::WhoAmIError(res));
        }

        Ok(res)
    }

    pub fn set_odr(&mut self, odr: u32) -> Result<(), Lps22dfError<T::Error>> {
        let odr: lps22df_internal::CtrlReg1Odr = odr.into();
        self.ctrl_reg1_set_odr(odr)?;

        Ok(())
    }

    pub fn set_avg(&mut self, avg: u32) -> Result<(), Lps22dfError<T::Error>> {
        let avg: lps22df_internal::CtrlReg1Avg = avg.into();
        self.ctrl_reg1_set_avg(avg)?;

        Ok(())
    }

    pub fn is_press_data_avail(&mut self) -> Result<bool, Lps22dfError<T::Error>> {
        let val = self.status_get_p_da()?;

        Ok(val)
    }

    pub fn is_temp_data_avail(&mut self) -> Result<bool, Lps22dfError<T::Error>> {
        let val = self.status_get_t_da()?;

        Ok(val)
    }

    pub fn get_temp(&mut self) -> Result<f32, Lps22dfError<T::Error>> {
        let raw = self.temp_out_l_h_get()?;
        let val = raw as f32 / 100.0;

        Ok(val)
    }

    pub fn get_press(&mut self) -> Result<f32, Lps22dfError<T::Error>> {
        let raw = self.press_out_xl_l_h_get()?;
        let val = raw as f32 / 4096.0;

        Ok(val)
    }

    pub fn get_oneshot(&mut self) -> Result<(f32, f32), Lps22dfError<T::Error>> {
        if self.ctrl_reg1_get_odr()? != lps22df_internal::CtrlReg1Odr::PowerDownOneShot {
            return Err(Lps22dfError::ContinuosModeEnabled);
        }

        self.ctrl_reg2_set_oneshot()?;

        while self.ctrl_reg2_get_oneshot()? != false {}

        let raw_press = self.press_out_xl_l_h_get()?;
        let raw_temp = self.temp_out_l_h_get()?;

        let press = raw_press as f32 / 4096.0;
        let temp = raw_temp as f32 / 100.0;

        Ok((press, temp))
    }

    pub fn enable_drdy(&mut self, signal_mode: SignalMode) -> Result<(), Lps22dfError<T::Error>> {
        match signal_mode {
            SignalMode::Pulsed => self.ctrl_reg4_set_drdy_pulsed(true)?,
            SignalMode::Latched => {
                self.ctrl_reg4_set_drdy_pulsed(false)?;
                self.press_out_h_get()?;
            }
        }
        self.ctrl_reg4_set_drdy(true)?;

        Ok(())
    }

    pub fn disable_drdy(&mut self) -> Result<(), Lps22dfError<T::Error>> {
        self.ctrl_reg4_set_drdy(false)?;

        Ok(())
    }
}
