use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

mod lps22df_internal;

pub struct Lps22df<B> {
    bus: B,
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

pub trait BusOperation {
    type Error;

    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>;
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_read_bytes(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    WhoAmIError(u8),
    WriteFailure,
    ContinuosModeEnabled,
    PowerDownOneShotModeEnabled,
    FifoNotEnabled,
    WatermarkEnabled,
}

pub enum SignalMode {
    Pulsed,
    Latched,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum FifoMode {
    Bypass = 0b000,
    FIFOMode = 0b001,
    Continuous = 0b010,
    BypassToFifo = 0b101,
    BypassToContinuous = 0b110,
    ContinuousToFifo = 0b111,
}

impl<B: BusOperation> Lps22df<B> {
    pub fn who_am_i(&mut self) -> Result<u8, Error<B::Error>> {
        let res = self.who_am_i_get()?;

        if res != 0xB4 {
            return Err(Error::WhoAmIError(res));
        }

        Ok(res)
    }

    pub fn get_odr(&mut self) -> Result<u32, Error<B::Error>> {
        let odr: u32 = self.ctrl_reg1_get_odr()?.into();

        Ok(odr)
    }

    pub fn set_odr(&mut self, odr: u32) -> Result<(), Error<B::Error>> {
        let odr: lps22df_internal::CtrlReg1Odr = odr.into();
        self.ctrl_reg1_set_odr(odr)?;

        Ok(())
    }

    pub fn get_avg(&mut self) -> Result<u32, Error<B::Error>> {
        let avg: u32 = self.ctrl_reg1_get_avg()?.into();

        Ok(avg)
    }

    pub fn set_avg(&mut self, avg: u32) -> Result<(), Error<B::Error>> {
        let avg: lps22df_internal::CtrlReg1Avg = avg.into();
        self.ctrl_reg1_set_avg(avg)?;

        Ok(())
    }

    pub fn is_press_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_get_p_da()?;

        Ok(val)
    }

    pub fn is_temp_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_get_t_da()?;

        Ok(val)
    }

    pub fn get_temp(&mut self) -> Result<f32, Error<B::Error>> {
        let raw = self.temp_out_l_h_get()?;
        let val = raw as f32 / 100.0;

        Ok(val)
    }

    pub fn get_press(&mut self) -> Result<f32, Error<B::Error>> {
        let raw = self.press_out_xl_l_h_get()?;
        let val = raw as f32 / 4096.0;

        Ok(val)
    }

    pub fn get_oneshot(&mut self) -> Result<(f32, f32), Error<B::Error>> {
        if self.ctrl_reg1_get_odr()? != lps22df_internal::CtrlReg1Odr::PowerDownOneShot {
            return Err(Error::ContinuosModeEnabled);
        }

        self.ctrl_reg2_set_oneshot()?;

        while self.ctrl_reg2_get_oneshot()? != false {}

        let raw_press = self.press_out_xl_l_h_get()?;
        let raw_temp = self.temp_out_l_h_get()?;

        let press = raw_press as f32 / 4096.0;
        let temp = raw_temp as f32 / 100.0;

        Ok((press, temp))
    }

    pub fn enable_drdy_to_int(
        &mut self,
        signal_mode: SignalMode,
    ) -> Result<(), Error<B::Error>> {
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

    pub fn disable_drdy_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_drdy(false)?;

        Ok(())
    }

    pub fn enable_fifo_full_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_f_full(true)?;

        Ok(())
    } 

    pub fn disable_fifo_full_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_f_full(false)?;

        Ok(())
    }

    pub fn enable_fifo_watermark_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_f_wtm(true)?;

        Ok(())
    }

    pub fn disable_fifo_watermark_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_f_wtm(false)?;

        Ok(())
    }

    pub fn enable_fifo_overwritten_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_f_ovr(true)?;

        Ok(())
    } 

    pub fn disable_fifo_overwritten_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_f_ovr(false)?;

        Ok(())
    }  

    pub fn enable_fifo(
        &mut self,
        mode: FifoMode,
        enable_wtm: bool,
        wtm_level: Option<u8>,
    ) -> Result<(), Error<B::Error>> {
        self.fifo_ctrl_set_trig_modes_f_mode(FifoMode::Bypass)?;
        match wtm_level {
            Some(value) => match value {
                0 => {
                    self.fifo_wtm_set(0)?;
                    self.fifo_ctrl_set_stop_on_wtm(false)?;
                }
                1..=127 => {
                    self.fifo_wtm_set(value)?;
                    self.fifo_ctrl_set_stop_on_wtm(enable_wtm)?;
                }
                128.. => {
                    self.fifo_wtm_set(127)?;
                    self.fifo_ctrl_set_stop_on_wtm(enable_wtm)?;
                }
            },
            None => {
                self.fifo_wtm_set(0)?;
                self.fifo_ctrl_set_stop_on_wtm(false)?;
            }
        }
        self.fifo_ctrl_set_trig_modes_f_mode(mode)?;

        Ok(())
    }

    pub fn disable_fifo(&mut self) -> Result<(), Error<B::Error>> {
        self.fifo_ctrl_set_trig_modes_f_mode(FifoMode::Bypass)?;
        self.fifo_wtm_set(0)?;
        self.fifo_ctrl_set_stop_on_wtm(false)?;

        Ok(())
    }

    pub fn get_fifo_data_length(&mut self) -> Result<u32, Error<B::Error>> {
        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo =
            self.fifo_ctrl_get_trig_modes_f_mode()?
        {
            return Err(Error::FifoNotEnabled);
        }
        let val: u32 = self.fifo_status1_get()? as u32;

        Ok(val)
    }

    pub fn is_watermark_full(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo =
            self.fifo_ctrl_get_trig_modes_f_mode()?
        {
            return Err(Error::FifoNotEnabled);
        }
        let res = self.fifo_status2_get_fifo_wtm_ia()?;

        Ok(res)
    }

    pub fn is_fifo_full(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo =
            self.fifo_ctrl_get_trig_modes_f_mode()?
        {
            return Err(Error::FifoNotEnabled);
        }
        if let true = self.fifo_ctrl_get_stop_on_wtm()? {
            return Err(Error::WatermarkEnabled);
        }
        let res = self.fifo_status2_get_fifo_full_ia()?;

        Ok(res)
    }

    //pub fn is_fifo_full

    pub fn is_fifo_overwritten(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo =
            self.fifo_ctrl_get_trig_modes_f_mode()?
        {
            return Err(Error::FifoNotEnabled);
        }
        let res = self.fifo_status2_get_fifo_ovr_ia()?;

        Ok(res)
    }

    pub fn read_fifo(&mut self, buffer: &mut [Option<f32>]) -> Result<(), Error<B::Error>> {
        let mode = self.fifo_ctrl_get_trig_modes_f_mode()?;

        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo = mode {
            return Err(Error::FifoNotEnabled);
        }

        let min_length = core::cmp::min(self.fifo_status1_get()? as usize, buffer.len());

        for i in 0..min_length {
            let raw = self.fifo_data_out_press_xl_l_h_get()?;
            buffer[i] = Some(raw as f32 / 4096.0);
        }

        for i in min_length..buffer.len() {
            buffer[i] = None;
        }

        if let FifoMode::FIFOMode = mode {
            if let true = self.fifo_status2_get_fifo_wtm_ia()? {
                self.fifo_ctrl_set_trig_modes_f_mode(FifoMode::Bypass)?;
                self.fifo_ctrl_set_trig_modes_f_mode(mode)?;
            }
            if let true = self.fifo_status2_get_fifo_full_ia()? {
                self.fifo_ctrl_set_trig_modes_f_mode(FifoMode::Bypass)?;
                self.fifo_ctrl_set_trig_modes_f_mode(mode)?;
            }
            if let true = self.fifo_status2_get_fifo_ovr_ia()? {
                self.fifo_ctrl_set_trig_modes_f_mode(FifoMode::Bypass)?;
                self.fifo_ctrl_set_trig_modes_f_mode(mode)?;
            }
        }

        Ok(())
    }
}
