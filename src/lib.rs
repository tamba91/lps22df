#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

mod lps22df_reg;

pub struct Lps22df<B: lps22df_reg::BusOperation> {
    bus: B,
}

impl<P> Lps22df<lps22df_reg::Lps22dfI2C<P>>
where
    P: I2c,
{
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

    pub fn disable_i2c_interface(&mut self) -> Result<(), Error<P::Error>> {
        self.if_ctrl_set_i2c_i3c_dis(true as u8)?;

        Ok(())
    }

    pub fn enable_i2c_interface(&mut self) -> Result<(), Error<P::Error>> {
        self.if_ctrl_set_i2c_i3c_dis(false as u8)?;

        Ok(())
    }
}

const PRESS_SENSITIVITY: f32 = 4096.0;
const TEMP_SENSITIVITY: f32 = 100.0;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum I2CAddress {
    Address0 = 0x5C,
    Address1 = 0x5D,
}

#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    Bus(B),
    WhoAmIError(u8),
    WriteFailure,
    UnexpectedAvgValue,
    ContinuosModeEnabled,
    PowerDownOneShotModeEnabled,
    FifoNotEnabled,
    FifoNotInBypassMode,
    WatermarkEnabled,
    InvalidWatermarkValue,
}

pub enum SignalMode {
    Pulsed,
    Latched,
}

pub enum IntLogic {
    ActiveHigh,
    ActiveLow,
}

pub enum DifferentialMode {
    Autozero,
    Autorefp,
}

pub enum DifferentialPressEvent {
    NoInterrupt,
    PressureLow,
    PressureHigh,
    BothInterrupt,
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

#[repr(u8)]
pub enum Lpf1Conf {
    OdrDiv2Lpf0Only = 0b00,
    OdrDiv4 = 0b10,
    OdrDiv9 = 0b11,
}

impl<B: lps22df_reg::BusOperation> Lps22df<B> {
    pub fn who_am_i(&mut self) -> Result<u8, Error<B::Error>> {
        let res = self.who_am_i_get()?;

        if res != 0xB4 {
            return Err(Error::WhoAmIError(res));
        }

        Ok(res)
    }

    pub fn get_odr(&mut self) -> Result<u32, Error<B::Error>> {
        let odr: lps22df_reg::CtrlReg1Odr = self.ctrl_reg1_get_odr()?.into();

        Ok(odr.into())
    }

    pub fn set_odr(&mut self, odr: u32) -> Result<(), Error<B::Error>> {
        let odr: lps22df_reg::CtrlReg1Odr = odr.into();
        self.ctrl_reg2_set_bdu(false as u8)?;
        self.ctrl_reg1_set_odr(odr as u8)?;

        Ok(())
    }

    pub fn get_avg(&mut self) -> Result<u32, Error<B::Error>> {
        let avg: lps22df_reg::CtrlReg1Avg = self.ctrl_reg1_get_avg()?.try_into()?;

        Ok(avg.into())
    }

    pub fn set_avg(&mut self, avg: u32) -> Result<(), Error<B::Error>> {
        let avg: lps22df_reg::CtrlReg1Avg = avg.into();
        self.ctrl_reg1_set_avg(avg as u8)?;

        Ok(())
    }

    pub fn is_press_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_get_p_da()?;

        Ok(val != 0)
    }

    pub fn is_temp_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_get_t_da()?;

        Ok(val != 0)
    }

    pub fn one_shot(&mut self) -> Result<(), Error<B::Error>> {
        if let lps22df_reg::CtrlReg1Odr::PowerDownOneShot = self.ctrl_reg1_get_odr()?.into() {
            self.press_out_h_get()?;
            self.ctrl_reg2_set_oneshot()?;
            while self.status_get_p_da()? == 0 {}

            return Ok(());
        } else {
            return Err(Error::ContinuosModeEnabled);
        }
    }

    pub fn get_temp(&mut self) -> Result<f32, Error<B::Error>> {
        let raw_temp = self.temp_out_l_h_get()?;
        let temp = raw_temp as f32 / TEMP_SENSITIVITY;

        Ok(temp)
    }

    pub fn get_temp_raw(&mut self) -> Result<i16, Error<B::Error>> {
        let raw_temp = self.temp_out_l_h_get()?;

        Ok(raw_temp)
    }

    pub fn get_press(&mut self) -> Result<f32, Error<B::Error>> {
        let raw_press = self.press_out_xl_l_h_get()?;
        let press = raw_press as f32 / PRESS_SENSITIVITY;

        Ok(press)
    }

    pub fn get_press_raw(&mut self) -> Result<i32, Error<B::Error>> {
        let raw_press = self.press_out_xl_l_h_get()?;

        Ok(raw_press)
    }

    pub fn get_values(&mut self) -> Result<(f32, f32), Error<B::Error>> {
        let raw_values = self.press_out_xl_l_h_temp_out_l_h_get()?;
        let press = raw_values.0 as f32 / PRESS_SENSITIVITY;
        let temp = raw_values.1 as f32 / TEMP_SENSITIVITY;

        Ok((press, temp))
    }

    pub fn get_values_raw(&mut self) -> Result<(i32, i16), Error<B::Error>> {
        let raw_values = self.press_out_xl_l_h_temp_out_l_h_get()?;

        Ok(raw_values)
    }

    pub fn set_int_logic(&mut self, int_logic: IntLogic) -> Result<(), Error<B::Error>> {
        match int_logic {
            IntLogic::ActiveHigh => self.ctrl_reg3_set_int_h_l(false as u8)?,
            IntLogic::ActiveLow => self.ctrl_reg3_set_int_h_l(true as u8)?,
        }

        Ok(())
    }

    pub fn enable_press_drdy_to_int(
        &mut self,
        signal_mode: SignalMode,
    ) -> Result<(), Error<B::Error>> {
        self.press_out_h_get()?;
        match signal_mode {
            SignalMode::Pulsed => self.ctrl_reg4_set_drdy_pulsed(true as u8)?,
            SignalMode::Latched => self.ctrl_reg4_set_drdy_pulsed(false as u8)?,
        }
        self.ctrl_reg4_set_drdy(true as u8)?;

        Ok(())
    }

    pub fn enable_press_thr_interrupt_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.int_source_get_pl_ph()?;
        self.ctrl_reg4_set_int_en(true as u8)?;

        Ok(())
    }

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

    pub fn disable_press_thr_interrupt_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_int_en(false as u8)?;

        Ok(())
    }

    pub fn disable_press_drdy_to_int(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg4_set_drdy(false as u8)?;
        self.ctrl_reg4_set_drdy_pulsed(false as u8)?;

        Ok(())
    }

    pub fn enable_press_low_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_ple(true as u8)?;

        Ok(())
    }

    pub fn disable_press_low_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_ple(false as u8)?;

        Ok(())
    }

    pub fn enable_press_high_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_phe(true as u8)?;

        Ok(())
    }

    pub fn disable_press_high_event(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_phe(false as u8)?;

        Ok(())
    }

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

    pub fn disengage_differential_mode(&mut self) -> Result<(), Error<B::Error>> {
        self.interrupt_cfg_set_reset_arp()?;
        self.interrupt_cfg_set_reset_az()?;

        Ok(())
    }

    pub fn set_press_threshold(&mut self, h_pa: f32) -> Result<(), Error<B::Error>> {
        let abs_h_pa: f32 = f32::from_bits(h_pa.to_bits() & (i32::MAX as u32));
        let threshold: [u8; 2] = ((abs_h_pa * 16.0) as u16).to_le_bytes();
        self.ths_p_l_set(threshold[0])?;
        self.ths_p_h_set(threshold[1] & 0x7F)?;

        Ok(())
    }

    pub fn get_press_int_status(&mut self) -> Result<DifferentialPressEvent, Error<B::Error>> {
        let val = self.int_source_get_pl_ph()?;

        Ok(val.into())
    }

    pub fn enable_fifo_full_to_int(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_full(true as u8)?;
            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn disable_fifo_full_to_int(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_full(false as u8)?;
            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn enable_fifo_watermark_to_int(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_wtm(true as u8)?;
            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn disable_fifo_watermark_to_int(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_wtm(false as u8)?;
            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn enable_fifo_overwritten_to_int(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_ovr(true as u8)?;
            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn disable_fifo_overwritten_to_int(&mut self) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.ctrl_reg4_set_int_f_ovr(false as u8)?;
            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn set_fifo_mode(&mut self, mode: FifoMode) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = mode {
            self.fifo_ctrl_set_trig_modes_f_mode(mode as u8)?;

            return Ok(());
        }
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            self.fifo_ctrl_set_trig_modes_f_mode(mode as u8)?;

            return Ok(());
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn set_fifo_watermark(
        &mut self,
        enable_size_fifo_to_watermark: bool,
        watermark_level: u8,
    ) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            match enable_size_fifo_to_watermark {
                true => match watermark_level {
                    0 => return Err(Error::InvalidWatermarkValue),
                    1..=127 => {
                        self.fifo_ctrl_set_stop_on_wtm(true as u8)?;
                        self.fifo_wtm_set(watermark_level)?;
                        return Ok(());
                    }
                    128.. => return Err(Error::InvalidWatermarkValue),
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
                    128.. => return Err(Error::InvalidWatermarkValue),
                },
            }
        } else {
            return Err(Error::FifoNotInBypassMode);
        }
    }

    pub fn get_fifo_data_length(&mut self) -> Result<u32, Error<B::Error>> {
        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo =
            self.fifo_ctrl_get_trig_modes_f_mode()?.into()
        {
            return Err(Error::FifoNotEnabled);
        }
        let val: u32 = self.fifo_status1_get()? as u32;

        Ok(val)
    }

    pub fn is_watermark_full(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::FifoNotEnabled);
        }
        let res = self.fifo_status2_get_fifo_wtm_ia()?;

        Ok(res != 0)
    }

    pub fn is_fifo_full(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::FifoNotEnabled);
        }
        if let true = self.fifo_ctrl_get_stop_on_wtm()? != 0 {
            return Err(Error::WatermarkEnabled);
        }
        let res = self.fifo_status2_get_fifo_full_ia()?;

        Ok(res != 0)
    }

    pub fn is_fifo_overwritten(&mut self) -> Result<bool, Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::FifoNotEnabled);
        }
        let res = self.fifo_status2_get_fifo_ovr_ia()?;

        Ok(res != 0)
    }

    pub fn read_fifo(&mut self, buffer: &mut [Option<f32>]) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass = self.fifo_ctrl_get_trig_modes_f_mode()?.into() {
            return Err(Error::FifoNotEnabled);
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

    pub fn read_fifo_raw(&mut self, buffer: &mut [Option<i32>]) -> Result<(), Error<B::Error>> {
        if let FifoMode::Bypass | FifoMode::BypassToContinuous | FifoMode::BypassToFifo =
            self.fifo_ctrl_get_trig_modes_f_mode()?.into()
        {
            return Err(Error::FifoNotEnabled);
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

    pub fn enable_lpf1_filter(&mut self, lpf1_conf: Lpf1Conf) -> Result<(), Error<B::Error>> {
        self.ctrl_reg2_set_lfpf_cfg_en_lpfp(lpf1_conf as u8)?;

        Ok(())
    }

    pub fn disable_lpf1_filter(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg2_set_lfpf_cfg_en_lpfp(Lpf1Conf::OdrDiv2Lpf0Only as u8)?;

        Ok(())
    }

    pub fn enable_i3c_asf_filters(&mut self) -> Result<(), Error<B::Error>> {
        self.i3c_if_ctrl_add_set_asf_on(true as u8)?;

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error<B::Error>> {
        self.ctrl_reg2_set_boot()?;
        while self.int_source_get_boot_on()? != 0 {}
        self.ctrl_reg2_set_swreset()?;
        while self.ctrl_reg2_get_swreset()? != 0 {}

        Ok(())
    }
}
