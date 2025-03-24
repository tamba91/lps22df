use bitfield::bitfield;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::{Operation, SpiDevice};
use generic_bus::BusOperation;

use super::Error;

#[derive(Clone, Copy)]
#[repr(u8)]  
pub(super) enum ReadMask {
    I2c = 0x00,
    Spi = 0x80,
}

impl<P: I2c> super::Lps22dfI2C<P> {
    pub(super) fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

impl<P: SpiDevice> super::Lps22dfSPI<P> {
    pub(super) fn new(spi: P) -> Self {
        Self { spi }
    }
}

impl<P: I2c> BusOperation for super::Lps22dfI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;

        Ok(())
    }

    #[inline]
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }

    #[inline]
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;

        Ok(())
    }
}

impl<P: SpiDevice> BusOperation for super::Lps22dfSPI<P> {
    type Error = P::Error;

    #[inline]
    fn read(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.read(rbuf)?;

        Ok(())
    }

    #[inline]
    fn write(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(wbuf)?;

        Ok(())
    }

    #[inline]
    fn write_read(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [Operation::Write(wbuf), Operation::Read(rbuf)])?;

        Ok(())
    }
}

impl<I: BusOperation> super::Lps22df<I> {
    #[inline]
    fn read_reg(&mut self, reg: Reg, buf: &mut [u8]) -> Result<(), Error<I::Error>> {
        self.interface.write_read(&[reg as u8 | self.read_mask as u8], buf).map_err(Error::Bus)?;

        Ok(())
    }

    #[inline]
    fn write_reg(&mut self, reg: Reg, val: u8) -> Result<(), Error<I::Error>> {
        self.interface.write(&[reg as u8, val]).map_err(Error::Bus)?;
        let mut arr: [u8; 1] = [0];
        self.read_reg(reg, &mut arr)?;
        if arr[0] != val {
            return Err(Error::WriteFailure);
        }

        Ok(())
    }

    #[inline]
    fn write_reg_no_chk(&mut self, reg: Reg, val: u8) -> Result<(), Error<I::Error>> {
        self.interface.write(&[reg as u8, val]).map_err(Error::Bus)?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_autorefp(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_autorefp(1);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_reset_arp(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_reset_arp(1);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_autozero(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_autozero(1);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_reset_az(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_reset_az(1);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_lir(&mut self, lir: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_lir(lir);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_ple(&mut self, ple: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_ple(ple);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn interrupt_cfg_set_phe(&mut self, phe: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::InterruptCfg, &mut arr)?;
        let mut val = InterruptCfg(arr[0]);
        val.set_interrupt_cfg_phe(phe);
        self.write_reg_no_chk(Reg::InterruptCfg, val.interrupt_cfg())?;

        Ok(())
    }

    pub(super) fn ths_p_l_set(&mut self, ths_p_l: u8) -> Result<(), Error<I::Error>> {
        self.write_reg(Reg::ThsPL, ths_p_l)?;

        Ok(())
    }

    pub(super) fn ths_p_h_set(&mut self, ths_p_h: u8) -> Result<(), Error<I::Error>> {
        self.write_reg(Reg::ThsPH, ths_p_h)?;

        Ok(())
    }

    pub(super) fn if_ctrl_set_i2c_i3c_dis(
        &mut self,
        i2c_i3c_dis: u8,
    ) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::IfCtrl, &mut arr)?;
        let mut val = IfCtrl(arr[0]);
        val.set_if_ctrl_i2c_i3c_dis(i2c_i3c_dis);
        self.write_reg(Reg::IfCtrl, val.if_ctrl())?;

        Ok(())
    }

    pub(super) fn who_am_i_get(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::WhoAmI, &mut arr)?;

        Ok(arr[0])
    }

    pub(super) fn ctrl_reg1_get_odr(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg1, &mut arr)?;
        let val: u8 = CtrlReg1(arr[0]).ctrl_reg1_odr();

        Ok(val)
    }

    pub(super) fn ctrl_reg1_set_odr(&mut self, odr: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg1, &mut arr)?;
        let mut val = CtrlReg1(arr[0]);
        val.set_ctrl_reg1_odr(odr);
        self.write_reg(Reg::CtrlReg1, val.ctrl_reg1())?;

        Ok(())
    }

    pub(super) fn ctrl_reg1_get_avg(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg1, &mut arr)?;
        let val: u8 = CtrlReg1(arr[0]).ctrl_reg1_avg();

        Ok(val)
    }

    pub(super) fn ctrl_reg1_set_avg(&mut self, avg: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg1, &mut arr)?;
        let mut val = CtrlReg1(arr[0]);
        val.set_ctrl_reg1_avg(avg);
        self.write_reg(Reg::CtrlReg1, val.ctrl_reg1())?;

        Ok(())
    }

    pub(super) fn ctrl_reg2_set_boot(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg2, &mut arr)?;
        let mut val = CtrlReg2(arr[0]);
        val.set_ctrl_reg2_boot(1);
        self.write_reg(Reg::CtrlReg2, val.ctrl_reg2())?;

        Ok(())
    }

    pub(super) fn ctrl_reg2_set_lfpf_cfg_en_lpfp(
        &mut self,
        lfpf_cfg_en_lpfp: u8,
    ) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg2, &mut arr)?;
        let mut val = CtrlReg2(arr[0]);
        val.set_ctrl_reg2_lfpf_cfg_en_lpfp(lfpf_cfg_en_lpfp);
        self.write_reg(Reg::CtrlReg2, val.ctrl_reg2())?;

        Ok(())
    }

    pub(super) fn ctrl_reg2_set_bdu(&mut self, bdu: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg2, &mut arr)?;
        let mut val = CtrlReg2(arr[0]);
        val.set_ctrl_reg2_bdu(bdu);
        self.write_reg(Reg::CtrlReg2, val.ctrl_reg2())?;

        Ok(())
    }

    pub(super) fn ctrl_reg2_get_swreset(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg2, &mut arr)?;
        let val: u8 = CtrlReg2(arr[0]).ctrl_reg2_swreset();

        Ok(val)
    }

    pub(super) fn ctrl_reg2_set_swreset(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg2, &mut arr)?;
        let mut val = CtrlReg2(arr[0]);
        val.set_ctrl_reg2_swreset(1);
        self.write_reg_no_chk(Reg::CtrlReg2, val.ctrl_reg2())?;

        Ok(())
    }

    pub(super) fn ctrl_reg2_set_oneshot(&mut self) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg2, &mut arr)?;
        let mut val = CtrlReg2(arr[0]);
        val.set_ctrl_reg2_oneshot(1);
        self.write_reg_no_chk(Reg::CtrlReg2, val.ctrl_reg2())?;

        Ok(())
    }

    pub(super) fn ctrl_reg3_set_int_h_l(&mut self, int_h_l: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg3, &mut arr)?;
        let mut val = CtrlReg3(arr[0]);
        val.set_ctrl_reg3_int_h_l(int_h_l);
        self.write_reg(Reg::CtrlReg3, val.ctrl_reg3())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_drdy_pulsed(
        &mut self,
        drdy_pulsed: u8,
    ) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_drdy_pls(drdy_pulsed);
        self.write_reg(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_drdy(&mut self, drdy: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_drdy(drdy);
        self.write_reg(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_int_en(&mut self, int_en: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_int_en(int_en);
        self.write_reg(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_int_f_full(
        &mut self,
        int_f_fool: u8,
    ) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_int_f_full(int_f_fool);
        self.write_reg(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_int_f_wtm(&mut self, int_f_wtm: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_int_f_wtm(int_f_wtm);
        self.write_reg(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_int_f_ovr(&mut self, int_f_ovr: u8) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_int_f_ovr(int_f_ovr);
        self.write_reg(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn fifo_ctrl_set_trig_modes_f_mode(
        &mut self,
        trig_modes_f_mode: u8,
    ) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoCtrl, &mut arr)?;
        let mut val = FifoCtrl(arr[0]);
        val.set_fifo_ctrl_trig_modes_f_mode(trig_modes_f_mode);
        self.write_reg(Reg::FifoCtrl, val.fifo_ctrl())?;

        Ok(())
    }

    pub(super) fn fifo_ctrl_get_trig_modes_f_mode(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoCtrl, &mut arr)?;
        let val: u8 = FifoCtrl(arr[0]).fifo_ctrl_trig_modes_f_mode();

        Ok(val)
    }

    pub(super) fn fifo_ctrl_set_stop_on_wtm(
        &mut self,
        stop_on_wtm: u8,
    ) -> Result<(), Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoCtrl, &mut arr)?;
        let mut val = FifoCtrl(arr[0]);
        val.set_fifo_ctrl_stop_on_wtm(stop_on_wtm);
        self.write_reg(Reg::FifoCtrl, val.fifo_ctrl())?;

        Ok(())
    }

    pub(super) fn fifo_ctrl_get_stop_on_wtm(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoCtrl, &mut arr)?;
        let val = FifoCtrl(arr[0]).fifo_ctrl_stop_on_wtm();

        Ok(val)
    }

    pub(super) fn fifo_wtm_set(&mut self, wtm: u8) -> Result<(), Error<I::Error>> {
        self.write_reg(Reg::FifoWtm, wtm & 0x7F)?;

        Ok(())
    }

    pub(super) fn int_source_get_boot_on(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::IntSource, &mut arr)?;
        let val = IntSource(arr[0]).int_source_boot_on();

        Ok(val)
    }

    pub(super) fn int_source_get_pl_ph(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::IntSource, &mut arr)?;
        let val = IntSource(arr[0]).int_source_pl_ph();

        Ok(val)
    }

    pub(super) fn fifo_status1_get(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoStatus1, &mut arr)?;

        Ok(arr[0])
    }

    pub(super) fn fifo_status2_get_fifo_wtm_ia(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoStatus2, &mut arr)?;
        let val = FifoStatus2(arr[0]).fifo_status2_fifo_wtm_ia();

        Ok(val)
    }

    pub(super) fn fifo_status2_get_fifo_ovr_ia(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoStatus2, &mut arr)?;
        let val = FifoStatus2(arr[0]).fifo_status2_fifo_ovr_ia();

        Ok(val)
    }

    pub(super) fn fifo_status2_get_fifo_full_ia(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::FifoStatus2, &mut arr)?;
        let val = FifoStatus2(arr[0]).fifo_status2_fifo_full_ia();

        Ok(val)
    }

    pub(super) fn status_get_p_da(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::Status, &mut arr)?;
        let val = Status(arr[0]).status_p_da();

        Ok(val)
    }

    pub(super) fn status_get_t_da(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::Status, &mut arr)?;
        let val = Status(arr[0]).status_t_da();

        Ok(val)
    }

    pub(super) fn temp_out_l_h_get(&mut self) -> Result<i16, Error<I::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_reg(Reg::TempOutL, &mut arr)?;
        let raw_temp: i16 = arr[0] as i16 | (arr[1] as i8 as i16) << 8;

        Ok(raw_temp)
    }

    pub(super) fn press_out_xl_l_h_get(&mut self) -> Result<i32, Error<I::Error>> {
        let mut arr: [u8; 3] = [0; 3];
        self.read_reg(Reg::PressOutXl, &mut arr)?;
        let raw_press: i32 = arr[0] as i32 | (arr[1] as i32) << 8 | (arr[2] as i8 as i32) << 16;

        Ok(raw_press)
    }

    pub(super) fn press_out_h_get(&mut self) -> Result<u8, Error<I::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_reg(Reg::PressOutH, &mut arr)?;
        let val: u8 = arr[0];

        Ok(val)
    }

    pub(super) fn press_out_xl_l_h_temp_out_l_h_get(
        &mut self,
    ) -> Result<(i32, i16), Error<I::Error>> {
        let mut arr: [u8; 5] = [0; 5];
        self.read_reg(Reg::PressOutXl, &mut arr)?;
        let raw_press: i32 = arr[0] as i32 | (arr[1] as i32) << 8 | (arr[2] as i8 as i32) << 16;
        let raw_temp: i16 = arr[3] as i16 | (arr[4] as i8 as i16) << 8;

        Ok((raw_press, raw_temp))
    }

    pub(super) fn fifo_data_out_press_xl_l_h_get(&mut self) -> Result<i32, Error<I::Error>> {
        let mut arr: [u8; 3] = [0; 3];
        self.read_reg(Reg::FifoDataOutPressXl, &mut arr)?;
        let val: i32 = arr[0] as i32 | (arr[1] as i32) << 8 | (arr[2] as i8 as i32) << 16;

        Ok(val)
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Reg {
    InterruptCfg = 0xB,
    ThsPL = 0xC,
    ThsPH = 0xD,
    IfCtrl = 0xE,
    WhoAmI = 0xF,
    CtrlReg1 = 0x10,
    CtrlReg2 = 0x11,
    CtrlReg3 = 0x12,
    CtrlReg4 = 0x13,
    FifoCtrl = 0x14,
    FifoWtm = 0x15,
    IntSource = 0x24,
    FifoStatus1 = 0x25,
    FifoStatus2 = 0x26,
    Status = 0x27,
    PressOutXl = 0x28,
    PressOutH = 0x2A,
    TempOutL = 0x2B,
    FifoDataOutPressXl = 0x78,
}

bitfield! {
    struct InterruptCfg(u8);
    interrupt_cfg, _: 7, 0;
    interrupt_cfg_autorefp, set_interrupt_cfg_autorefp: 7, 7;
    interrupt_cfg_reset_arp, set_interrupt_cfg_reset_arp: 6, 6;
    interrupt_cfg_autozero, set_interrupt_cfg_autozero: 5, 5;
    interrupt_cfg_reset_az, set_interrupt_cfg_reset_az: 4, 4;
    not_used3, _: 3, 3;
    interrupt_cfg_lir, set_interrupt_cfg_lir: 2, 2;
    interrupt_cfg_ple, set_interrupt_cfg_ple: 1, 1;
    interrupt_cfg_phe, set_interrupt_cfg_phe: 0, 0;
}

bitfield! {
    struct IfCtrl(u8);
    if_ctrl, _: 7, 0;
    if_ctrl_int_en_i3c, set_if_ctrl_int_en_i3c: 7, 7;
    if_ctrl_i2c_i3c_dis, set_if_ctrl_i2c_i3c_dis: 6, 6;
    if_ctrl_sim, set_if_ctrl_sim: 5, 5;
    if_ctrl_sda_pu_en, set_if_ctrl_sda_pu_en: 5, 5;
    if_ctrl_sdo_pu_en, set_if_ctrl_sdo_pu_en: 4, 4;
    if_ctrl_int_pd_dis, set_if_ctrl_int_pd_dis: 3, 3;
    if_ctrl_cs_pu_dis, set_if_ctrl_cs_pu_dis: 2, 2;
    not_used0, _: 0, 0;
}

bitfield! {
    struct CtrlReg1(u8);
    ctrl_reg1, _: 7, 0;
    not_used7, _: 7, 7;
    ctrl_reg1_odr, set_ctrl_reg1_odr: 6, 3;
    ctrl_reg1_avg, set_ctrl_reg1_avg: 2, 0;
}

bitfield! {
    struct CtrlReg2(u8);
    ctrl_reg2, _: 7, 0;
    ctrl_reg2_boot, set_ctrl_reg2_boot: 6, 6;
    ctrl_reg2_lfpf_cfg_en_lpfp, set_ctrl_reg2_lfpf_cfg_en_lpfp: 5, 4;
    ctrl_reg2_bdu, set_ctrl_reg2_bdu: 3, 3;
    ctrl_reg2_swreset, set_ctrl_reg2_swreset: 2, 2;
    not_used1, _: 1, 1;
    ctrl_reg2_oneshot, set_ctrl_reg2_oneshot: 0, 0;
}

bitfield! {
    struct CtrlReg3(u8);
    ctrl_reg3, _: 7, 0;
    not_used7_4, _: 7, 4;
    ctrl_reg3_int_h_l, set_ctrl_reg3_int_h_l: 3, 3;
    not_used2, _: 2, 2;
    ctrl_reg3_pp_od, set_ctrl_reg3_pp_od: 1, 1;
    ctrl_reg3_if_add_inc, set_ctrl_reg3_if_add_inc: 0, 0;
}

bitfield! {
    struct CtrlReg4(u8);
    ctrl_reg4, _: 7, 0;
    not_used7, _: 7, 7;
    ctrl_reg4_drdy_pls, set_ctrl_reg4_drdy_pls: 6, 6;
    ctrl_reg4_drdy, set_ctrl_reg4_drdy: 5, 5;
    ctrl_reg4_int_en, set_ctrl_reg4_int_en: 4, 4;
    not_used3, _: 3, 3;
    ctrl_reg4_int_f_full, set_ctrl_reg4_int_f_full: 2, 2;
    ctrl_reg4_int_f_wtm, set_ctrl_reg4_int_f_wtm: 1, 1;
    ctrl_reg4_int_f_ovr, set_ctrl_reg4_int_f_ovr: 0, 0;
}

bitfield! {
    struct FifoCtrl(u8);
    fifo_ctrl, _: 7, 0;
    not_used7_4, _: 7, 4;
    fifo_ctrl_stop_on_wtm, set_fifo_ctrl_stop_on_wtm: 3, 3;
    fifo_ctrl_trig_modes_f_mode, set_fifo_ctrl_trig_modes_f_mode: 2, 0;
}

bitfield! {
    struct I3cIfCtrlAdd(u8);
    i3c_if_ctrl_add, _: 7, 0;
    not_used7_6, _: 7, 6;
    i3c_if_ctrl_add_asf_on, set_i3c_if_ctrl_add_asf_on: 5, 5;
    i3c_if_ctrl_add_i3c_bus_avb_sel1, set_i3c_if_ctrl_add_i3c_bus_avb_sel1: 1, 1;
    i3c_if_ctrl_add_i3c_bus_avb_sel0, set_i3c_if_ctrl_add_i3c_bus_avb_sel0: 0, 0;
}

bitfield! {
    struct IntSource(u8);
    int_source, _: 7, 0;
    int_source_boot_on, _: 7, 7;
    not_used6_3, _: 6, 3;
    int_source_ia, _: 2, 2;
    int_source_pl_ph, _: 1, 0;
}

bitfield! {
    struct Status(u8);
    status, _: 7, 0;
    not_used7_6, _: 7, 6;
    status_t_or, _: 5, 5;
    status_p_or, _: 4, 4;
    not_used3_2, _: 3, 2;
    status_t_da, _: 1, 1;
    status_p_da, _: 0, 0;
}

bitfield! {
    struct FifoStatus2(u8);
    fifo_status2, _: 7, 0;
    fifo_status2_fifo_wtm_ia, _: 7, 7;
    fifo_status2_fifo_ovr_ia, _: 6, 6;
    fifo_status2_fifo_full_ia, _: 5, 5;

}

#[repr(u8)]
pub(super) enum CtrlReg1Odr {
    PowerDownOneShot = 0b0000,
    Hz1 = 0b0001,
    Hz4 = 0b0010,
    Hz10 = 0b0011,
    Hz25 = 0b0100,
    Hz50 = 0b0101,
    Hz75 = 0b0110,
    Hz100 = 0b0111,
    Hz200 = 0b1000,
}

impl From<u32> for CtrlReg1Odr {
    fn from(value: u32) -> Self {
        match value {
            0 => Self::PowerDownOneShot,
            1 => Self::Hz1,
            2..=4 => Self::Hz4,
            5..=10 => Self::Hz10,
            11..=25 => Self::Hz25,
            26..=50 => Self::Hz50,
            51..=75 => Self::Hz75,
            76..=100 => Self::Hz100,
            101.. => Self::Hz200,
        }
    }
}

impl From<CtrlReg1Odr> for u32 {
    fn from(value: CtrlReg1Odr) -> Self {
        match value {
            CtrlReg1Odr::PowerDownOneShot => 0,
            CtrlReg1Odr::Hz1 => 1,
            CtrlReg1Odr::Hz4 => 4,
            CtrlReg1Odr::Hz10 => 10,
            CtrlReg1Odr::Hz25 => 25,
            CtrlReg1Odr::Hz50 => 50,
            CtrlReg1Odr::Hz75 => 75,
            CtrlReg1Odr::Hz100 => 100,
            CtrlReg1Odr::Hz200 => 200,
        }
    }
}

impl From<u8> for CtrlReg1Odr {
    fn from(value: u8) -> Self {
        match value {
            0b0000 => Self::PowerDownOneShot,
            0b0001 => Self::Hz1,
            0b0010 => Self::Hz4,
            0b0011 => Self::Hz10,
            0b0100 => Self::Hz25,
            0b0101 => Self::Hz50,
            0b0110 => Self::Hz75,
            0b0111 => Self::Hz100,
            0b1000.. => Self::Hz200,
        }
    }
}
#[repr(u8)]
pub(super) enum CtrlReg1Avg {
    Avg4 = 0b000,
    Avg8 = 0b001,
    Avg16 = 0b010,
    Avg32 = 0b011,
    Avg64 = 0b100,
    Avg128 = 0b101,
    Avg512 = 0b111,
}

impl From<u32> for CtrlReg1Avg {
    fn from(value: u32) -> Self {
        match value {
            0..=4 => Self::Avg4,
            5..=8 => Self::Avg8,
            9..=16 => Self::Avg16,
            17..=32 => Self::Avg32,
            33..=64 => Self::Avg64,
            65..=128 => Self::Avg128,
            129.. => Self::Avg512,
        }
    }
}

impl From<CtrlReg1Avg> for u32 {
    fn from(value: CtrlReg1Avg) -> Self {
        match value {
            CtrlReg1Avg::Avg4 => 4,
            CtrlReg1Avg::Avg8 => 8,
            CtrlReg1Avg::Avg16 => 16,
            CtrlReg1Avg::Avg32 => 32,
            CtrlReg1Avg::Avg64 => 64,
            CtrlReg1Avg::Avg128 => 128,
            CtrlReg1Avg::Avg512 => 512,
        }
    }
}

pub(super) struct UnexpectedAvgValue;

impl<B> From<UnexpectedAvgValue> for Error<B> {
    fn from(_: UnexpectedAvgValue) -> Self {
        Error::InvalidRegisterValue
    }
}

impl TryFrom<u8> for CtrlReg1Avg {
    type Error = UnexpectedAvgValue;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b000 => Ok(CtrlReg1Avg::Avg4),
            0b001 => Ok(CtrlReg1Avg::Avg8),
            0b010 => Ok(CtrlReg1Avg::Avg16),
            0b011 => Ok(CtrlReg1Avg::Avg32),
            0b100 => Ok(CtrlReg1Avg::Avg64),
            0b101 => Ok(CtrlReg1Avg::Avg128),
            0b111 => Ok(CtrlReg1Avg::Avg512),
            _ => Err(UnexpectedAvgValue),
        }
    }
}

impl From<u8> for super::DifferentialPressEvent {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::NoInterrupt,
            1 => Self::PressureHigh,
            2 => Self::PressureLow,
            3.. => Self::BothInterrupt,
        }
    }
}

impl From<u8> for super::FifoMode {
    fn from(value: u8) -> Self {
        match value {
            0b000 => Self::Bypass,
            0b100 => Self::Bypass,
            0b001 => Self::Fifo,
            0b010 => Self::Continuous,
            0b011 => Self::Continuous,
            0b101 => Self::BypassToFifo,
            0b110 => Self::BypassToContinuous,
            0b111.. => Self::ContinuousToFifo,
        }
    }
}
