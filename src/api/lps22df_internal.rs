use bitfield::bitfield;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

use super::Lps22dfError;

pub struct Lps22dfI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P: I2c> Lps22dfI2C<P> {
    pub(super) fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

pub struct Lps22dfSPI<P> {
    spi: P,
}

impl<P: SpiDevice> Lps22dfSPI<P> {
    pub(super) fn new(spi: P) -> Self {
        Self { spi }
    }
}

impl<P: I2c> super::BusOperation for Lps22dfI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;

        Ok(())
    }

    #[inline]
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }

    #[inline]
    fn write_read_bytes(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;

        Ok(())
    }
}

impl<P: SpiDevice> super::BusOperation for Lps22dfSPI<P> {
    type Error = P::Error;

    #[inline]
    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.read(rbuf)?;

        Ok(())
    }

    #[inline]
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(wbuf)?;

        Ok(())
    }

    #[inline]
    fn write_read_bytes(&mut self, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.write(wbuf)?;
        self.spi.read(rbuf)?;

        Ok(())
    }
}

impl<B: super::BusOperation> super::Lps22df<B> {
    fn read_from_register(
        &mut self,
        reg: Reg,
        buf: &mut [u8],
    ) -> Result<(), Lps22dfError<B::Error>> {
        self.bus
            .write_read_bytes(&[reg as u8], buf)
            .map_err(Lps22dfError::Bus)?;

        Ok(())
    }

    #[inline]
    fn write_to_register(&mut self, reg: Reg, val: u8) -> Result<(), Lps22dfError<B::Error>> {
        self.bus
            .write_bytes(&[reg as u8, val])
            .map_err(Lps22dfError::Bus)?;
        let mut arr: [u8; 1] = [0];
        self.read_from_register(reg, &mut arr)?;
        if arr[0] != val {
            return Err(Lps22dfError::WriteFailure);
        }

        Ok(())
    }

    #[inline]
    fn write_to_register_no_check(
        &mut self,
        reg: Reg,
        val: u8,
    ) -> Result<(), Lps22dfError<B::Error>> {
        self.bus
            .write_bytes(&[reg as u8, val])
            .map_err(Lps22dfError::Bus)?;

        Ok(())
    }

    pub(super) fn who_am_i_get(&mut self) -> Result<u8, Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::WhoAmI, &mut arr)?;

        Ok(arr[0])
    }

    pub(super) fn ctrl_reg1_get_odr(&mut self) -> Result<CtrlReg1Odr, Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg1, &mut arr)?;
        let val = (CtrlReg1(arr[0]).ctrl_reg1_odr()) as u32;
        let odr: CtrlReg1Odr = val.into();

        Ok(odr)
    }

    pub(super) fn ctrl_reg1_set_odr(
        &mut self,
        odr: CtrlReg1Odr,
    ) -> Result<(), Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg1, &mut arr)?;
        let mut val = CtrlReg1(arr[0]);
        val.set_ctrl_reg1_odr(odr as u8);
        self.write_to_register(Reg::CtrlReg1, val.ctrl_reg1())?;

        Ok(())
    }

    pub(super) fn ctrl_reg1_set_avg(
        &mut self,
        avg: CtrlReg1Avg,
    ) -> Result<(), Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg1, &mut arr)?;
        let mut val = CtrlReg1(arr[0]);
        val.set_ctrl_reg1_avg(avg as u8);
        self.write_to_register(Reg::CtrlReg1, val.ctrl_reg1())?;

        Ok(())
    }
    pub(super) fn ctrl_reg2_get_oneshot(&mut self) -> Result<bool, Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg2, &mut arr)?;
        let val = CtrlReg2(arr[0]).ctrl_reg2_oneshot();

        Ok(val != 0)
    }

    pub(super) fn ctrl_reg2_set_oneshot(&mut self) -> Result<(), Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg2, &mut arr)?;
        let mut val = CtrlReg2(arr[0]);
        val.set_ctrl_reg2_oneshot(true as u8);
        self.write_to_register_no_check(Reg::CtrlReg2, val.ctrl_reg2())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_drdy_pulsed(
        &mut self,
        drdy_pulsed: bool,
    ) -> Result<(), Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_drdy_pls(drdy_pulsed as u8);
        self.write_to_register(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn ctrl_reg4_set_drdy(&mut self, drdy: bool) -> Result<(), Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg4, &mut arr)?;
        let mut val = CtrlReg4(arr[0]);
        val.set_ctrl_reg4_drdy(drdy as u8);
        self.write_to_register(Reg::CtrlReg4, val.ctrl_reg4())?;

        Ok(())
    }

    pub(super) fn status_get_t_da(&mut self) -> Result<bool, Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status, &mut arr)?;
        let val = Status(arr[0]);

        Ok(val.status_t_da() != 0)
    }

    pub(super) fn status_get_p_da(&mut self) -> Result<bool, Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status, &mut arr)?;
        let val = Status(arr[0]);

        Ok(val.status_p_da() != 0)
    }

    pub(super) fn temp_out_l_h_get(&mut self) -> Result<i32, Lps22dfError<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::TempOutL, &mut arr)?;
        let val: i32 = arr[0] as i32 | (arr[1] as i32) << 8;

        Ok(val)
    }

    pub(super) fn press_out_xl_l_h_get(&mut self) -> Result<i32, Lps22dfError<B::Error>> {
        let mut arr: [u8; 3] = [0; 3];
        self.read_from_register(Reg::PressOutXl, &mut arr)?;
        let val: i32 = arr[0] as i32 | (arr[1] as i32) << 8 | (arr[2] as i32) << 16;

        Ok(val)
    }

    pub(super) fn press_out_h_get(&mut self) -> Result<u8, Lps22dfError<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::PressOutH, &mut arr)?;
        let val: u8 = arr[0];

        Ok(val)
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    WhoAmI = 0xF,
    CtrlReg1 = 0x10,
    CtrlReg2 = 0x11,
    CtrlReg4 = 0x13,
    Status = 0x27,
    PressOutXl = 0x28,
    PressOutH = 0x2A,
    TempOutL = 0x2B,
}

bitfield! {
    pub struct CtrlReg1(u8);
    ctrl_reg1, _: 7, 0;
    not_used7, _: 7, 7;
    ctrl_reg1_odr, set_ctrl_reg1_odr: 6, 3;
    ctrl_reg1_avg, set_ctrl_reg1_avg: 2, 0;
}

bitfield! {
    pub struct CtrlReg2(u8);
    ctrl_reg2, _: 7, 0;
    ctrl_reg2_boot, set_ctrl_reg2_boot: 6, 6;
    ctrl_reg2_lfpf_cfg, set_ctrl_reg2_lfpf_cfg: 5, 5;
    ctrl_reg2_en_lpfp, set_ctrl_reg2_en_lpfp: 4, 4;
    ctrl_reg2_bdu, set_ctrl_reg2_bdu: 3, 3;
    ctrl_reg2_swreset, set_ctrl_reg2_swreset: 2, 2;
    not_used1, _: 1, 1;
    ctrl_reg2_oneshot, set_ctrl_reg2_oneshot: 0, 0;
}

bitfield! {
    pub struct CtrlReg4(u8);
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
    struct Status(u8);
    status, _: 7, 0;
    not_used7_6, _: 7, 6;
    status_t_or, _: 5, 5;
    status_p_or, _: 4, 4;
    not_used3_2, _: 3, 2;
    status_t_da, _: 1, 1;
    status_p_da, _: 0, 0;
}

#[derive(PartialEq)]
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
