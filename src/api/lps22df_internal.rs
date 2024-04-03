use super::{Lps22dfError, Lps22dfI2C};
use embedded_hal::i2c::I2c;
use bitfield::bitfield;


pub trait Lps22dfDriverInternal {
    type Error;

    fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Lps22dfError<Self::Error>>;
    fn write_bytes(&mut self, buf: &[u8]) -> Result<(), Lps22dfError<Self::Error>>;
    fn write_read_bytes(
        &mut self,
        wbuf: &[u8],
        rbuf: &mut [u8],
    ) -> Result<(), Lps22dfError<Self::Error>>;

    #[inline]
    fn read_from_register(
        &mut self,
        reg: Reg,
        buf: &mut [u8],
    ) -> Result<(), Lps22dfError<Self::Error>> {
        self.write_read_bytes(&[reg as u8], buf)?;

        Ok(())
    }

    #[inline]
    fn write_to_register(&mut self, reg: Reg, val: u8) -> Result<(), Lps22dfError<Self::Error>> {
        self.write_bytes(&[reg as u8, val])?;
        let mut arr: [u8; 1] = [0];
        self.read_from_register(reg, &mut arr)?;
        if arr[0] != val {
            return Err(Lps22dfError::WriteFailure);
        }

        Ok(())
    }

    fn ctrl_reg1_set_odr(&mut self, odr: CtrlReg1Odr) -> Result<(), Lps22dfError<Self::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg1, &mut arr)?;
        let mut val = CtrlReg1(arr[0]);
        val.set_ctrl_reg1_odr(odr as u8);
        self.write_to_register(Reg::CtrlReg1, val.ctrl_reg1())?;

        Ok(())
    }

    fn ctrl_reg1_set_avg(&mut self, avg: CtrlReg1Avg) -> Result<(), Lps22dfError<Self::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::CtrlReg1, &mut arr)?;
        let mut val = CtrlReg1(arr[0]);
        val.set_ctrl_reg1_avg(avg as u8);
        self.write_to_register(Reg::CtrlReg1, val.ctrl_reg1())?;
        
        Ok(())
    }

    fn status_get_t_da(&mut self) -> Result<bool, Lps22dfError<Self::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status, &mut arr)?;
        let val = Status(arr[0]);

        Ok(val.status_t_da() != 0)
    }

    fn status_get_p_da(&mut self) -> Result<bool, Lps22dfError<Self::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status, &mut arr)?;
        let val = Status(arr[0]);

        Ok(val.status_p_da() != 0)
    }

    fn temp_out_get_l_h(&mut self) -> Result<i32, Lps22dfError<Self::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::TempOutL, &mut arr)?;
        let val: i32 = arr[0] as i32 | (arr[1] as i32) << 8;

        Ok(val)
    }
}

impl<P> Lps22dfDriverInternal for Lps22dfI2C<P>
where
    P: I2c,
{
    type Error = P::Error;

    #[inline]
    fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Lps22dfError<Self::Error>> {
        self.i2c
            .read(self.address, buf)
            .map_err(Lps22dfError::I2C)?;

        Ok(())
    }

    #[inline]
    fn write_bytes(&mut self, buf: &[u8]) -> Result<(), Lps22dfError<Self::Error>> {
        self.i2c
            .write(self.address, buf)
            .map_err(Lps22dfError::I2C)?;

        Ok(())
    }

    #[inline]
    fn write_read_bytes(
        &mut self,
        wbuf: &[u8],
        rbuf: &mut [u8],
    ) -> Result<(), Lps22dfError<Self::Error>> {
        self.i2c
            .write_read(self.address, wbuf, rbuf)
            .map_err(Lps22dfError::I2C)?;

        Ok(())
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    WhoAmI = 0xF,
    CtrlReg1 = 0x10,
    Status = 0x27,
    TempOutL = 0x2B,
}

bitfield! {
    struct CtrlReg1(u8);
    ctrl_reg1, _: 7, 0;
    not_used7, _: 7, 7;
    ctrl_reg1_odr, set_ctrl_reg1_odr: 6, 3;
    ctrl_reg1_avg, set_ctrl_reg1_avg: 2, 0;
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

#[repr(u8)]
pub enum CtrlReg1Odr {
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
            2 ..= 4 => Self::Hz4,
            5 ..= 10 => Self::Hz10,
            11 ..= 25 => Self::Hz25,
            26 ..= 50 => Self::Hz50,
            51 ..= 75 => Self::Hz75,
            76 ..= 100 => Self::Hz100,
            101 .. => Self::Hz200,
        }
    }
}

#[repr(u8)]
pub enum CtrlReg1Avg {
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
            0 ..= 4 => Self::Avg4,
            5 ..= 8 => Self::Avg8,
            9 ..= 16 => Self::Avg16,
            17 ..= 32 => Self::Avg32,
            33 ..= 64 => Self::Avg64,
            65 ..= 128 => Self::Avg128,
            129.. => Self::Avg512,
        }
    }
}