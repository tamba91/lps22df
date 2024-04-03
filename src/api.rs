use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

use self::lps22df_internal::{CtrlReg1Avg, CtrlReg1Odr};

mod lps22df_internal;

pub struct  Lps22dfI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

pub struct  Lps22dfSPI<P> {
    spi: P,
}


impl<P> Lps22dfI2C<P>
where
    P: I2c,
{
    pub fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self{ i2c, address }
    }
}

impl<P> Lps22dfSPI<P>
where
    P: SpiDevice,
{
    pub fn new(spi: P) -> Self {
        Self { spi }
    }
}

impl<P> Lps22dfApi for Lps22dfI2C<P> where P: I2c {}
pub trait Lps22dfApi:  lps22df_internal::Lps22dfDriverInternal{

    fn who_am_i(&mut self) -> Result<u8, Lps22dfError<Self::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(lps22df_internal::Reg::WhoAmI, &mut arr)?;
        if arr[0] != 0xB4 {
            return Err(Lps22dfError::WhoAmIError(arr[0]));
        }

        Ok(arr[0])
    }

    fn set_odr(&mut self, odr: u32) -> Result<(), Lps22dfError<Self::Error>> {
        let odr: CtrlReg1Odr = odr.into();
        self.ctrl_reg1_set_odr(odr)?;
        
        Ok(())
    }

    fn set_avg(&mut self, avg: u32) -> Result<(), Lps22dfError<Self::Error>> {
        let avg: CtrlReg1Avg = avg.into();
        self.ctrl_reg1_set_avg(avg)?;

        Ok(())
    }

    fn is_press_data_avail(&mut self) -> Result<bool, Lps22dfError<Self::Error>> {
        let val = self.status_get_p_da()?;

        Ok(val)
    }

    fn is_temp_data_avail(&mut self) -> Result<bool, Lps22dfError<Self::Error>> {
        let val = self.status_get_t_da()?;

        Ok(val)
    }

    fn get_temp(&mut self) -> Result<f32, Lps22dfError<Self::Error>> {
        let raw = self.temp_out_get_l_h()?;
        let val = raw as f32 / 100.0;
        
        Ok(val)
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Lps22dfError<P> {
    I2C(P),
    SPI(P),
    WhoAmIError(u8),
    WriteFailure,
    InvalidValue,
}
