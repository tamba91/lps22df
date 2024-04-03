use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

mod lps22df_internal;

pub struct Lps22df<T> {
    bus: T,
}

impl<P: I2c> Lps22df<lps22df_internal::Lps22dfI2C<P>> {
    pub fn new_i2c(i2c: P, address: SevenBitAddress) -> Self {
        let bus = lps22df_internal::Lps22dfI2C::new(i2c, address);
        Self{bus}
    }
}

impl<P: SpiDevice> Lps22df<lps22df_internal::Lps22dfSPI<P>> {
    pub fn new_spi(spi: P) -> Self {
        let bus = lps22df_internal::Lps22dfSPI::new(spi);
        Self{bus}
    }
}

pub trait BusOperation {
    type Error;
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Lps22dfError<Self::Error>>;

    fn write_read_bytes(
        &mut self,
        wbuf: &[u8],
        rbuf: &mut [u8],
    ) -> Result<(), Lps22dfError<Self::Error>>;
}

#[derive(Copy, Clone, Debug)]
pub enum Lps22dfError<P> {
    I2C(P),
    SPI(P),
    WhoAmIError(u8),
    WriteFailure,
    InvalidValue,
}

impl<T: BusOperation> Lps22df<T> {
    
    pub fn who_am_i(&mut self) -> Result<u8, Lps22dfError<T::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(lps22df_internal::Reg::WhoAmI, &mut arr)?;
        if arr[0] != 0xB4 {
            return Err(Lps22dfError::WhoAmIError(arr[0]));
        }

        Ok(arr[0])
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
        let raw = self.temp_out_get_l_h()?;
        let val = raw as f32 / 100.0;
        
        Ok(val)
    }

    pub fn get_press(&mut self) -> Result<f32, Lps22dfError<T::Error>> {
        let raw = self.press_out_get_xl_l_h()?;
        let val = raw as f32 / 4096.0;

        Ok(val)
    }
}

