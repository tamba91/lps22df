use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

//use self::lps22df_internal::{CtrlReg1Avg, CtrlReg1Odr};

mod lps22df_internal;

pub struct Lps22df<T> {
    bus: T,
}

impl<P: I2c> Lps22df<lps22df_internal::Lps22dfI2C<P>> {
    pub fn new_i2c(i2c: P, address: SevenBitAddress) -> Self {
        Lps22df {
            bus: lps22df_internal::Lps22dfI2C { i2c, address },
        }
    }
}

impl<P: SpiDevice> Lps22df<lps22df_internal::Lps22dfSPI<P>> {
    pub fn new_spi(spi: P) -> Self {
        Lps22df {
            bus: lps22df_internal::Lps22dfSPI { spi },
        }
    }
}

impl<T: lps22df_internal::BusOperation> Lps22df<T> {
    fn read_from_register(
        &mut self,
        reg: lps22df_internal::Reg,
        buf: &mut [u8],
    ) -> Result<(), lps22df_internal::Lps22dfError<T::Error>> {
        self.bus.write_read_bytes(&[reg as u8], buf)?;

        Ok(())
    }
    pub fn who_am_i(&mut self) -> Result<u8, lps22df_internal::Lps22dfError<T::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(lps22df_internal::Reg::WhoAmI, &mut arr)?;
        if arr[0] != 0xB4 {
            return Err(lps22df_internal::Lps22dfError::WhoAmIError(arr[0]));
        }

        Ok(arr[0])
    }
}
