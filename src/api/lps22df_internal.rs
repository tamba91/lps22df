use embedded_hal::i2c::{I2c, SevenBitAddress};
//use embedded_hal::spi::SpiDevice;

pub(crate) struct  Lps22dfI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P:I2c> Lps22dfI2C<P> {
    pub(crate) fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self {i2c, address}
    }
}

// pub struct  Lps22dfSPI<P> {
//     pub(crate) spi: P,
// }

pub trait BusOperation {
    type Error;

    fn write_read_bytes(
        &mut self,
        wbuf: &[u8],
        rbuf: &mut [u8],
    ) -> Result<(), Lps22dfError<Self::Error>>;
}

impl<P: I2c> BusOperation for Lps22dfI2C<P> {
    type Error = P::Error;

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

// impl<P: SpiDevice> BusOperation for Lps22dfSPI<P> {
//     type Error = P::Error;

//     #[inline]
//     fn write_read_bytes(
//         &mut self,
//         wbuf: &[u8],
//         rbuf: &mut [u8],
//     ) -> Result<(), Lps22dfError<Self::Error>> {
//         self.spi.write(wbuf).map_err(Lps22dfError::SPI)?;
//         self.spi.read(rbuf).map_err(Lps22dfError::SPI)?;

//         Ok(())
//     }
// }

#[derive(Copy, Clone, Debug)]
pub enum Lps22dfError<P> {
    I2C(P),
    SPI(P),
    WhoAmIError(u8),
    WriteFailure,
    InvalidValue,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    WhoAmI = 0xF,
    // CtrlReg1 = 0x10,
    // Status = 0x27,
    // TempOutL = 0x2B,
}
