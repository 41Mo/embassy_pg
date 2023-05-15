use embassy_usb::driver::EndpointError;

pub trait Writer {
    async fn write<'a>(&'a mut self, data: &'a [u8]) -> Result<(), EndpointError>;
}

pub trait Reader {
    async fn read<'a>(&'a mut self, data: &'a mut [u8]) -> Result<usize, EndpointError>;
}
