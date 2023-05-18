use board::hal as board;
use crate::traits;
use embassy_usb::class::cdc_acm;
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use embassy_usb::UsbDevice;
use static_cell::StaticCell;

pub struct Console {}

impl Console {
    pub fn new(
        p: board::Peripherals,
    ) -> (USB, Reciever, Sender) {
        let driver = board::usb_driver(p);

        // Create embassy-usb Config
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Embassy");
        config.product = Some("USB-serial example");
        config.serial_number = Some("12345678");

        // Required for windows compatiblity.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = Builder::new(
            driver,
            config,
            &mut crate::singleton!([0; 256])[..],
            &mut crate::singleton!([0; 256])[..],
            &mut crate::singleton!([0; 256])[..],
            &mut crate::singleton!([0; 128])[..],
        );

        // Create classes on the builder.
        let class =
            cdc_acm::CdcAcmClass::new(&mut builder, crate::singleton!(cdc_acm::State::new()), 64);
        let (tx, rx) = class.split();

        // Build the builder.
        let usb = builder.build();

        (USB(usb), Reciever(rx), Sender(tx))
    }
}

pub struct USB (pub UsbDevice<'static, board::UsbDriver>);

impl USB {
    pub async fn run(&mut self) {
        loop {
            self.0.run_until_suspend().await;
            self.0.wait_resume().await;
        }
    }
}

pub struct Sender(pub cdc_acm::Sender<'static, board::UsbDriver>);

impl traits::Writer for Sender {
    async fn write<'a>(&'a mut self, data: &'a [u8]) -> Result<(), EndpointError> {
        if self.0.dtr() != true || self.0.rts() != true {
            return Err(EndpointError::Disabled)
        }
        self.0.wait_connection().await;
        self.0.write_packet(data).await
    }
}

pub struct Reciever(pub cdc_acm::Receiver<'static, board::UsbDriver>);

impl traits::Reader for Reciever {
    async fn read<'a>(&'a mut self, data: &'a mut [u8]) -> Result<usize, EndpointError> {
        self.0.wait_connection().await;
        self.0.read_packet(data).await
    }
}

