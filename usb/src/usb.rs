use defmt::{debug, info, Format};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pipe::*;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{Receiver, Sender};
use embassy_usb::driver::EndpointError;
use embassy_usb::{driver::Driver, UsbDevice};
use futures::pin_mut;
use crate::board;
use embassy_usb::class::cdc_acm;
use static_cell::StaticCell;

const RB_SIZE: usize = 512;

static mut RB: Pipe<NoopRawMutex, RB_SIZE> = Pipe::new();

#[derive(Format)]
enum TransmitErr {
    BufferOverflow,
    Disabled,
    Timeout,
}

pub struct Console<'a, T: Driver<'a>> {
    pub device: UsbDevice<'a, T>,
    pub rx: Receiver<'a, T>,
    pub tx: Sender<'a, T>,
}

impl Console<'static, board::UsbDriver> {
    pub fn new(p: embassy_stm32::Peripherals) -> Console<'static, board::UsbDriver> {
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
        let class = cdc_acm::CdcAcmClass::new(&mut builder, crate::singleton!(cdc_acm::State::new()), 64);
        let (tx, rx) = class.split();

        // Build the builder.
        let usb = builder.build();

        Console {
            device: usb,
            tx,
            rx,
        }
    }
}

async fn poll<'a, T: Driver<'a>>(device: &mut UsbDevice<'a, T>) {
    loop {
        debug!("run until suspend");
        device.run_until_suspend().await;
        debug!("Suspended, waiting resume");
        device.wait_resume().await;
    }
}

async fn rx<'a, T: Driver<'a>>(rx: &mut Receiver<'a, T>) {
    let mut buf = [0u8; 256];
    loop {
        rx.wait_connection().await;
        let res = with_timeout(Duration::from_millis(10), rx.read_packet(&mut buf)).await;
        match res {
            Ok(rx_res) => {
                match rx_res {
                    Ok(_) => (),
                    Err(e) => {
                        debug!("rx err: {}", e);
                    }
                };
            }
            Err(_) => {
                // info!("usb rx timeout: {}", e);
            }
        }
    }
}

async fn transmit_bytes<'a, T: Driver<'a>>(
    tx: &mut Sender<'a, T>,
    buf: &[u8],
) -> Result<(), TransmitErr> {
    let r = with_timeout(Duration::from_millis(5), tx.write_packet(&buf)).await;
    let txr = match r {
        Ok(txr) => txr,
        Err(_) => {
            return Err(TransmitErr::Timeout);
        }
    };
    match txr {
        Ok(_) => (),
        Err(e) => {
            match e {
                EndpointError::BufferOverflow => return Err(TransmitErr::BufferOverflow),
                EndpointError::Disabled => return Err(TransmitErr::Disabled),
            };
        }
    };

    Ok(())
}

async fn tx<'a, T: Driver<'a>>(tx: &mut Sender<'a, T>) {
    Timer::after(Duration::from_secs(5)).await;
    let consumer = unsafe { RB.reader() };
    let mut buf = [0u8; 64];
    debug!("Tx start task");
    tx.wait_connection().await;
    loop {
        let bytes = with_timeout(Duration::from_millis(1000), consumer.read(&mut buf)).await;
        let bytes = match bytes {
            Ok(bytes) => {
                /* debug!("bytes: {}", bytes); */
                bytes
            }
            Err(e) => {
                debug!("recv_from_rb: {}", e);
                0
            }
        };
        match transmit_bytes(tx, &buf[..bytes]).await {
            Ok(_) => (),
            Err(e) => {
                debug!("tx: {}", e);
                break;
            }
        };
    }
}

pub async fn console_print(bytes: &mut [u8]) {
    let p = unsafe { RB.writer() };
    let mut b_written = 0;
    while b_written != bytes.len() {
        b_written += p.write(&mut bytes[b_written..]).await
    }
}

#[embassy_executor::task]
pub async fn usb_task(mut c: Console<'static, crate::board::UsbDriver>) -> ! {
    loop {
        c.device.disable().await;
        debug!("device disabled");
        let tx_fut = tx(&mut c.tx);
        let rx_fut = rx(&mut c.rx);
        let run_fut = poll(&mut c.device);
        pin_mut!(tx_fut, rx_fut, run_fut);
        embassy_futures::select::select3(run_fut, tx_fut, rx_fut).await;
        debug!("restarting usb device");
    }
}
