#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m_rt::entry;
use defmt::{panic, *};
use embassy_executor::{Executor, Spawner};
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::time::mhz;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::{self, interrupt, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, UsbDevice};
use embedded_hal_async as eha;
use futures::future::{join, select, Either};
use futures::{join, pin_mut, select_biased, stream, FutureExt};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static EXECUTOR: StaticCell<Executor> = StaticCell::new();
static SIGNAL: Signal<CriticalSectionRawMutex, usize> = Signal::new();
static SIGNAL_USB_RESET: Signal<CriticalSectionRawMutex, bool> = Signal::new();

type MyDriver = Driver<'static, USB_OTG_FS>;

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

struct SerialConsole<D, R, S> {
    device: D,
    rx: R,
    tx: S,
}

enum UsbTaskStates {
    Suspended,
    NeedReset,
}

type Console = SerialConsole<
    UsbDevice<'static, Driver<'static, USB_OTG_FS>>,
    Receiver<'static, Driver<'static, USB_OTG_FS>>,
    Sender<'static, Driver<'static, USB_OTG_FS>>,
>;


impl Console {
    async fn poll(mut device: UsbDevice<'static, Driver<'static, USB_OTG_FS>>) {
        loop {
            let run_status = {
                let usb_run_fut = device.run_until_suspend();
                let usb_reset_fut = SIGNAL_USB_RESET.wait();
                pin_mut!(usb_run_fut);
                pin_mut!(usb_reset_fut);
                debug!("run until suspend!");

                match select(usb_run_fut, usb_reset_fut).await {
                    Either::Left(_) => {
                        debug!("suspended!");
                        UsbTaskStates::Suspended
                    }
                    Either::Right(_) => {
                        debug!("need reset!");
                        UsbTaskStates::NeedReset
                    }
                }
            };
            match run_status {
                UsbTaskStates::Suspended => {
                    debug!("waiting resume");
                    device.wait_resume().await;
                    debug!("usb resumed")
                }
                UsbTaskStates::NeedReset => {
                    device.disable().await;
                    debug!("usb disabled")
                }
            };
        }
    }

    async fn rx(mut rx: Receiver<'static, Driver<'static, USB_OTG_FS>>) {
        let mut buf = [0u8; 256];
        loop {
            rx.wait_connection().await;
            let res = with_timeout(Duration::from_millis(10), rx.read_packet(&mut buf)).await;
            match res {
                Ok(rx_res) => {
                    match rx_res {
                        Ok(_) => (),
                        Err(e) => {
                            info!("rx err: {}", e);
                        }
                    };
                }
                Err(e) => {
                    // info!("usb rx timeout: {}", e);
                }
            }
        }
    }

    async fn tx(mut tx: Sender<'static, Driver<'static, USB_OTG_FS>>) {
        // let buf: [u8; 200] = array_init::from_iter(iter).unwrap();
        const buf_size: usize = 256;
        let mut buf: [u8; buf_size] = [0u8; buf_size];
        for i in 0..buf_size {
            buf[i] = i as u8;
        }
        loop {
            tx.wait_connection().await;
            SIGNAL.wait().await;
            let mut start = 0;
            for i in (64..buf_size + 1).step_by(64) {
                let res =
                    with_timeout(Duration::from_millis(100), tx.write_packet(&buf[start..i])).await;
                match res {
                    Ok(tx_res) => {
                        match tx_res {
                            Ok(_) => start = i,
                            Err(e) => {
                                debug!("tx err: {}", e);
                                match e {
                                    EndpointError::BufferOverflow => {
                                        Timer::after(Duration::from_secs(5)).await
                                    }
                                    EndpointError::Disabled => tx.wait_connection().await,
                                };
                            }
                        };
                    }
                    Err(e) => {
                        debug!("usb tx timeout: {}", e);
                        /*
                         * FIXME unnessesary reset on tx timout. But if i dont do this, usb stack
                         * cant reach succesfull setup state
                         */
                        SIGNAL_USB_RESET.signal(true);
                        Timer::after(Duration::from_secs(5)).await;
                        break;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_task(c: Console) {
    let tx_fut = Console::tx(c.tx);
    let rx_fut = Console::rx(c.rx);
    let run_fut = Console::poll(c.device);
    join!(run_fut, tx_fut, rx_fut);
}

#[embassy_executor::task]
async fn sending_task() {
    let mut counter: usize = 0;

    loop {
        Timer::after(Duration::from_millis(2)).await;

        SIGNAL.signal(counter);

        counter = counter.wrapping_add(1);
    }
}

#[entry]
fn main() -> ! {
    info!("Entry point");

    let mut config = Config::default();
    config.rcc.sys_ck = Some(mhz(400));
    config.rcc.hclk = Some(mhz(200));
    config.rcc.pll1.q_ck = Some(mhz(100));
    let p = embassy_stm32::init(config);

    // Create the driver, from the HAL.
    let irq = interrupt::take!(OTG_FS);
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        irq,
        p.PA12,
        p.PA11,
        &mut singleton!([0u8; 256])[..],
    );

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
        &mut singleton!([0; 256])[..],
        &mut singleton!([0; 256])[..],
        &mut singleton!([0; 256])[..],
        &mut singleton!([0; 128])[..],
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, singleton!(State::new()), 64);
    let (tx, rx) = class.split();

    // Build the builder.
    let usb = builder.build();

    let executor = EXECUTOR.init(Executor::new());

    let console = Console {
        device: usb,
        tx,
        rx,
    };

    executor.run(|spawner| {
        unwrap!(spawner.spawn(usb_task(console)));
        unwrap!(spawner.spawn(sending_task()));
    })
}
