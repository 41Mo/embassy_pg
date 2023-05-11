use core::mem::MaybeUninit;

use defmt::{debug, info, Format};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::channel::*;
use embassy_time::{with_timeout, Duration, TimeoutError, Timer};
use embassy_usb::class::cdc_acm::{Receiver, Sender};
use embassy_usb::driver::EndpointError;
use embassy_usb::{driver::Driver, UsbDevice};
use futures::pin_mut;
use heapless::spsc::{Consumer, Producer, Queue};
use static_cell::StaticCell;

const RB_SIZE: usize = 512;
const CHUNK_SIZE: usize = 64;

static mut RB: Queue<u8, RB_SIZE> = Queue::new();
static SENDER: CriticalSectionMutex<StaticCell<Producer<u8, RB_SIZE>>> =
    CriticalSectionMutex::new(StaticCell::new());

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

async fn recieve_from_rb<'a>(c: &mut Consumer<'a, u8, RB_SIZE>, slice: &mut [u8]) -> usize {
    let mut bytes_readed = 0;
    while c.ready() && bytes_readed < CHUNK_SIZE {
        slice[bytes_readed] = c.dequeue().unwrap();
        bytes_readed += 1;
    }
    return bytes_readed;
}

async fn tx<'a, T: Driver<'a>>(tx: &mut Sender<'a, T>) {
    Timer::after(Duration::from_secs(5)).await;
    let mut consumer = unsafe { RB.split().1 };
    let mut buf = [0u8; 64];
    debug!("Tx start task");
    tx.wait_connection().await;
    loop {
        let bytes = with_timeout(
            Duration::from_millis(1000),
            recieve_from_rb(&mut consumer, &mut buf),
        )
        .await;
        let bytes = match bytes {
            Ok(bytes) => bytes,
            Err(e) => {debug!("recv_from_rb: {}", e); 0},
        };
        // debug!("bytes readed {}", bytes);
        match transmit_bytes(tx, &buf[..bytes]).await {
            Ok(_) => (),
            Err(e) => {
                debug!("tx: {}", e);
                break;
            }
        };
    }
}

pub async fn console_print(bytes: &[u8]) {
    let mut p = unsafe { RB.split().0 };
    for i in bytes {
        let _ = p.enqueue(i.clone());
    }
}

#[embassy_executor::task]
pub async fn usb_task(mut c: Console<'static, crate::UsbDriver>) -> ! {
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
