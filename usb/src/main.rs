#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

mod usb;
mod board;
mod mavlink_ops;
mod traits;

use cortex_m_rt::entry;
use defmt::{debug, info, unwrap};
use embassy_executor::Executor;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[macro_export]
macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

#[entry]
fn main() -> ! {
    info!("Entry point");

    let executor = EXECUTOR.init(Executor::new());

    let console = board::init();

    executor.run(|spawner| {
        unwrap!(spawner.spawn(mavlink_ops::mavlnk_task(console.0, console.1, console.2)));

    })
}
