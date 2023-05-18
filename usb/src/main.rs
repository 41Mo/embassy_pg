#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

mod usb;
mod mavlink_ops;
mod traits;

use cortex_m_rt::entry;
use defmt::{debug, info, unwrap};
use embassy_executor::Executor;
use {defmt_rtt as _, panic_probe as _};
use static_cell::StaticCell;

use board::singleton;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    info!("Entry point");

    let executor = EXECUTOR.init(Executor::new());

    let console = init();

    executor.run(|spawner| {
        unwrap!(spawner.spawn(mavlink_ops::mavlnk_task(console.0, console.1, console.2)));

    })
}

fn init() -> (
    usb::USB,
    usb::Reciever,
    usb::Sender,
) {
    let p = board::hal::init_periph();
    usb::Console::new(p)
}
