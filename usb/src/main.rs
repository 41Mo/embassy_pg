#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod usb;
mod board;

use cortex_m_rt::entry;
use defmt::{debug, info, unwrap};
use embassy_executor::Executor;
use embassy_time::{Duration, Timer, Instant};
use embassy_futures::join::join;
use mavlink;
use mavlink::common as mav_common;
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

static MAV_HEADER: mavlink::MavHeader = mavlink::MavHeader {
    system_id: 1,
    component_id: 1,
    sequence: 0,
};

async fn send_heartbeat() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        let mut msg = mavlink::MAVLinkV2MessageRaw::new();
        msg.serialize_message(
            MAV_HEADER,
            &mut mav_common::MavMessage::HEARTBEAT(mav_common::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: mavlink::common::MavType::MAV_TYPE_FIXED_WING,
                autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
                base_mode: mavlink::common::MavModeFlag::empty(),
                system_status: mavlink::common::MavState::MAV_STATE_ACTIVE,
                mavlink_version: 0x3,
            }),
        );
        usb::console_print(msg.as_bytes()).await;
    }
}

async fn send_scaled_imu() -> ! {
    loop {
        Timer::after(Duration::from_hz(10)).await;
        let mut msg = mavlink::MAVLinkV2MessageRaw::new();
        msg.serialize_message(
            MAV_HEADER,
            &mut mav_common::MavMessage::SCALED_IMU(mav_common::SCALED_IMU_DATA {
                time_boot_ms: Instant::now().as_millis() as u32,
                xacc: 0,
                yacc: 0,
                zacc: 9810,
                xgyro: 0,
                ygyro: 0,
                zgyro: 0,
                xmag: 0,
                ymag: 0,
                zmag: 0,
            }),
        );
        usb::console_print(msg.as_bytes()).await;
    }
}

#[embassy_executor::task]
async fn sending_task() {
    join(send_heartbeat(), send_scaled_imu()).await;
}

#[entry]
fn main() -> ! {
    info!("Entry point");


    let executor = EXECUTOR.init(Executor::new());

    let console = board::init();

    executor.run(|spawner| {
        unwrap!(spawner.spawn(usb::usb_task(console)));
        unwrap!(spawner.spawn(sending_task()));
    })
}
