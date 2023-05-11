#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod usb;

use cortex_m_rt::entry;
use defmt::{debug, info, unwrap};
use embassy_executor::Executor;
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::time::mhz;
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{self, interrupt, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, Instant};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use embassy_futures::join::join;
use mavlink;
use mavlink::common as mav_common;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::usb::{usb_task, Console};

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

pub type UsbDriver = Driver<'static, USB_OTG_FS>;

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
    // join(send_heartbeat(), send_scaled_imu()).await;
    send_scaled_imu().await;
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
