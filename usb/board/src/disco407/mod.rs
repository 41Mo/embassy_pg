use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{interrupt};
pub use crate::Peripherals;
use crate::singleton;

const SYS_CK: Hertz = mhz(168);
const HCLK: Hertz = mhz(168);
// const HSE_CLK: Hertz = mhz(24);

pub type UsbDriver = Driver<'static, USB_OTG_FS>;

pub fn usb_driver(p: Peripherals) -> UsbDriver {
    let irq = interrupt::take!(OTG_FS);
    Driver::new_fs(
        p.USB_OTG_FS,
        irq,
        p.PA12,
        p.PA11,
        &mut singleton!([0u8; 256])[..],
    )
}

pub fn init_periph() -> Peripherals {
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(SYS_CK);
    config.rcc.hclk = Some(HCLK);
    config.rcc.pll48 = true;
    // config.rcc.hse = Some(HSE_CLK);

    embassy_stm32::init(config)
}

