use crate::usb;
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{interrupt, Peripherals};
use static_cell::StaticCell;

const SYS_CK: Hertz = mhz(400);
const HCLK: Hertz = mhz(200);
const PLL1_QCK: Hertz = mhz(80);
// const PLL2_PCK: Hertz = mhz(180);
// const PLL2_QCK: Hertz = mhz(72);
// const PLL2_RCK: Hertz = mhz(360);
// const HSE_CLK: Hertz = mhz(24);

pub type UsbDriver = Driver<'static, USB_OTG_FS>;

pub fn usb_driver(p: Peripherals) -> UsbDriver {
    let irq = interrupt::take!(OTG_FS);
    Driver::new_fs(
        p.USB_OTG_FS,
        irq,
        p.PA12,
        p.PA11,
        &mut crate::singleton!([0u8; 256])[..],
    )
}

fn init_periph() -> Peripherals {
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(SYS_CK);
    config.rcc.hclk = Some(HCLK);
    config.rcc.pll1.q_ck = Some(PLL1_QCK);
    // config.rcc.pll2.p_ck = Some(PLL2_PCK);
    // config.rcc.pll2.q_ck = Some(PLL2_QCK);
    // config.rcc.pll2.r_ck = Some(PLL2_RCK);
    // config.rcc.hse = Some(HSE_CLK);

    embassy_stm32::init(config)
}

pub fn init() -> (
    usb::USB,
    usb::Reciever,
    usb::Sender,
) {
    let p = init_periph();
    usb::Console::new(p)
}
