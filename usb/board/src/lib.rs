#![cfg_attr(not(test), no_std)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![feature(type_alias_impl_trait)]
#![crate_type = "staticlib"]


#[macro_export]
macro_rules! singleton {
    ($val:expr) => {{
        use static_cell::StaticCell;
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

#[cfg(feature = "matekh743")]
pub mod matekh743;
#[cfg(feature = "matekh743")]
pub use matekh743 as hal;

#[cfg(feature = "f407-disco")]
pub mod disco407;

#[cfg(feature = "f407-disco")]
pub use disco407 as hal;

pub use embassy_stm32::Peripherals as Peripherals;
