//! SPDX-License-Identifier: MIT OR Apache-2.0
//!
//! Copyright (c) 2021–2024 The rp-rs Developers
//! Copyright (c) 2021 rp-rs organization
//! Copyright (c) 2025 Raspberry Pi Ltd.
//!
//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the rp2040 and rp235x.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.

#![no_std]
#![no_main]
#![allow(unused, clippy::empty_loop)]

use defmt::*;
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

#[cfg(target_arch = "riscv32")]
use panic_halt as _;
#[cfg(target_arch = "arm")]
use panic_probe as _;

// Alias for our HAL crate
use hal::entry;

use pimoroni_pico_explorer::{
    PicoExplorer,
    hal::{Adc, Sio},
};
#[cfg(rp2350)]
use rp235x_hal as hal;

#[cfg(rp2040)]
use rp2040_hal as hal;
use rp2040_hal::{clocks::ClockSource, pac};

// use arrayvec::ArrayString;

// use bsp::entry;
// use bsp::hal;
// use rp_pico as bsp;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[unsafe(link_section = ".boot2")]
#[used]
#[cfg(rp2040)]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
#[cfg(rp2350)]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp2040 and rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    info!("Program start");

    // let mut p = hal::pac::Peripherals::take().unwrap();
    let mut p = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(p.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.get_freq().to_Hz());
    let mut adc = Adc::new(p.ADC, &mut p.RESETS);
    let sio = Sio::new(p.SIO);

    let (mut explorer, pins) = PicoExplorer::new(
        p.IO_BANK0,
        p.PADS_BANK0,
        sio.gpio_bank0,
        p.SPI0,
        adc,
        &mut p.RESETS,
        &mut delay,
    );

    // #[cfg(rp2040)]
    // let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // #[cfg(rp2350)]
    // let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // let sio = hal::Sio::new(pac.SIO);

    // let pins = hal::gpio::Pins::new(
    //     pac.IO_BANK0,
    //     pac.PADS_BANK0,
    //     sio.gpio_bank0,
    //     &mut pac.RESETS,
    // );

    // let mut led_pin_0 = pins.gpio25.into_push_pull_output();
    // let mut led_pin_1 = pins.gpio18.into_push_pull_output();
    // let mut led_pin_2 = pins.gpio19.into_push_pull_output();
    // let mut led_pin_3 = pins.gpio20.into_push_pull_output();

    // loop {
    //     led_pin_0.set_high().unwrap();
    //     timer.delay_ms(200);

    //     led_pin_1.set_high().unwrap();
    //     timer.delay_ms(200);

    //     led_pin_3.set_low().unwrap();
    //     timer.delay_ms(200);

    //     led_pin_2.set_high().unwrap();
    //     timer.delay_ms(200);

    //     led_pin_1.set_low().unwrap();
    //     timer.delay_ms(200);

    //     led_pin_3.set_high().unwrap();
    //     timer.delay_ms(200);

    //     led_pin_2.set_low().unwrap();
    //     timer.delay_ms(200);
    // }

    // let mut buf = ArrayString::<100>::new();

    // writeln!(&mut buf, "Hello World!").unwrap();

    // let buf: &str = "Hello world!";

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::GREEN)
        .background_color(Rgb565::BLACK)
        .build();
    Text::with_alignment("Hello", Point::new(20, 30), style, Alignment::Left)
        .draw(&mut explorer.screen)
        .unwrap();
    Text::with_alignment("World", Point::new(20, 50), style, Alignment::Left)
        .draw(&mut explorer.screen)
        .unwrap();

    loop {}
}

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp_binary_info::EntryAddr; 5] = [
    rp_binary_info::rp_cargo_bin_name!(),
    rp_binary_info::rp_cargo_version!(),
    rp_binary_info::rp_program_description!(c"Blinky Example"),
    rp_binary_info::rp_cargo_homepage_url!(),
    rp_binary_info::rp_program_build_attribute!(),
];
