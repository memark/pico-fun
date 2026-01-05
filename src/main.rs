#![no_std]
#![no_main]
//
#![allow(clippy::empty_loop)]

use cortex_m::{self as _, delay::Delay};
use cortex_m_rt::entry;
use defmt::{debug, info};
use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::{
    image::Image,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::{delay::DelayNs, spi::MODE_0};
use embedded_hal_02::digital::v2::InputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use fugit::RateExtU32 as _;
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig};
use panic_probe as _;
use rp235x_hal::{
    Adc, Clock as _, I2C, Sio, Spi, Timer, Watchdog, block,
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, FunctionSioOutput, FunctionSpi, Pins, PullNone},
    pac::Peripherals,
};
use tinytga::Tga;

const XTAL_FREQ_HZ: u32 = 12_000_000_u32;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: block::ImageDef = block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    info!("application started");

    unsafe {
        embedded_alloc::init!(HEAP, 120 * 1024);
    }

    let mut p = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let sio = Sio::new(p.SIO);

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let _adc = Adc::new(p.ADC, &mut p.RESETS);
    let mut delay = Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());
    let mut timer = Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    if true {
        let spi_sclk = pins.gpio18.reconfigure::<FunctionSpi, PullNone>();
        let spi_mosi = pins.gpio19.reconfigure::<FunctionSpi, PullNone>();
        let spi_bus = Spi::<_, _, _, 8>::new(p.SPI0, (spi_mosi, spi_sclk)).init(
            &mut p.RESETS,
            125u32.MHz(),
            16u32.MHz(),
            MODE_0,
        );

        let dc = pins.gpio16.reconfigure::<FunctionSioOutput, PullNone>();
        let cs = pins.gpio17.reconfigure::<FunctionSioOutput, PullNone>();

        let mut display = {
            use mipidsi::{
                Builder,
                interface::SpiInterface,
                models,
                options::{Orientation, Rotation},
            };

            let device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

            let buffer = {
                static mut BUF: [u8; 1024] = [0; 1024];
                #[allow(static_mut_refs)]
                unsafe {
                    &mut BUF
                }
            };

            let di = SpiInterface::new(device, dc, buffer);

            Builder::new(models::ST7789, di)
                .display_size(240, 240)
                .orientation(Orientation::default().rotate(Rotation::Deg270))
                .init(&mut delay)
                .unwrap()
        };

        // Colors are inverted...
        display.clear(RgbColor::WHITE).unwrap();

        if false {
            use ratatui::{prelude::*, widgets::*};

            let backend = EmbeddedBackend::new(&mut display, EmbeddedBackendConfig::default());
            debug!("backend initialized");

            let mut terminal = Terminal::new(backend).unwrap();
            debug!("terminal initialized");

            debug!("drawing...");

            terminal
                .draw(|frame| {
                    let text = "Ratatui on Raspberry Pi Pico!";
                    let paragraph = Paragraph::new(text.blue().bold())
                        .wrap(Wrap { trim: true })
                        .centered();
                    let bordered_block = Block::bordered()
                        .padding(Padding::uniform(2))
                        .border_style(Style::new().yellow().bold())
                        .title("Mousefood");
                    frame.render_widget(paragraph.block(bordered_block), frame.area());
                })
                .unwrap();

            debug!("...done");
        }

        if true {
            let tga = Tga::from_slice(include_bytes!("../assets/rust-pride.tga")).unwrap();
            let image = Image::new(&tga, Point::zero());

            image.draw(&mut display).unwrap();
            image
                .translate(Point::new(100, 100))
                .draw(&mut display)
                .unwrap();
        }
    }

    if true {
        use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

        let sda = pins
            .gpio20
            .into_pull_up_input()
            .into_function::<FunctionI2C>();
        let scl = pins
            .gpio21
            .into_pull_up_input()
            .into_function::<FunctionI2C>();

        let i2c = I2C::i2c0(
            p.I2C0,
            sda,
            scl,
            400.kHz(),
            &mut p.RESETS,
            clocks.system_clock.freq(),
        );

        let interface = I2CDisplayInterface::new(i2c);

        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate180)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        let _ = display.clear(BinaryColor::Off);
        display.flush().unwrap();

        debug!("display initialized");

        {
            debug!("drawing...");

            let text_style = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(BinaryColor::On)
                .build();

            Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();

            debug!("...done");
        }
    }

    if true {
        use i2c_character_display::*;

        let sda = pins
            .gpio6
            .into_pull_up_input()
            .into_function::<FunctionI2C>();
        let scl = pins
            .gpio7
            .into_pull_up_input()
            .into_function::<FunctionI2C>();

        let i2c = I2C::i2c1(
            p.I2C1,
            sda,
            scl,
            400.kHz(),
            &mut p.RESETS,
            clocks.system_clock.freq(),
        );

        let mut lcd = CharacterDisplayPCF8574T::new(i2c, LcdDisplayType::Lcd16x2, delay);

        defmt::unwrap!((|| {
            lcd.init()?;
            debug!("display initialized");

            debug!("drawing...");
            lcd.clear()?.home()?.print("Hello, LCD!")?;
            debug!("...done");

            Ok::<_, CharacterDisplayError<_>>(())
        })());

        let _a = pins.gpio12.into_pull_up_input();
        let b = pins.gpio13.into_pull_up_input();
        let _x = pins.gpio14.into_pull_up_input();
        let y = pins.gpio15.into_pull_up_input();

        loop {
            if b.is_low().unwrap() {
                debug!("B pressed");
                let _ = lcd.scroll_display_left();
                DelayNs::delay_ms(&mut timer, 200);
            }
            if y.is_low().unwrap() {
                debug!("Y pressed");
                let _ = lcd.scroll_display_right();
                DelayNs::delay_ms(&mut timer, 200);
            }
        }
    }

    loop {}
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 Template"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];
