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
use embedded_hal_02::digital::v2::{InputPin, OutputPin as _};
use embedded_hal_bus::spi::ExclusiveDevice;
use fugit::RateExtU32 as _;
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig};
use panic_probe as _;
use rp235x_hal::{
    Adc, Clock as _, I2C, Sio, Spi, Timer, Watchdog, block,
    clocks::{ClocksManager, init_clocks_and_plls},
    gpio::{
        FunctionI2C, FunctionPio0, FunctionSio, FunctionSioOutput, FunctionSpi, Pin, Pins,
        PullNone, PullUp, SioInput, bank0::*,
    },
    pac::{I2C1, Peripherals, RESETS},
    pio::PIOExt as _,
    timer::CopyableTimer0,
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

    let _a = pins.gpio12.into_pull_up_input();
    let b = pins.gpio13.into_pull_up_input();
    let _x = pins.gpio14.into_pull_up_input();
    let y = pins.gpio15.into_pull_up_input();

    // SPI0, MIPIDSI, ST7789
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

        // Ratatui
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

        // Tinytga
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

    // I2C0, SSD1306
    if true {
        use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

        let sda = pins.gpio20.reconfigure();
        let scl = pins.gpio21.reconfigure();
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

    // Try W blinky
    if true {
        // let pwr = Output::new(p.PIN_23, Level::Low);
        let mut pwr = pins.gpio23.into_push_pull_output();
        pwr.set_low().unwrap();

        // let cs = Output::new(p.PIN_25, Level::High);
        let mut cs = pins.gpio25.into_push_pull_output();
        cs.set_high().unwrap();

        // let mut pio = Pio::new(p.PIO0, Irqs);
        let (mut pio0, sm0, _sm1, _sm2, _sm3) = p.PIO0.split(&mut p.RESETS);

        let _mosi = pins.gpio24.reconfigure::<FunctionSpi, PullNone>();
        let _sck = pins.gpio29.reconfigure::<FunctionSpi, PullNone>();

        // let spi = PioSpi::new(
        //     &mut pio.common,
        //     pio.sm0,
        //     RM2_CLOCK_DIVIDER,
        //     pio.irq0,
        //     cs,
        //     p.PIN_24,
        //     p.PIN_29,
        //     p.DMA_CH0,
        // );

        //         let spi_sclk = pins.gpio18.reconfigure::<FunctionSpi, PullNone>();
        // let spi_mosi = pins.gpio19.reconfigure::<FunctionSpi, PullNone>();
        // let spi_bus = Spi::<_, _, _, 8>::new(p.SPI0, (spi_mosi, spi_sclk)).init(
        //     &mut p.RESETS,
        //     125u32.MHz(),
        //     16u32.MHz(),
        //     MODE_0,
        // );

        // let spi = Spi::<_, _, _, 8>::new(p.SPI0, (_mosi, _sck)).init(
        //     &mut p.RESETS,
        //     /* spi_freq_hz */ 8_000_000u32,
        //     &embedded_hal::spi::MODE_0,
        // );

        // let (_net_device, mut control) = setup_radio(&spawner, pwr, spi).await;

        // let one_sec = Duration::from_secs(1);
        // loop {
        //     info!("led on!");
        //     control.gpio_set(0, true).await;
        //     Timer::after(delay).await;

        //     info!("led off!");
        //     control.gpio_set(0, false).await;
        //     Timer::after(delay).await;
        // }
    }

    // I2C, LCD PCF8574T
    if true {
        let lcd = setup_lcd16xx2(pins, clocks, delay, &mut timer, p.I2C1, &mut p.RESETS);

        loop {
            if b.is_low().unwrap() {
                debug!("B pressed");
                let _ = lcd.scroll_display_left();
                DelayNs::delay_ms(timer, 200);
            }
            if y.is_low().unwrap() {
                debug!("Y pressed");
                let _ = lcd.scroll_display_right();
                DelayNs::delay_ms(timer, 200);
            }
        }
    }

    loop {}
}

fn setup_lcd16xx2(
    pins: Pins,
    clocks: ClocksManager,
    delay: Delay,
    timer: &mut Timer<CopyableTimer0>,
    i2c1: I2C1,
    resets: &mut RESETS,
) -> () {
    use i2c_character_display::*;

    let sda = pins.gpio6.reconfigure();
    let scl = pins.gpio7.reconfigure();
    let i2c = I2C::i2c1(
        i2c1,
        sda,
        scl,
        400.kHz(),
        resets,
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

    lcd
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
