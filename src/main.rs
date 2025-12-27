#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
//
#![allow(unused, clippy::empty_loop)]

mod dummy_pin;

use crate::dummy_pin::DummyPin;
use cortex_m::{self as _, delay::Delay, prelude::*};
use cortex_m_rt::entry;
use defmt::{debug, info};
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::{delay::DelayNs, spi::MODE_0};
// use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::String;
// use mipidsi::{Builder, interface::SpiInterface, models, options::ColorInversion};
// use mousefood::prelude::*;
// use mousefood::{EmbeddedBackendConfig, embedded_graphics::prelude::DrawTarget};
use panic_probe as _;
// use ratatui::{
//     Frame, Terminal,
//     style::{Style, Stylize},
//     widgets::{Block, Paragraph, Wrap},
// };
use rp_pico::{
    Pins,
    hal::{
        self, Adc, Clock, Sio, Spi, Watchdog,
        clocks::init_clocks_and_plls,
        fugit::RateExtU32,
        gpio::{
            self, FunctionSioOutput, FunctionSpi, Pin, PullNone,
            bank0::{Gpio16, Gpio17, Gpio18, Gpio19},
        },
        multicore::{Multicore, Stack},
        pwm::Slices,
        spi::Enabled,
    },
    pac::{CorePeripherals, Peripherals, SPI0},
};
use st7789::ST7789;

const XTAL_FREQ_HZ: u32 = 12_000_000_u32;

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[global_allocator]
static HEAP: Heap = Heap::empty();

fn core1_task() {
    loop {}
}

#[entry]
fn main() -> ! {
    unsafe {
        embedded_alloc::init!(HEAP, 100 * 1024);
    }

    info!("Application started");

    let mut p = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();
    let mut sio = Sio::new(p.SIO);

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

    let mut adc = Adc::new(p.ADC, &mut p.RESETS);
    let mut delay = Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());

    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    let spi_sclk = pins.gpio18.reconfigure::<FunctionSpi, PullNone>();
    let spi_mosi = pins.gpio19.reconfigure::<FunctionSpi, PullNone>();
    let spi_screen = Spi::<_, _, _, 8>::new(p.SPI0, (spi_mosi, spi_sclk)).init(
        &mut p.RESETS,
        125u32.MHz(),
        16u32.MHz(),
        MODE_0,
    );

    let dc = pins.gpio16.reconfigure::<FunctionSioOutput, PullNone>();
    let cs = pins.gpio17.reconfigure::<FunctionSioOutput, PullNone>();
    let spii_screen = SPIInterface::new(spi_screen, dc, cs);

    let mut screen = ST7789::new(spii_screen, Some(DummyPin), Some(DummyPin), 240, 240);

    screen.init(&mut delay).unwrap();
    screen
        .set_orientation(st7789::Orientation::Portrait)
        .unwrap();

    // let dc = pins.gpio16.into_push_pull_output();
    // let cs = pins.gpio17.into_push_pull_output();
    // let sck = pins.gpio18.into_function::<FunctionSpi>();
    // let mosi = pins.gpio19.into_function::<FunctionSpi>();
    // let miso = pins.gpio4.into_function::<FunctionSpi>(); // dummy, not wired
    // let rst = pins.gpio20.into_push_pull_output(); // not wired -> acts as "no reset"

    // // SPI0 in MODE_0 is what ST7789 expects
    // let spi = Spi::<_, _, _>::new(p.SPI0, (mosi, miso, sck)).init(
    //     &mut p.RESETS,
    //     clocks.peripheral_clock.freq(),
    //     32.MHz(),
    //     embedded_hal::spi::MODE_0,
    // );

    // debug!("spi initialized");

    // static mut BUF: [u8; 1024] = [0; 1024];

    // #[allow(static_mut_refs)]
    // let buffer = unsafe { &mut BUF };

    // // let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss1, 60_000_000_u32, Mode::Mode0).unwrap();
    // let spi_device = ExclusiveDevice::new_no_delay(spi, DummyPin).unwrap();
    // // let mut buffer = [0_u8; 512];
    // let di = SpiInterface::new(spi_device, dc, buffer);
    // // let mut delay = Delay::new();
    // let mut display = Builder::new(models::ST7789, di)
    //     .display_size(240, 240)
    //     .invert_colors(ColorInversion::Inverted)
    //     .init(&mut delay)
    //     .unwrap();

    // debug!("display initialized");

    // let backend = EmbeddedBackend::new(&mut display, EmbeddedBackendConfig::default());
    // debug!("backend initialized");

    // let mut terminal = Terminal::new(backend).unwrap();
    // debug!("terminal initialized");

    // screen.clear(Rgb565::BLACK);

    debug!("drawing...");

    {
        // terminal.draw(draw).unwrap();
    }

    {
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::GREEN)
            .background_color(Rgb565::BLACK)
            .build();
        Text::with_alignment("Hello", Point::new(20, 30), style, Alignment::Left)
            .draw(&mut screen)
            .unwrap();
        Text::with_alignment("World", Point::new(20, 50), style, Alignment::Left)
            .draw(&mut screen)
            .unwrap();
    }

    debug!("...done");

    loop {}
}

// fn draw(frame: &mut Frame) {
//     let text = "Ratatui on embedded devices!";
//     let paragraph = Paragraph::new(text.dark_gray()).wrap(Wrap { trim: true });
//     let bordered_block = Block::bordered()
//         .border_style(Style::new().yellow())
//         .title("Mousefood");
//     frame.render_widget(paragraph.block(bordered_block), frame.area());
// }

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp_binary_info::EntryAddr; 5] = [
    rp_binary_info::rp_cargo_bin_name!(),
    rp_binary_info::rp_cargo_version!(),
    rp_binary_info::rp_program_description!(c"Blinky Example"),
    rp_binary_info::rp_cargo_homepage_url!(),
    rp_binary_info::rp_program_build_attribute!(),
];
