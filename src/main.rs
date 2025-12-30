#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
//
#![allow(unused, clippy::empty_loop)]

mod dummy_pin;

use core::cell::RefCell;

use crate::dummy_pin::DummyPin;
use cortex_m::{self as _, delay::Delay, prelude::*};
use cortex_m_rt::entry;
use defmt::{debug, info};
use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::{
    mono_font::{
        MonoTextStyleBuilder,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::{BinaryColor, Rgb565},
    prelude::*,
    text::{Alignment, Baseline, Text},
};
use embedded_hal::{delay::DelayNs, spi::MODE_0};
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::{String, Vec};
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig};
use panic_probe as _;
use ratatui::widgets::Borders;
use rp_pico::{
    Pins,
    hal::{
        self, Adc, Clock, Sio, Spi, Watchdog,
        clocks::init_clocks_and_plls,
        fugit::RateExtU32,
        gpio::{
            self, FunctionI2C, FunctionSioOutput, FunctionSpi, Pin, PullNone,
            bank0::{Gpio16, Gpio17, Gpio18, Gpio19},
        },
        multicore::{Multicore, Stack},
        pwm::Slices,
        spi::Enabled,
    },
    pac::{CorePeripherals, Peripherals, SPI0},
};

const XTAL_FREQ_HZ: u32 = 12_000_000_u32;

// static mut CORE1_STACK: Stack<4096> = Stack::new();

#[global_allocator]
static HEAP: Heap = Heap::empty();

fn core1_task() {
    loop {}
}

#[entry]
fn main() -> ! {
    unsafe {
        embedded_alloc::init!(HEAP, 120 * 1024);
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

    if false {
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
                Builder, Display,
                interface::SpiInterface,
                models::{self, Model},
                options::{Orientation, RefreshOrder, Rotation},
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

        display.clear(Rgb565::BLACK).unwrap();

        {
            use heapless::Vec;
            use ratatui::{prelude::*, symbols::*, widgets::*};

            let backend = EmbeddedBackend::new(&mut display, EmbeddedBackendConfig::default());
            debug!("backend initialized");

            let mut terminal = Terminal::new(backend).unwrap();
            debug!("terminal initialized");

            debug!("drawing...");

            terminal
                .draw(|frame| {
                    // let text = "Ratatui on Raspberry Pi Pico!";
                    // let paragraph = Paragraph::new(text.blue().bold())
                    //     .wrap(Wrap { trim: true })
                    //     .centered();
                    // let bordered_block = Block::bordered()
                    //     .padding(Padding::uniform(2))
                    //     .border_style(Style::new().yellow().bold())
                    //     .title("Mousefood");
                    // frame.render_widget(paragraph.block(bordered_block), frame.area());

                    // let line_gauge = LineGauge::default()
                    //     .block(Block::bordered().title("Progress"))
                    //     .filled_style(Style::new().white().on_black().bold())
                    //     .filled_symbol(symbols::line::THICK_HORIZONTAL)
                    //     .ratio(0.4);
                    // frame.render_widget(line_gauge, frame.area());

                    let datasets = [
                        Dataset::default()
                            .name("Heavy")
                            .marker(Marker::Dot)
                            .graph_type(GraphType::Scatter)
                            .style(Style::new().yellow())
                            .data(&HEAVY_PAYLOAD_DATA),
                        Dataset::default()
                            .name("Medium".underlined())
                            .marker(Marker::Braille)
                            .graph_type(GraphType::Scatter)
                            .style(Style::new().magenta())
                            .data(&MEDIUM_PAYLOAD_DATA),
                        Dataset::default()
                            .name("Small")
                            .marker(Marker::Dot)
                            .graph_type(GraphType::Scatter)
                            .style(Style::new().cyan())
                            .data(&SMALL_PAYLOAD_DATA),
                    ];

                    let chart = Chart::new(datasets.to_vec())
                        .block(
                            Block::bordered().title(
                                Line::from("Mousefood - Ratatui on Pi Pico RP2040")
                                    .cyan()
                                    .bold()
                                    .centered(),
                            ),
                        )
                        .x_axis(
                            Axis::default()
                                .title("Year")
                                .bounds([1960., 2020.])
                                .style(Style::default().fg(Color::Gray))
                                .labels(["1960", "1990", "2020"]),
                        )
                        .y_axis(
                            Axis::default()
                                .title("Cost")
                                .bounds([0., 75000.])
                                .style(Style::default().fg(Color::Gray))
                                .labels(["0", "37 500", "75 000"]),
                        )
                        .hidden_legend_constraints((
                            Constraint::Ratio(1, 2),
                            Constraint::Ratio(1, 2),
                        ));

                    frame.render_widget(chart, frame.area());
                })
                .unwrap();

            debug!("...done");
        }
    }

    {
        use core::fmt::Write;
        use embedded_hal_bus::i2c::*;
        use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

        let sda = pins
            .gpio4
            .into_pull_up_input()
            .into_function::<FunctionI2C>();
        let scl = pins
            .gpio5
            .into_pull_up_input()
            .into_function::<FunctionI2C>();

        let i2c = hal::I2C::i2c0(
            p.I2C0,
            sda,
            scl,
            400.kHz(),
            &mut p.RESETS,
            clocks.system_clock.freq(),
        );

        // let csd = CriticalSectionDevice::new(bus);
        let rc = RefCell::new(i2c);
        let rcd = RefCellDevice::new(&rc);

        let interface = I2CDisplayInterface::new(rcd);

        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            // .into_terminal_mode();
            .into_buffered_graphics_mode();

        display.init().unwrap();
        // display.clear().unwrap();

        debug!("display initialized");

        debug!("drawing...");

        // for c in 97..123 {
        //     let cc = &[c];
        //     let _ = display.write_str(unsafe { str::from_utf8_unchecked(cc) });
        // }
        // for c in 65..91 {
        //     let cc = &[c];
        //     let _ = display.write_str(unsafe { str::from_utf8_unchecked(cc) });

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
        // }

        debug!("...done");
    }

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

const HEAVY_PAYLOAD_DATA: [(f64, f64); 9] = [
    (1965., 8200.),
    (1967., 5400.),
    (1981., 65400.),
    (1989., 30800.),
    (1997., 10200.),
    (2004., 11600.),
    (2014., 4500.),
    (2016., 7900.),
    (2018., 1500.),
];

const MEDIUM_PAYLOAD_DATA: [(f64, f64); 29] = [
    (1963., 29500.),
    (1964., 30600.),
    (1965., 177_900.),
    (1965., 21000.),
    (1966., 17900.),
    (1966., 8400.),
    (1975., 17500.),
    (1982., 8300.),
    (1985., 5100.),
    (1988., 18300.),
    (1990., 38800.),
    (1990., 9900.),
    (1991., 18700.),
    (1992., 9100.),
    (1994., 10500.),
    (1994., 8500.),
    (1994., 8700.),
    (1997., 6200.),
    (1999., 18000.),
    (1999., 7600.),
    (1999., 8900.),
    (1999., 9600.),
    (2000., 16000.),
    (2001., 10000.),
    (2002., 10400.),
    (2002., 8100.),
    (2010., 2600.),
    (2013., 13600.),
    (2017., 8000.),
];

const SMALL_PAYLOAD_DATA: [(f64, f64); 23] = [
    (1961., 118_500.),
    (1962., 14900.),
    (1975., 21400.),
    (1980., 32800.),
    (1988., 31100.),
    (1990., 41100.),
    (1993., 23600.),
    (1994., 20600.),
    (1994., 34600.),
    (1996., 50600.),
    (1997., 19200.),
    (1997., 45800.),
    (1998., 19100.),
    (2000., 73100.),
    (2003., 11200.),
    (2008., 12600.),
    (2010., 30500.),
    (2012., 20000.),
    (2013., 10600.),
    (2013., 34500.),
    (2015., 10600.),
    (2018., 23100.),
    (2019., 17300.),
];
