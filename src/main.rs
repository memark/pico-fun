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
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::String;
// use mipidsi::{Builder, interface::SpiInterface, models, options::ColorInversion};
// use mousefood::prelude::*;
// use mousefood::{EmbeddedBackendConfig, embedded_graphics::prelude::DrawTarget};
use panic_probe as _;
use pimoroni_pico_explorer::PicoExplorer;
// use ratatui::{
//     Frame, Terminal,
//     style::{Style, Stylize},
//     widgets::{Block, Paragraph, Wrap},
// };
use rp_pico::{
    Pins,
    hal::{
        Adc, Clock, Sio, Spi, Watchdog,
        clocks::init_clocks_and_plls,
        fugit::RateExtU32,
        gpio::FunctionSpi,
        multicore::{Multicore, Stack},
        pwm::Slices,
    },
    pac::{CorePeripherals, Peripherals},
};

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

    // let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    // let cores = mc.cores();
    // let core1 = &mut cores[1];

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
    // let mut delay = Delay::new(core0.SYST, clocks.system_clock.freq().to_Hz());
    let mut delay = Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());

    // let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    debug!("pins set up");

    //

    // let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    // let cores = mc.cores();
    // let core1 = &mut cores[1];
    // #[allow(static_mut_refs)]
    // let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
    //     let mut core1 = unsafe { pac::CorePeripherals::steal() };
    //     // let mut delay1 = Delay::new(core1.SYST, system_freq_hz);

    //     let mut delay1 = Delay::new(core1.SYST, hz);

    //     delay1.delay_us(500);

    //     loop {
    //         // debug!("From Core 1.");
    //         delay1.delay_ms(500);
    //     }
    // });

    //

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

    let (mut explorer, pins) = PicoExplorer::new(
        p.IO_BANK0,
        p.PADS_BANK0,
        sio.gpio_bank0,
        p.SPI0,
        adc,
        &mut p.RESETS,
        &mut delay,
    );

    // // LCD pins (Pico Explorer)
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

    explorer.screen.clear(Rgb565::BLACK);

    loop {
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
                .draw(&mut explorer.screen)
                .unwrap();
        }

        debug!("...done");
        delay.delay_ms(10);
    }

    //

    if false {
        let mut buzzer = pins.gpio4.into_push_pull_output();

        // Rough ~1 kHz (500 µs high + 500 µs low)
        loop {
            // buzzer.set_high();
            // delay.delay_us(500);
            // buzzer.set_low();
            // delay.delay_us(500);
        }
    } else {
        let mut pwm_slices = Slices::new(p.PWM, &mut p.RESETS);
        let pwm = &mut pwm_slices.pwm2;
        pwm.enable();

        // let freq_hz = 2000;
        // let clk = 125_000_000;
        // let top = clk / freq_hz;
        // pwm0.set_top(top);

        // 2 kHz: freq = clk_sys / (top + 1)
        // with clk_sys = 125 MHz, top ≈ 62_499
        // pwm0.set_top(62_499);
        // pwm0.set_top(100_000);
        // pwm0.enable();

        let freq = 440;

        let top = 125_000_000_u32 / freq / 250;
        let top: u16 = top.try_into().unwrap();

        let channel_a = &mut pwm.channel_a;
        // let buzzer = pins.gpio0;
        let buzzer = pins.gpio4;
        channel_a.output_to(buzzer);

        channel_a.set_duty(1100 / 8);

        pwm.set_div_int(255);
        pwm.set_div_frac(0);

        loop {
            // debug!("From Core 0.");
            // delay.delay_ms(500);
        }

        loop {
            // pwm.set_top(1100);
            // delay.delay_ms(500);

            // pwm.set_top(554);
            // delay.delay_ms(500);
        }
    }

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

// fn set_freq<P: rp_pico::hal::pwm::SliceId>(pwm: &mut pwm::PwmSlice<P>, freq_hz: u32) {
//     let clk = 125_000_000; // clk_sys
//     let top = clk / freq_hz;
//     pwm.set_top(top);
// }

// class Buzzer:
//     def __init__(self, pin):
//         self.pwm = PWM(Pin(pin))

//     def set_tone(self, freq, duty=0.5):
//         if freq < 50.0:  # uh... https://github.com/micropython/micropython/blob/af64c2ddbd758ab6bac0fcca94c66d89046663be/ports/rp2/machine_pwm.c#L105-L119
//             self.pwm.duty_u16(0)
//             return False

//         self.pwm.freq(freq)
//         self.pwm.duty_u16(int(65535 * duty))
//         return True

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp_binary_info::EntryAddr; 5] = [
    rp_binary_info::rp_cargo_bin_name!(),
    rp_binary_info::rp_cargo_version!(),
    rp_binary_info::rp_program_description!(c"Blinky Example"),
    rp_binary_info::rp_cargo_homepage_url!(),
    rp_binary_info::rp_program_build_attribute!(),
];
