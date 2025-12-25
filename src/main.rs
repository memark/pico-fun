#![no_std]
#![no_main]
//
#![allow(unused, clippy::empty_loop)]

use cortex_m::delay::Delay;
use cortex_m::prelude::*;
use defmt::{debug, info};
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use hal::entry;
use heapless::String;
use panic_probe as _;
use pimoroni_pico_explorer::{
    PicoExplorer,
    hal::{Adc, Sio},
};
use rp2040_hal::{
    self as hal, Watchdog,
    clocks::{ClockSource, init_clocks_and_plls},
    gpio::{FunctionPwm, FunctionSio, Pin, PullDown, bank0::Gpio0},
    multicore::{Multicore, Stack},
    pac::{self, CorePeripherals, Peripherals},
    pwm::{self, InputHighRunning, Slices},
};
// use rtt_target::{rprintln, rtt_init_print};

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_FREQ_HZ: u32 = 12_000_000_u32;

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task() {
    loop {}
}

#[entry]
fn main() -> ! {
    // rtt_init_print!();
    // rprintln!("Hello from RTT");

    info!("Application started");

    let mut pac = Peripherals::take().unwrap();
    let core0 = CorePeripherals::take().unwrap();
    let mut sio = Sio::new(pac.SIO);

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let hz = clocks.system_clock.get_freq().to_Hz();

    let mut delay = Delay::new(core0.SYST, hz);
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let (mut explorer, pins) = PicoExplorer::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        pac.SPI0,
        adc,
        &mut pac.RESETS,
        &mut delay,
    );

    //

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    #[allow(static_mut_refs)]
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let mut core1 = unsafe { pac::CorePeripherals::steal() };
        // let mut delay1 = Delay::new(core1.SYST, system_freq_hz);

        let mut delay1 = Delay::new(core1.SYST, hz);

        delay1.delay_us(500);

        loop {
            debug!("From Core 1.");
            delay1.delay_ms(500);
        }
    });

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
    Text::with_alignment("Loading... 1", Point::new(20, 30), style, Alignment::Left)
        .draw(&mut explorer.screen)
        .unwrap();

    //

    if false {
        let mut buzzer = pins.gpio4.into_push_pull_output();

        // Rough ~1 kHz (500 µs high + 500 µs low)
        loop {
            buzzer.set_high();
            delay.delay_us(500);
            buzzer.set_low();
            delay.delay_us(500);
        }
    } else {
        let mut pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
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

        Text::with_alignment("Loading... 2", Point::new(20, 50), style, Alignment::Left)
            .draw(&mut explorer.screen)
            .unwrap();

        loop {
            debug!("From Core 0.");
            delay.delay_ms(500);
        }

        loop {
            // pwm0.set_top(top);
            // delay.delay_ms(500);

            // pwm0.set_top(top / 2);
            // delay.delay_ms(500);

            pwm.set_top(1100);
            delay.delay_ms(500);

            pwm.set_top(554);
            delay.delay_ms(500);
        }
    }

    // // let mut pin_0 = pins.gpio0.into_push_pull_output();
    // let buzzer_pin = pins.gpio0.into_function::<FunctionPwm>();

    // let pwm_slices = Slices::new(p.PWM, &mut p.RESETS);

    // // let mut pwm = pwm_slices.pwm0;
    // // pwm.set_ph_correct();
    // // pwm.enable();

    // // let pwm = pwm.into_mode::<InputHighRunning>();

    // // // Channel A for even gpio pin outputs
    // // let mut channel_a = pwm.channel_a;
    // // let channel_pin_a = channel_a.output_to(pins.gpio0);

    // // channel_a.set_duty_cycle(0x00ff);
    // // let max_duty_cycle = channel_a.max_duty_cycle();
    // // channel_a.set_inverted(); // Invert the output
    // // channel_a.clr_inverted(); // Don't invert the output

    // let mut pwm = pwm_slices.pwm0;
    // pwm.set_ph_correct(); // optional but nicer for audio
    // pwm.enable();

    // let mut ch = &mut pwm.channel_a;
    // ch.output_to(buzzer_pin);

    // // let freq_hz = 440;
    // // let clk = 125_000_000_u32; // clk_sys
    // // let top = clk / freq_hz;
    // // pwm.set_top(top as u16);

    // pwm.set_top(25000);

    //

    Text::with_alignment("...done!", Point::new(20, 50), style, Alignment::Left)
        .draw(&mut explorer.screen)
        .unwrap();

    loop {}
}

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
