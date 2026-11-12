#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![no_std]
#![no_main]

use ch58x_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{Input, Pull};
use hal::peripherals;
use hal::uart::UartTx;
use ch58x_rs_kit::log;
use ch58x_rs_kit::logger::init as init_logger;
use ch58x_rs_kit::sr_led_driver::SrLedDriver;

const LED_COUNT: usize = 48;

// Cyberlamp-light LED channel addresses, decoded from the SM16106SC outputs.
// addr0 is WW1 and addr1 is UV1, so each 16-channel block maps OUT15..OUT0.
// CW: 3, 4, 8, 9, 14, 19, 20, 24, 25, 30, 35, 36, 40, 41, 46
// WW: 0, 7, 10, 13, 15, 16, 23, 26, 29, 31, 32, 39, 42, 45, 47
// UV: 1, 2, 5, 6, 17, 18, 21, 22, 33, 34, 37, 38
// IR: 11, 12, 27, 28, 43, 44
const CW_LED_ADDRESSES: &[usize] = &[3, 4, 8, 9, 14, 19, 20, 24, 25, 30, 35, 36, 40, 41, 46];
const WW_LED_ADDRESSES: &[usize] = &[0, 7, 10, 13, 15, 16, 23, 26, 29, 31, 32, 39, 42, 45, 47];
const UV_LED_ADDRESSES: &[usize] = &[1, 2, 5, 6, 17, 18, 21, 22, 33, 34, 37, 38];
const IR_LED_ADDRESSES: &[usize] = &[11, 12, 27, 28, 43, 44];
const LED_GROUPS: [&[usize]; 4] = [CW_LED_ADDRESSES, WW_LED_ADDRESSES, UV_LED_ADDRESSES, IR_LED_ADDRESSES];
const OE_PWM_PERIOD_US: u64 = 1_000;
const BRIGHTNESS_STEPS: i32 = 40;
const ENCODER_DEADZONE_STEPS: i32 = 2;
const INITIAL_BRIGHTNESS_STEP: i32 = 20;
const BUTTON_DEBOUNCE_PWM_TICKS: u8 = 30;
const BRIGHTNESS_DUTY_US: [u64; BRIGHTNESS_STEPS as usize + 1] = [
    0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 49, 59, 70, 84, 100, 119, 143, 170,
    203, 242, 289, 346, 412, 492, 588, 702, 838, 1_000,
];

fn read_encoder_state(enc_a: &Input<'_, peripherals::PA5>, enc_b: &Input<'_, peripherals::PA4>) -> u8 {
    ((enc_a.is_high() as u8) << 1) | enc_b.is_high() as u8
}

fn decode_encoder_delta(previous: u8, current: u8) -> i32 {
    const DELTAS: [i32; 16] = [
        0, -1, 1, 0, //
        1, 0, 0, -1, //
        -1, 0, 0, 1, //
        0, 1, -1, 0,
    ];

    DELTAS[((previous << 2) | current) as usize]
}

fn brightness_duty_us(brightness_step: i32) -> u64 {
    BRIGHTNESS_DUTY_US[brightness_step.clamp(0, BRIGHTNESS_STEPS) as usize]
}

fn encoder_brightness_step(encoder_step: i32) -> i32 {
    encoder_step.clamp(0, BRIGHTNESS_STEPS)
}

fn fill_led_group_frame(frame: &mut [u8; LED_COUNT], led_addresses: &[usize]) {
    frame.fill(0);
    for &address in led_addresses {
        frame[address] = 0xFF;
    }
}

type LampDriver = SrLedDriver<'static, peripherals::SPI0, LED_COUNT>;

#[embassy_executor::task]
async fn led_bringup_task(
    mut sr_driver: LampDriver,
    pin_a: peripherals::PA5,
    pin_b: peripherals::PA4,
    button_pin: peripherals::PB22,
) -> ! {
    log!("Cyberlamp LED bringup task started");

    let enc_a = Input::new(pin_a, Pull::Up);
    let enc_b = Input::new(pin_b, Pull::Up);
    let button = Input::new(button_pin, Pull::Up);
    let mut encoder_state = read_encoder_state(&enc_a, &enc_b);
    let mut brightness_step = INITIAL_BRIGHTNESS_STEP;
    let mut active_group = 0usize;
    let mut button_was_pressed = button.is_low();
    let mut button_debounce_ticks = 0u8;

    let mut frame = [0u8; LED_COUNT];
    fill_led_group_frame(&mut frame, LED_GROUPS[active_group]);

    sr_driver.enable_output();
    sr_driver
        .write(0, &frame)
        .expect("48 LED bringup frame should fit driver buffer");
    sr_driver.update();

    loop {
        let current_encoder_state = read_encoder_state(&enc_a, &enc_b);
        if current_encoder_state != encoder_state {
            let delta = decode_encoder_delta(encoder_state, current_encoder_state);
            encoder_state = current_encoder_state;
            brightness_step =
                (brightness_step + delta).clamp(-ENCODER_DEADZONE_STEPS, BRIGHTNESS_STEPS + ENCODER_DEADZONE_STEPS);
        }

        let button_is_pressed = button.is_low();
        if button_debounce_ticks > 0 {
            button_debounce_ticks -= 1;
        } else if button_is_pressed && !button_was_pressed {
            active_group = (active_group + 1) % LED_GROUPS.len();
            fill_led_group_frame(&mut frame, LED_GROUPS[active_group]);
            sr_driver
                .write(0, &frame)
                .expect("48 LED group frame should fit driver buffer");
            sr_driver.update();
            button_debounce_ticks = BUTTON_DEBOUNCE_PWM_TICKS;
        }
        button_was_pressed = button_is_pressed;

        let on_us = brightness_duty_us(encoder_brightness_step(brightness_step));
        let off_us = OE_PWM_PERIOD_US - on_us;

        if on_us > 0 {
            sr_driver.enable_output();
            Timer::after(Duration::from_micros(on_us)).await;
        }

        if off_us > 0 {
            sr_driver.disable_output();
            Timer::after(Duration::from_micros(off_us)).await;
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);
    hal::embassy::init();

    let serial = UartTx::new(p.UART0, p.PB7, Default::default()).unwrap();
    init_logger(serial);
    log!("\n\n\nCyberlamp Firmware Started!");

    // Cyberlamp-light SM16106SC chain:
    // SCLK - PA13, DAT - PA14, /OE - PA8, LAT - PA9.
    let sr_driver =
        SrLedDriver::<_, LED_COUNT>::new_from_pins::<false, _, _, _, _>(p.SPI0, p.PA13, p.PA14, p.PA8, p.PA9);

    log!("Shift register LED driver initialized");
    log!("  SPI: SCLK=PA13, DAT=PA14");
    log!("  GPIO: OE=PA8, LAT=PA9");

    // Timer::after(Duration::from_secs(1)).await;

    // sr_driver.test_write(0xFF);
    // sr_driver.test_write(0x00);
    log!("Starting shift register background task...");

    spawner
        .spawn(led_bringup_task(sr_driver, p.PA5, p.PA4, p.PB22))
        .unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
