#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![no_std]
#![no_main]

use ch58x_hal as hal;
use core::fmt::Write as _;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_local::delay::DelayNs;
use hal::delay::CycleDelay;
use hal::gpio::{Input, Pull};
use hal::uart::UartTx;
use hal::{ble, peripherals};
use ch58x_rs_kit::ble_periph::current_time_service::current_time_init;
use ch58x_rs_kit::ble_periph::{common_init, devinfo_init, peripheral_keep_running};
use ch58x_rs_kit::ble_periph::nus;
use cyberlamp_rs::protocol::{Command, Decoder, Reply, MAX_LEVEL};
use ch58x_rs_kit::logger::init as init_logger;
use ch58x_rs_kit::settings_store::{SettingsStore, PAYLOAD_LEN};
use ch58x_rs_kit::sr_led_driver::SrLedDriver;
use ch58x_rs_kit::{get_configured_rtc, log, tmos_mainloop};

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
/// Spread mode lights the first N LEDs of these lists. Farthest-point order, computed from the
/// LED positions in cyberlamp-light.kicad_pcb: each next LED is the one farthest from all LEDs
/// already lit, so any count is spread over the three sectors of the lamp. CW starts near the
/// centre, WW starts far from the first CW LED.
const CW_SPREAD_ORDER: [usize; 15] = [25, 35, 9, 30, 14, 41, 19, 4, 46, 24, 20, 3, 36, 8, 40];
const WW_SPREAD_ORDER: [usize; 15] = [32, 7, 29, 13, 39, 10, 42, 45, 0, 16, 26, 15, 47, 23, 31];
const OE_PWM_PERIOD_US: u64 = 1_000;
/// On-phases shorter than this are busy-waited: the embassy timer only ticks every ~30.5 us.
const BUSY_WAIT_THRESHOLD_US: u64 = 120;
const BRIGHTNESS_STEPS: i32 = MAX_LEVEL as i32;
const ENCODER_DEADZONE_STEPS: i32 = 2;
const INITIAL_BRIGHTNESS_STEP: i32 = 20;
/// Cool/warm mix: 0 = only cool white, MIX_STEPS = only warm white.
const MIX_STEPS: i32 = MAX_LEVEL as i32;
const INITIAL_MIX_STEP: i32 = MIX_STEPS / 2;
const BUTTON_DEBOUNCE_PWM_TICKS: u8 = 30;
/// Settings are written to flash once they have been stable for this long.
const SAVE_DELAY: Duration = Duration::from_secs(2);
/// DataFlash offset of the settings page (BLE SNV is disabled, so the area is free).
const SETTINGS_FLASH_ADDR: u32 = 0;
/// v1: [version, mode, mix, white, uv, ir]
/// v2: [version, mode | spread << 2, mix, white, uv, ir] (all levels are 0..=40, 6 bits)
const SETTINGS_VERSION: u8 = 2;
const BRIGHTNESS_DUTY_US: [u64; BRIGHTNESS_STEPS as usize + 1] = [
    0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 29, 35, 41, 49, 59, 70, 84, 100, 119, 143, 170,
    203, 242, 289, 346, 412, 492, 588, 702, 838, 1_000,
];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum LampMode {
    /// Cool + warm white; knob = colour temperature, button + knob = brightness.
    White = 0,
    Uv = 1,
    Ir = 2,
    /// Like white, but /OE stays on (no software PWM): brightness = number of lit LEDs,
    /// spread over the lamp. Avoids the PWM flicker caused by BLE interrupts.
    Spread = 3,
}

impl LampMode {
    const COUNT: usize = 4;

    fn next(self) -> Self {
        match self {
            LampMode::White => LampMode::Spread,
            LampMode::Spread => LampMode::Uv,
            LampMode::Uv => LampMode::Ir,
            LampMode::Ir => LampMode::White,
        }
    }

    fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(LampMode::White),
            1 => Some(LampMode::Uv),
            2 => Some(LampMode::Ir),
            3 => Some(LampMode::Spread),
            _ => None,
        }
    }

    fn name(self) -> &'static str {
        match self { Self::White => "white", Self::Uv => "uv", Self::Ir => "ir", Self::Spread => "spread" }
    }

    /// Knob alone sets the colour mix, button + knob the brightness.
    fn is_white(self) -> bool {
        matches!(self, Self::White | Self::Spread)
    }
}

struct LampRequest { session: nus::Session, command: Command }
static LAMP_COMMANDS: Channel<CriticalSectionRawMutex, LampRequest, 1> = Channel::new();
static LAMP_REPLIES: Channel<CriticalSectionRawMutex, Reply, 1> = Channel::new();

#[embassy_executor::task]
async fn nus_console() {
    let mut decoder = Decoder::new();
    let mut session = None;
    loop {
        let packet = nus::receive().await;
        if !packet.session.is_current() { continue; }
        if session != Some(packet.session) {
            decoder.reset();
            session = Some(packet.session);
        }
        for &byte in packet.bytes() {
            if !packet.session.is_current() { break; }
            if let Some(command) = decoder.push(byte) {
                let reply = match command {
                    Ok(command) => {
                        LAMP_COMMANDS.send(LampRequest { session: packet.session, command }).await;
                        LAMP_REPLIES.receive().await
                    }
                    Err(error) => {
                        let mut reply = Reply::new();
                        reply.write_str(error.message()).unwrap();
                        reply
                    }
                };
                if let Err(error) = nus::send(packet.session, reply.bytes()).await {
                    log!("NUS reply not sent: {:?}", error);
                }
            }
        }
    }
}

/// Everything that survives a power cycle.
#[derive(Clone, Copy, PartialEq, Eq)]
struct LampSettings {
    mode: LampMode,
    mix_step: i32,
    /// Per-mode encoder brightness, including the deadzone overshoot.
    brightness_step: [i32; LampMode::COUNT],
}

impl LampSettings {
    const DEFAULT: Self = Self {
        mode: LampMode::White,
        mix_step: INITIAL_MIX_STEP,
        brightness_step: [INITIAL_BRIGHTNESS_STEP; LampMode::COUNT],
    };

    fn encode(&self) -> [u8; PAYLOAD_LEN] {
        let level = |mode: LampMode| self.brightness_step[mode as usize].clamp(0, BRIGHTNESS_STEPS) as u8;
        [
            SETTINGS_VERSION,
            self.mode as u8 | level(LampMode::Spread) << 2,
            self.mix_step.clamp(0, MIX_STEPS) as u8,
            level(LampMode::White),
            level(LampMode::Uv),
            level(LampMode::Ir),
        ]
    }

    fn decode(payload: &[u8; PAYLOAD_LEN]) -> Option<Self> {
        let (mode, spread) = match payload[0] {
            1 => (payload[1], INITIAL_BRIGHTNESS_STEP),
            2 => (payload[1] & 0b11, (payload[1] >> 2) as i32),
            _ => return None,
        };
        let mut brightness_step = [0; LampMode::COUNT];
        brightness_step[LampMode::White as usize] = payload[3] as i32;
        brightness_step[LampMode::Uv as usize] = payload[4] as i32;
        brightness_step[LampMode::Ir as usize] = payload[5] as i32;
        brightness_step[LampMode::Spread as usize] = spread;
        Some(Self {
            mode: LampMode::from_u8(mode)?,
            mix_step: (payload[2] as i32).min(MIX_STEPS),
            brightness_step: brightness_step.map(|step| step.min(BRIGHTNESS_STEPS)),
        })
    }
}

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

fn led_group_frame(led_addresses: &[usize]) -> [u8; LED_COUNT] {
    let mut frame = [0u8; LED_COUNT];
    for &address in led_addresses {
        frame[address] = 0xFF;
    }
    frame
}

/// Spread mode: number of cool and warm LEDs to light. The total follows the brightness
/// (0..=30 LEDs), the mix splits it like the white mode; once one colour runs out of LEDs,
/// the rest goes to the other one.
fn spread_counts(brightness_step: i32, mix_step: i32) -> (usize, usize) {
    const PER_GROUP: i32 = CW_SPREAD_ORDER.len() as i32;
    let total = (brightness_step.clamp(0, BRIGHTNESS_STEPS) * 2 * PER_GROUP + BRIGHTNESS_STEPS / 2) / BRIGHTNESS_STEPS;
    let mut ww = (total * mix_step.clamp(0, MIX_STEPS) + MIX_STEPS / 2) / MIX_STEPS;
    let mut cw = total - ww;
    if cw > PER_GROUP {
        ww += cw - PER_GROUP;
        cw = PER_GROUP;
    }
    if ww > PER_GROUP {
        cw += ww - PER_GROUP;
        ww = PER_GROUP;
    }
    (cw as usize, ww as usize)
}

/// What the shift registers currently hold.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Frame {
    /// One of the fixed colour groups (index into the group frames).
    Group(usize),
    /// The first `cw` / `ww` LEDs of the spread orders.
    Spread { cw: usize, ww: usize },
}

impl Frame {
    fn data(self, groups: &[[u8; LED_COUNT]]) -> [u8; LED_COUNT] {
        match self {
            Frame::Group(idx) => groups[idx],
            Frame::Spread { cw, ww } => {
                let mut frame = [0u8; LED_COUNT];
                for &address in CW_SPREAD_ORDER[..cw].iter().chain(&WW_SPREAD_ORDER[..ww]) {
                    frame[address] = 0xFF;
                }
                frame
            }
        }
    }
}

/// Shift `frame` into the registers and latch it, unless it is already loaded.
fn load(sr_driver: &mut LampDriver, loaded_frame: &mut Option<Frame>, groups: &[[u8; LED_COUNT]], frame: Frame) {
    if *loaded_frame != Some(frame) {
        sr_driver
            .write(0, &frame.data(groups))
            .expect("48 LED frame should fit driver buffer");
        sr_driver.load_frame();
        *loaded_frame = Some(frame);
    }
}

/// Wait with sub-tick precision for short on-phases, yield to the executor for long ones.
async fn wait_us(us: u64) {
    if us == 0 {
        return;
    }
    if us < BUSY_WAIT_THRESHOLD_US {
        CycleDelay.delay_us(us as u32);
    } else {
        Timer::after_micros(us).await;
    }
}

type LampDriver = SrLedDriver<'static, peripherals::SPI0, LED_COUNT>;


/// Show one LED group for `on_us` microseconds, then blank the outputs again.
async fn on_phase(
    sr_driver: &mut LampDriver,
    loaded_frame: &mut Option<Frame>,
    frames: &[[u8; LED_COUNT]],
    frame_idx: usize,
    on_us: u64,
) {
    if on_us == 0 {
        return;
    }
    load(sr_driver, loaded_frame, frames, Frame::Group(frame_idx));
    sr_driver.enable_output();
    wait_us(on_us).await;
    sr_driver.disable_output();
}

#[embassy_executor::task]
async fn lamp_task(
    mut sr_driver: LampDriver,
    pin_a: peripherals::PA5,
    pin_b: peripherals::PA4,
    button_pin: peripherals::PB22,
) -> ! {
    log!("Cyberlamp LED task started");

    const FRAME_CW: usize = 0;
    const FRAME_WW: usize = 1;
    const FRAME_UV: usize = 2;
    const FRAME_IR: usize = 3;
    let frames = [
        led_group_frame(CW_LED_ADDRESSES),
        led_group_frame(WW_LED_ADDRESSES),
        led_group_frame(UV_LED_ADDRESSES),
        led_group_frame(IR_LED_ADDRESSES),
    ];
    let mut loaded_frame: Option<Frame> = None;
    sr_driver.disable_output();

    let (mut store, stored) = SettingsStore::load(SETTINGS_FLASH_ADDR);
    let mut settings = match stored.as_ref().and_then(LampSettings::decode) {
        Some(settings) => {
            log!(
                "Restored settings: mode={:?} mix={} brightness={:?}",
                settings.mode,
                settings.mix_step,
                settings.brightness_step
            );
            settings
        }
        None => {
            log!("No stored settings, using defaults");
            LampSettings::DEFAULT
        }
    };
    let mut saved_payload = settings.encode();
    let mut last_change = Instant::now();

    let enc_a = Input::new(pin_a, Pull::Up);
    let enc_b = Input::new(pin_b, Pull::Up);
    let button = Input::new(button_pin, Pull::Up);
    let mut encoder_state = read_encoder_state(&enc_a, &enc_b);
    let mut button_pressed = button.is_low();
    let mut rotated_while_pressed = button_pressed; // never switch mode on a press held since boot
    let mut button_debounce_ticks = 0u8;

    loop {
        // --- Encoder ---
        let current_encoder_state = read_encoder_state(&enc_a, &enc_b);
        if current_encoder_state != encoder_state {
            let delta = decode_encoder_delta(encoder_state, current_encoder_state);
            encoder_state = current_encoder_state;
            if delta != 0 {
                if button_pressed {
                    rotated_while_pressed = true;
                }
                if settings.mode.is_white() && !button_pressed {
                    settings.mix_step = (settings.mix_step + delta).clamp(0, MIX_STEPS);
                } else {
                    let brightness = &mut settings.brightness_step[settings.mode as usize];
                    *brightness =
                        (*brightness + delta).clamp(-ENCODER_DEADZONE_STEPS, BRIGHTNESS_STEPS + ENCODER_DEADZONE_STEPS);
                }
                last_change = Instant::now();
            }
        }

        // --- Button: a click (press + release without turning the knob) cycles the mode ---
        let button_is_pressed = button.is_low();
        if button_debounce_ticks > 0 {
            button_debounce_ticks -= 1;
        } else if button_is_pressed != button_pressed {
            button_pressed = button_is_pressed;
            button_debounce_ticks = BUTTON_DEBOUNCE_PWM_TICKS;
            if button_pressed {
                rotated_while_pressed = false;
            } else if !rotated_while_pressed {
                settings.mode = settings.mode.next();
                last_change = Instant::now();
                log!("Mode: {:?}", settings.mode);
            }
        }

        // Apply remote commands here, with the same state and persistence as the knob.
        if let Ok(request) = LAMP_COMMANDS.try_receive() {
            let mut reply = Reply::new();
            if !request.session.is_current() {
                reply.write_str("ERR disconnected\n").unwrap();
            } else {
                let previous = settings;
                match request.command {
                    Command::Brightness(level) => {
                        settings.brightness_step[settings.mode as usize] = level as i32;
                        writeln!(reply, "OK brightness={}", level).unwrap();
                    }
                    Command::Mix(level) => {
                        settings.mix_step = level as i32;
                        writeln!(reply, "OK mix={}", level).unwrap();
                    }
                    Command::Mode(mode) => {
                        settings.mode = LampMode::from_u8(mode).unwrap();
                        writeln!(reply, "OK mode={}", settings.mode.name()).unwrap();
                    }
                    Command::Status => {
                        let level = |mode: LampMode| settings.brightness_step[mode as usize].clamp(0, BRIGHTNESS_STEPS);
                        writeln!(reply, "OK mode={} mix={} brightness={} white={} spread={} uv={} ir={}",
                            settings.mode.name(), settings.mix_step, level(settings.mode),
                            level(LampMode::White), level(LampMode::Spread),
                            level(LampMode::Uv), level(LampMode::Ir)).unwrap();
                    }
                    Command::Help => {
                        reply.write_str("OK br 0..40; mx 0..40; md wh|sp|uv|ir; st; he\n").unwrap();
                    }
                }
                if settings != previous { last_change = Instant::now(); }
            }
            // The console permits exactly one outstanding request.
            assert!(LAMP_REPLIES.try_send(reply).is_ok());
        }

        // --- Persist once the settings have settled ---
        let payload = settings.encode();
        if payload != saved_payload && last_change.elapsed() >= SAVE_DELAY {
            sr_driver.disable_output();
            if store.save(&payload) {
                log!("Settings saved");
            } else {
                log!("Settings save FAILED");
            }
            saved_payload = payload;
        }

        // --- Spread: /OE stays on, the frame sets the brightness ---
        if settings.mode == LampMode::Spread {
            let (cw, ww) = spread_counts(settings.brightness_step[LampMode::Spread as usize], settings.mix_step);
            load(&mut sr_driver, &mut loaded_frame, &frames, Frame::Spread { cw, ww });
            if cw + ww > 0 {
                sr_driver.enable_output();
            } else {
                sr_driver.disable_output();
            }
            Timer::after_micros(OE_PWM_PERIOD_US).await;
            continue;
        }

        // --- One software PWM period on /OE ---
        // Spread mode leaves /OE on; the PWM modes start every period with it off.
        sr_driver.disable_output();
        let on_us = brightness_duty_us(settings.brightness_step[settings.mode as usize]);
        match settings.mode {
            LampMode::Spread => unreachable!(),
            LampMode::White => {
                // Time-multiplex the two white groups; the total on-time stays constant, so the
                // overall brightness doesn't change with the colour temperature.
                let ww_us = on_us * settings.mix_step as u64 / MIX_STEPS as u64;
                let cw_us = on_us - ww_us;
                on_phase(&mut sr_driver, &mut loaded_frame, &frames, FRAME_CW, cw_us).await;
                on_phase(&mut sr_driver, &mut loaded_frame, &frames, FRAME_WW, ww_us).await;
            }
            LampMode::Uv => on_phase(&mut sr_driver, &mut loaded_frame, &frames, FRAME_UV, on_us).await,
            LampMode::Ir => on_phase(&mut sr_driver, &mut loaded_frame, &frames, FRAME_IR, on_us).await,
        }
        wait_us(OE_PWM_PERIOD_US - on_us).await;
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    // Cyberlamp is mains-powered. Poll continuously: qingke-rt's WFE-based idle can
    // stall both the lamp and BLE, and plain WFI does not reliably wake with IRQs
    // masked by Embassy's standard executor. Keep this board awake until that path
    // is fixed; tasks and timers still use Embassy's normal cooperative scheduler.
    let executor = embassy_executor::raw::Executor::new(core::ptr::null_mut());
    // SAFETY: this entry function never returns, and the executor is never moved
    // after taking its address. Only this thread polls it.
    let executor: &'static embassy_executor::raw::Executor = unsafe { &*(&executor as *const _) };
    let spawner = executor.spawner();
    spawner.spawn(app_main(spawner)).unwrap();
    loop {
        unsafe { executor.poll(); }
    }
}

#[qingke_rt::interrupt]
fn Exception() {
    ch58x_rs_kit::fatal::halt(core::format_args!("CPU exception"))
}

#[embassy_executor::task]
async fn app_main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);
    ch58x_rs_kit::fatal::set_safe_state_hook(leds_off);
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
    log!("Starting lamp task...");

    spawner
        .spawn(lamp_task(sr_driver, p.PA5, p.PA4, p.PB22))
        .unwrap();

    let rtc = get_configured_rtc();

    log!("System Clocks: {}", hal::sysctl::clocks().hclk);
    log!("ChipID: 0x{:02x}", hal::signature::get_chip_id());
    log!("RTC datetime: {}", rtc.now());
    let lsi_hz = hal::ble::calibrate_lsi()
        .expect("BLE needs a calibrated 32000 Hz internal clock");
    log!("BLE LSI calibrated: {} Hz", lsi_hz);

    log!("BLE Lib Version: {}", ble::lib_version());

    let mut ble_config = ble::Config::default();
    ble_config.pa_config = None;
    ble_config.mac_addr = [0x21, 0x37, 0x04, 0x20, 0x69, 0x96].into();
    let (task_id, sub) = hal::ble::init(ble_config).unwrap();
    log!("BLE hal task id: {}", task_id);

    unsafe {
        common_init();
        use hal::ble::ffi::*;
        use hal::ble::gap::GAPRole_SetParameter;
        // Automatic 2M PHY negotiation establishes a link but stalls ATT on this board.
        // Keep 1M until that path has been investigated separately.
        let phy = GAP_PHY_BIT_LE_1M as u8;
        GAPRole_SetParameter(GAPROLE_PHY_TX_SUPPORTED, 1, &phy as *const _ as _).unwrap();
        GAPRole_SetParameter(GAPROLE_PHY_RX_SUPPORTED, 1, &phy as *const _ as _).unwrap();
        devinfo_init();
        current_time_init();
        nus::init();
    }

    spawner.spawn(tmos_mainloop()).unwrap();
    spawner.spawn(nus_console()).unwrap();

    peripheral_keep_running(spawner, task_id, sub, rtc).await
}

/// Fatal-error safe state: PA8 is the active-low LED output enable.
fn leds_off() {
    let oe = hal::gpio::Output::new(
        unsafe { hal::peripherals::PA8::steal() },
        hal::gpio::Level::High,
        hal::gpio::OutputDrive::_5mA,
    );
    core::mem::forget(oe);
}
