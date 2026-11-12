//! Cyberlamp firmware controlled by BLE broadcasts: the lamp only scans (observer role), it is
//! not connectable and never transmits. Any number of lamps follow the same sender.
//! Packet format: see `cyberlamp_rs::adv`. Knob and button work as in `cyberlamp_fw`.

#![feature(impl_trait_in_assoc_type)]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]
#![no_std]
#![no_main]

use core::cell::Cell;

use ch58x_hal as hal;
use ch58x_rs_kit::logger::init as init_logger;
use ch58x_rs_kit::sr_led_driver::SrLedDriver;
use ch58x_rs_kit::{get_configured_rtc, log, tmos_mainloop};
use critical_section::Mutex;
use cyberlamp_rs::adv::{Broadcast, GROUP_ALL};
use cyberlamp_rs::lamp::{lamp_task, leds_off, Request, LED_COUNT, REQUESTS};
use embassy_executor::Spawner;
use hal::ble;
use hal::ble::ffi::*;
use hal::ble::gap::*;
use hal::uart::UartTx;

/// 4800 * 0.625 ms = 3 s per discovery period, restarted right away.
const SCAN_PERIOD: u16 = 4800;
const DISCOVERY_MODE: u8 = DEVDISC_MODE_ALL;
const PASSIVE_SCAN: u8 = 0; // receive only, never send scan requests
const NO_WHITE_LIST: u8 = 0;

/// Sequence number of the last broadcast handed to the lamp.
static LAST_SEQ: Mutex<Cell<Option<u8>>> = Mutex::new(Cell::new(None));

fn start_discovery() {
    unsafe { GAPRole_ObserverStartDiscovery(DISCOVERY_MODE, PASSIVE_SCAN, NO_WHITE_LIST).unwrap() };
}

/// Runs in the TMOS task (thread mode), see `tmos_mainloop`.
unsafe extern "C" fn observer_event(event: &gapRoleEvent_t) {
    match event.gap.opcode {
        GAP_DEVICE_INIT_DONE_EVENT => {
            log!("Scanning for lamp broadcasts");
            start_discovery();
        }
        GAP_DEVICE_DISCOVERY_EVENT => start_discovery(),
        GAP_DEVICE_INFO_EVENT => {
            let info = event.deviceInfo;
            if info.pEvtData.is_null() {
                return;
            }
            let data = core::slice::from_raw_parts(info.pEvtData, info.dataLen as usize);
            let Some(broadcast) = Broadcast::parse(data) else { return };
            if broadcast.group != GROUP_ALL {
                return;
            }
            critical_section::with(|cs| {
                let last_seq = LAST_SEQ.borrow(cs);
                if last_seq.get() == Some(broadcast.seq) {
                    return; // the sender repeats each packet, apply it once
                }
                let request = Request::Broadcast {
                    mode: broadcast.mode,
                    brightness: broadcast.brightness,
                    mix: broadcast.mix,
                };
                // If the lamp hasn't taken the previous request yet, retry with the next packet.
                if REQUESTS.try_send(request).is_ok() {
                    last_seq.set(Some(broadcast.seq));
                }
            });
        }
        _ => {}
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    // Poll continuously, like cyberlamp_fw: the lamp is mains-powered.
    let executor = embassy_executor::raw::Executor::new(core::ptr::null_mut());
    // SAFETY: this entry function never returns, and the executor is never moved
    // after taking its address. Only this thread polls it.
    let executor: &'static embassy_executor::raw::Executor = unsafe { &*(&executor as *const _) };
    let spawner = executor.spawner();
    spawner.spawn(app_main(spawner)).unwrap();
    loop {
        unsafe { executor.poll() };
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
    log!("\n\n\nCyberlamp broadcast-controlled firmware started");

    // Cyberlamp-light SM16106SC chain: SCLK - PA13, DAT - PA14, /OE - PA8, LAT - PA9.
    let sr_driver =
        SrLedDriver::<_, LED_COUNT>::new_from_pins::<false, _, _, _, _>(p.SPI0, p.PA13, p.PA14, p.PA8, p.PA9);
    spawner.spawn(lamp_task(sr_driver, p.PA5, p.PA4, p.PB22)).unwrap();

    // The RTC is the 32 kHz timebase of the BLE stack.
    let _rtc = get_configured_rtc();
    let lsi_hz = hal::ble::calibrate_lsi().expect("BLE needs a calibrated 32000 Hz internal clock");
    log!("BLE LSI calibrated: {} Hz", lsi_hz);
    log!("BLE Lib Version: {}", ble::lib_version());

    let mut ble_config = ble::Config::default();
    ble_config.pa_config = None;
    let (task_id, _events) = ble::init(ble_config).unwrap();
    log!("BLE hal task id: {}", task_id);

    static OBSERVER_CALLBACKS: gapRoleObserverCB_t = gapRoleObserverCB_t { eventCB: Some(observer_event) };
    unsafe {
        GAPRole_ObserverInit().unwrap();
        GAP_SetParamValue(TGAP_DISC_SCAN, SCAN_PERIOD).unwrap();
        GAP_SetParamValue(TGAP_DISC_SCAN_PHY, GAP_PHY_BIT_LE_1M).unwrap();
        // Report every advertisement, not just the first one per sender and scan period:
        // a sender's packet changes when its state changes.
        GAP_SetParamValue(TGAP_FILTER_ADV_REPORTS, 0).unwrap();
        GAPRole_ObserverStartDevice(&OBSERVER_CALLBACKS).unwrap();
    }

    spawner.spawn(tmos_mainloop()).unwrap();

    loop {
        embassy_time::Timer::after_secs(3600).await;
    }
}
