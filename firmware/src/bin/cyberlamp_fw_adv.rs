//! Cyberlamp firmware controlled by BLE broadcasts *and* the BLE console (Nordic UART Service).
//!
//! The lamp is a connectable peripheral (NUS, like `cyberlamp_fw`) and scans for broadcasts at
//! the same time, using the central role for scanning only (WCH's tested peripheral + central
//! combination, see the SDK's `CentPeri` example); it never connects to anything itself.
//! Packet format: see `cyberlamp_rs::adv`. Knob and button work as in `cyberlamp_fw`.

#![feature(impl_trait_in_assoc_type)]
#![feature(generic_const_exprs)]
#![allow(incomplete_features)]
#![no_std]
#![no_main]

use core::cell::Cell;
use core::ptr::addr_of_mut;

use ch58x_hal as hal;
use ch58x_rs_kit::ble_periph::current_time_service::current_time_init;
use ch58x_rs_kit::ble_periph::{common_init_named, devinfo_init, nus, peripheral_keep_running};
use ch58x_rs_kit::logger::init as init_logger;
use ch58x_rs_kit::sr_led_driver::SrLedDriver;
use ch58x_rs_kit::{get_configured_rtc, log, tmos_mainloop};
use critical_section::Mutex;
use cyberlamp_rs::adv::{Broadcast, GROUP_ALL};
use cyberlamp_rs::lamp::{lamp_task, leds_off, nus_console, Request, LED_COUNT, REQUESTS};
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
/// Lamp packets heard in the current discovery period (repeats included), for the log.
static HEARD: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

extern "C" {
    // Missing from ch58x-hal's bindings. CH58xBLE_LIB.h:
    // bStatus_t GAPRole_CentralStartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList);
    fn GAPRole_CentralStartDiscovery(mode: u8, active_scan: u8, white_list: u8) -> bStatus_t;
}

fn start_discovery() {
    unsafe { GAPRole_CentralStartDiscovery(DISCOVERY_MODE, PASSIVE_SCAN, NO_WHITE_LIST).unwrap() };
}

fn on_advertisement(data: &[u8]) {
    let Some(broadcast) = Broadcast::parse(data) else { return };
    if broadcast.group != GROUP_ALL {
        return;
    }
    critical_section::with(|cs| {
        let heard = HEARD.borrow(cs);
        heard.set(heard.get() + 1);
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

/// Central-role events (scanning only). Runs in the TMOS task (thread mode).
unsafe extern "C" fn central_event(event: *mut gapRoleEvent_t) {
    let event = &*event;
    match event.gap.opcode {
        GAP_DEVICE_INIT_DONE_EVENT => {
            log!("Scanning for lamp broadcasts");
            start_discovery();
        }
        GAP_DEVICE_DISCOVERY_EVENT => {
            let heard = critical_section::with(|cs| HEARD.borrow(cs).replace(0));
            if heard > 0 {
                log!("Heard {} lamp packets in the last scan period", heard);
            }
            start_discovery();
        }
        GAP_DEVICE_INFO_EVENT => {
            let info = event.deviceInfo;
            if !info.pEvtData.is_null() {
                on_advertisement(core::slice::from_raw_parts(info.pEvtData, info.dataLen as usize));
            }
        }
        _ => {}
    }
}

static mut CENTRAL_CALLBACKS: gapCentralRoleCB_t = gapCentralRoleCB_t {
    rssiCB: None,
    eventCB: Some(central_event),
    ChangCB: None,
};
/// The central role never connects, so it never pairs.
static mut CENTRAL_BOND_CALLBACKS: gapBondCBs_t = gapBondCBs_t {
    passcodeCB: None,
    pairStateCB: None,
    oobCB: None,
};

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
    log!("\n\n\nCyberlamp broadcast + console firmware started");

    // Cyberlamp-light SM16106SC chain: SCLK - PA13, DAT - PA14, /OE - PA8, LAT - PA9.
    let sr_driver =
        SrLedDriver::<_, LED_COUNT>::new_from_pins::<false, _, _, _, _>(p.SPI0, p.PA13, p.PA14, p.PA8, p.PA9);
    spawner.spawn(lamp_task(sr_driver, p.PA5, p.PA4, p.PB22)).unwrap();

    let rtc = get_configured_rtc();
    let lsi_hz = hal::ble::calibrate_lsi().expect("BLE needs a calibrated 32000 Hz internal clock");
    log!("BLE LSI calibrated: {} Hz", lsi_hz);
    log!("BLE Lib Version: {}", ble::lib_version());

    let mut ble_config = ble::Config::default();
    ble_config.pa_config = None;
    // Many lamps run this firmware: each one uses its chip's own address.
    ble_config.use_factory_mac_address();
    let (task_id, events) = ble::init(ble_config).unwrap();
    log!("BLE hal task id: {}, address {}", task_id, hal::ble::MacAddress::from_raw(hal::isp::get_mac_address()));

    unsafe {
        common_init_named("cyberlamp"); // peripheral role
        GAPRole_CentralInit().unwrap();
        GAP_SetParamValue(TGAP_DISC_SCAN, SCAN_PERIOD).unwrap();
        GAP_SetParamValue(TGAP_DISC_SCAN_PHY, GAP_PHY_BIT_LE_1M).unwrap();
        // Report every advertisement, not just the first one per sender and scan period:
        // a sender's packet changes when its state changes.
        GAP_SetParamValue(TGAP_FILTER_ADV_REPORTS, 0).unwrap();

        // Automatic 2M PHY negotiation establishes a link but stalls ATT on this board.
        // Keep 1M until that path has been investigated separately.
        let phy = GAP_PHY_BIT_LE_1M as u8;
        GAPRole_SetParameter(GAPROLE_PHY_TX_SUPPORTED, 1, &phy as *const _ as _).unwrap();
        GAPRole_SetParameter(GAPROLE_PHY_RX_SUPPORTED, 1, &phy as *const _ as _).unwrap();
        devinfo_init();
        current_time_init();
        nus::init();

        GAPRole_CentralStartDevice(task_id, addr_of_mut!(CENTRAL_BOND_CALLBACKS), addr_of_mut!(CENTRAL_CALLBACKS))
            .unwrap();
    }

    spawner.spawn(tmos_mainloop()).unwrap();
    spawner.spawn(nus_console()).unwrap();

    peripheral_keep_running(spawner, task_id, events, rtc).await
}
