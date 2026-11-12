//! Cyberlamp firmware with the BLE console (Nordic UART Service).

#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![no_std]
#![no_main]

use ch58x_hal as hal;
use embassy_executor::Spawner;
use hal::uart::UartTx;
use hal::ble;
use ch58x_rs_kit::ble_periph::current_time_service::current_time_init;
use ch58x_rs_kit::ble_periph::{common_init_named, devinfo_init, nus, peripheral_keep_running};
use ch58x_rs_kit::logger::init as init_logger;
use ch58x_rs_kit::sr_led_driver::SrLedDriver;
use ch58x_rs_kit::{get_configured_rtc, log, tmos_mainloop};
use cyberlamp_rs::lamp::{lamp_task, leds_off, nus_console, LED_COUNT};


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
        common_init_named("cyberlamp");
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
