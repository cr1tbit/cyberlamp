# cyberlamp firmware

Rust firmware for the cyberlamp-light board (CH582F).

- Modes: white → spread → UV → IR, a button click (without turning the knob) selects the next one
- White and spread: knob = colour temperature, button + knob = brightness; UV / IR: knob = brightness
- Spread mode keeps /OE on and sets the brightness by the number of lit LEDs (0..30), in an
  order spread over the whole lamp. It has no software PWM, so BLE activity can't make it flicker.
- LEDs: 48 channels on SM16106SC shift registers (SPI0: SCLK=PA13, DAT=PA14; /OE=PA8 software PWM, LAT=PA9)
- Settings are saved to DataFlash two seconds after the last change
- BLE: Nordic UART Service console, see below. [`utils/lamp_control.html`](utils/lamp_control.html)
  is a Web Bluetooth page with buttons and sliders for it, published at
  <https://cr1tbit.github.io/cyberlamp/> (Chrome/Edge). To try local changes, serve it from
  `http://localhost`, e.g. `python3 -m http.server` in `utils/`

Firmware variants:

- `cyberlamp_fw`: full firmware with the BLE console (connectable, Nordic UART Service)
- `cyberlamp_fw_adv`: controlled by BLE broadcasts, for many lamps at once. The lamp only
  listens (not connectable, never transmits); every lamp in range follows the same sender.
  Send a state from a Linux PC with `python3 utils/lamp_broadcast.py --mode sp --brightness 20 --mix 10`
  (BlueZ `bluetoothctl`, no root). Packet format: [`src/adv.rs`](src/adv.rs).
  A lamp applies each broadcast once, so the knob still works between broadcasts.
- `cyberlamp_fw_simple`: minimal LED test without BLE

## Building and flashing

Requirements: [rustup](https://rustup.rs) (the toolchain is pinned in `rust-toolchain.toml`),
[wlink](https://github.com/ch32-rs/wlink) and a WCH-LinkE probe (`cargo install wlink`).

```sh
cargo run --release --bin cyberlamp_fw     # build, flash with wlink, reset
```

Logs are printed on UART0 TX (PB7) at 115200 baud, e.g. via the WCH-LinkE's serial port.

Shared code comes from [ch58x-rs-kit](https://github.com/cr1tbit/ch58x-rs-playground) and
[ch58x-hal](https://github.com/cr1tbit/ch58x-hal) (git dependencies, pinned in `Cargo.lock`).
To work on them locally, add a temporary override to `Cargo.toml`:

```toml
[patch."https://github.com/cr1tbit/ch58x-rs-playground.git"]
ch58x-rs-kit = { path = "../../ch58x-rs-playground/ch58x-rs-kit" }
```

## Bluetooth console

`cyberlamp_fw` exposes the [Nordic UART Service](https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrf/libraries/bluetooth/services/nus.html)
for lamp commands. Connect to `21:37:04:20:69:96` with a BLE UART terminal or GATT
client, enable notifications on TX, and write text to RX.

| Attribute | UUID | Operation |
| --- | --- | --- |
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | Discover |
| RX | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` | Write / Write Without Response |
| TX | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` | Notify |

End each command with a newline; CRLF also works. Send one command and wait for
its newline-terminated reply before sending the next.

| Command | Effect |
| --- | --- |
| `br 20` | Set the current mode's brightness step, `0..40` (0 is off) |
| `mx 20` | Set white colour mix, `0..40` (0 is cool, 40 is warm), used by white and spread |
| `md sp` | Select `wh` (white), `sp` (spread), `uv` or `ir`; each remembers its own brightness |
| `st` | Read mode, mix, current brightness, and the brightness of every mode |
| `he` | List commands |

The long names from earlier versions (`brightness`, `mix`, `mode white`, `status`, `help`)
still work. For example, writing `br 20\n` produces `OK brightness=20\n` on TX.
Settings use the same two-second delayed DataFlash save as physical knob/button
changes. `mx` configures the white mix even while another mode is selected.

The current ATT MTU is 23: split writes into at most 20 bytes and join TX
notifications until a newline arrives. Commands may span writes; a line may
contain up to 64 bytes before its terminator. Invalid commands return `ERR ...`
and leave settings unchanged. Disconnect clears partial lines and notification
subscriptions; subscribe again after reconnecting. If RX is overrun (ATT error
`0x11`), send a blank newline to resynchronize, then retry the command.

The pure parser checks can run on the host:

```sh
rustc --edition 2021 --test --target x86_64-unknown-linux-gnu \
  src/protocol.rs -o /tmp/cyberlamp-protocol-tests
/tmp/cyberlamp-protocol-tests
```

Use UART and external BLE clients for debugging: debugger memory reads can hang
the running board.

Panic and CPU exception handling disable LED output and report the
fault over UART, then halt. They do not blink the LED output-enable pin.
