# cyberlamp firmware

Rust firmware for the cyberlamp-light board (CH582F).

- Modes: white / UV / IR, a button click (without turning the knob) selects the next one
- White: knob = colour temperature, button + knob = brightness; UV / IR: knob = brightness
- LEDs: 48 channels on SM16106SC shift registers (SPI0: SCLK=PA13, DAT=PA14; /OE=PA8 software PWM, LAT=PA9)
- Settings are saved to DataFlash two seconds after the last change
- BLE: Nordic UART Service console, see below

`cyberlamp_fw` is the full firmware, `cyberlamp_fw_simple` a minimal LED test without BLE.

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
| `brightness 20` | Set the current mode's brightness step, `0..40` (0 is off) |
| `mix 20` | Set white colour mix, `0..40` (0 is cool, 40 is warm) |
| `mode white` | Select `white`, `uv`, or `ir`; each remembers its own brightness |
| `status` | Read mode, mix, current brightness, and all three per-mode brightness settings |
| `help` | List commands |

For example, writing `brightness 20\n` produces `OK brightness=20\n` on TX.
Settings use the same two-second delayed DataFlash save as physical knob/button
changes. `mix` configures the white mix even while another mode is selected.

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
