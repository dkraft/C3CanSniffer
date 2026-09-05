# C3CanSniffer

ESP32-C3 Super Mini + SN65HVD230 in-car CAN sniffer.

Board pinout: [ESP32-C3 Super Mini](https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/#pinout)

- **Bus:** C3 TWAI (CAN 2.0), GPIO4 TX / GPIO5 RX
- **Field console:** BLE Nordic UART, name `C3-CAN`
- **Laptop:** Bleak (`host/sniffer.py`) — not ble-serial
- **Phone:** Serial Bluetooth Terminal, BLE profile **Nordic UART**
- **Bench flash:** USB-C with 12 V unplugged

BLE cannot dump an unfiltered 500 kbit/s car bus. Firmware defaults to **listen-only** and **diff** (print a frame only when that ID’s payload changes). Filter to one ID for a live stream of a single message.

## Power

Do **not** plug USB-C and the vehicle 5 V feed at the same time.

```
12V battery
    │
    ▼
  5V buck  ──►  Super Mini 5V pin
                    │
                   3V3  ──►  SN65HVD230 VCC
                   GND  ──►  SN65HVD230 GND
```

SN65HVD230 is 3.3 V. Never put 5 V on its VCC or on a C3 GPIO.

## Wiring

| SN65HVD230 | Super Mini |
|---|---|
| VCC | **3V3** |
| GND | GND |
| TXD / D | **GPIO4** (TWAI TX) |
| RXD / R | **GPIO5** (TWAI RX) |
| RS | GND (high-speed) or 10–100 kΩ to GND |
| CANH / CANL | vehicle bus; 120 Ω at each end |

Cheap “SN65HVD230” boards often swap TXD/RXD silkscreen. If TX looks live and RX is dead, swap those two wires.

## Firmware (Arduino)

Board: **ESP32C3 Dev Module** (or Super Mini equivalent), USB CDC on boot enabled.

Library: **NimBLE-Arduino** (this machine already has 2.1.2 under `Arduino/libraries`).

Sketch: `can_sniffer_c3/can_sniffer_c3.ino`

Flash on USB, then unplug USB before applying 12 V → 5 V.

## Laptop (Bleak)

```bash
python3 -m pip install -r host/requirements.txt
python3 host/sniffer.py
python3 host/sniffer.py --log /tmp/can.log
```

Only one BLE central at a time: quit the phone app before the laptop, and vice versa.

## Commands

Same on USB Serial, Bleak stdin, or the phone terminal:

| cmd | effect |
|---|---|
| `all` | accept every ID |
| `0x2E1` / `737` | hardware + software filter to one ID |
| `50` `125` `250` `500` `800` | kbit/s, re-inits TWAI |
| `s` | toggle silent / listen-only |
| `d` | toggle diff mode |
| `r` | reset counters and diff history |
| `st` | TEC/REC, queue, drops |
| `p` | ping (BOOT button on GPIO9 does the same) |
| `h` | help |

Line format:

```
123456  2E1       8 00 00 1A F0 00 00 00 00
```
