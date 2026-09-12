# C3CanSniffer

ESP32-C3 Super Mini + SN65HVD230 in-car CAN sniffer.

Board pinout: [ESP32-C3 Super Mini](https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/#pinout)

- **Bus:** C3 TWAI (CAN 2.0), GPIO4 TX / GPIO5 RX
- **Field console:** BLE Nordic UART, name `C3-CAN`
- **Laptop:** Bleak (`host/sniffer.py`) — not ble-serial
- **Phone:** Serial Bluetooth Terminal, BLE profile **Nordic UART**
- **Bench flash:** USB-C with 12 V unplugged

BLE cannot dump an unfiltered 500 kbit/s car bus. Firmware defaults to **listen-only** and **classify** (name bytes on the C3, do not log frames). Type an ID to inspect that message.

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

One letter from the phone toggles that sensor’s stream. Two letters are system commands so they never steal a sensor key.

| cmd | effect |
|---|---|
| `w` | toggle wheel-speed diffs |
| `s` | toggle steering diffs |
| `y` | toggle yaw diffs |
| `g` | toggle GPS diffs |
| `v` | toggle vehicle-speed diffs |
| `x` | toggle forward-accel diffs |
| `h` | toggle heading diffs |
| `o` | toggle odometer diffs |
| `l` | toggle lateral-accel diffs |
| `r` | toggle roll diffs |
| `z` | toggle vertical / G diffs |
| `n` | print named sensors |
| `c` | classify now |
| `d` | dump all changed frames (BLE will choke on a busy bus) |
| `si` | silent / listen-only |
| `rs` | reset counters and names (fresh identify) |
| `st` | TEC/REC, queue, drops, watches |
| `p` | ping (BOOT button too) |
| `?` | help |
| `all` | classify every ID, no frame dump |
| `0x2E1` / `737` | lock one ID and dump it |
| `50` `125` `250` `500` `800` | kbit/s |

### Classify — name IDs on the C3, stream only what you toggle

Boot starts from scratch. The C3 watches the whole bus, does not log frames, and names fields as the car behaves:

1. Byte roles: `STATIC` `COUNTER` `CSUM` `MUX` `ANALOG`
2. 16-bit analog pairs (the smoother endian)
3. Vehicle logic: wheels are a cluster of similar 16-bit values; steer is the 16-bit that moves while wheels are quiet; yaw follows steer once moving; GPS is the slow walker that goes both ways; odo only increases

| name | meaning | how it gets named |
|---|---|---|
| `W` | wheels | 3–4 similar 16-bit fields, ~10 ms |
| `S` | steering | moves while wheels are still |
| `Y` | yaw | sign tracks steer when rolling |
| `G` | GPS | slow 16/32-bit, not monotonic |
| `V` | vehicle speed | matches wheel mean |
| `X` | forward accel | sign tracks wheel accel |
| `L` | lateral accel | sign tracks steer while moving |
| `H` | heading | medium-rate angle, not GPS/odo |
| `O` | odometer | 16-bit, only increases |
| `R` | roll | leftover IMU-rate signed field |
| `Z` | vertical / G | stable band, may not exist |

A letter arms a watch even before the name exists. When that field is named and its value changes:

```
[Y] 2E1  b2=12  00 00 00 0C 00 00 00 C3
```

Type `n` for the table. `rs` forgets names and starts over.

Line format when dumping:

```
123456  2E1       8 00 00 1A F0 00 00 00 00
```
