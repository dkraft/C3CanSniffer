/*************************************************************************
 * can_sniffer_c3  - in-car CAN sniffer
 *
 * Hardware: ESP32-C3 Super Mini + SN65HVD230
 *   Pinout: https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/#pinout
 *   C3 TWAI (CAN 2.0) on GPIO4 (TX) / GPIO5 (RX)
 *   BLE Nordic UART (NUS) for field debug; USB CDC for bench flash
 *
 * Power (in car):
 *   12V -> 5V buck -> Super Mini 5V pin     (USB-C unplugged)
 *   Super Mini 3V3 -> SN65HVD230 VCC        (3.3V transceiver, not 5V)
 *   GND common
 *
 * Wiring (by signal name; cheap-module silkscreen is often swapped):
 *   GPIO4  -> 230 TXD / D
 *   GPIO5  -> 230 RXD / R
 *   GND    -> 230 GND
 *   3V3    -> 230 VCC
 *   230 RS -> GND (or 10-100k to GND for slope control)
 *   230 CANH / CANL -> bus; 120 ohm at each end
 *
 * Do not plug USB-C while the 12V->5V supply is on the 5V pin.
 *
 * Console (USB Serial @ 115200, or BLE NUS / Serial Bluetooth Terminal):
 *   all          sniff every ID
 *   <id>         filter one ID, e.g. 0x2E1 or 737
 *   50|125|250|500|800   bitrate kbit/s (re-inits TWAI)
 *   s            toggle SILENT (listen-only, default on)
 *   d            toggle DIFF (only changed payloads; default on)
 *   r            reset counters + diff history
 *   st           TWAI status (TEC/REC/bus-off)
 *   p            ping (also BOOT button on GPIO9)
 *   h            help
 *
 * Super Mini BOOT button (GPIO9, next to USB): click = ping over BLE+USB.
 * GPIO8 onboard LED blinks on ping. BLE does not dump the boot banner;
 * type p or click BOOT for a ping. No periodic heartbeat.
 *
 * Output (candump-like, one line per printed frame):
 *   <count>  <ID>  <RTR?>  <DLC>  <byte0 ...>
 *
 * BLE cannot carry an unfiltered 500 kbit/s bus. Leave DIFF on, or
 * filter to one ID, unless you are on USB on the bench.
 *
 * Library: NimBLE-Arduino (already in Arduino/libraries)
 *************************************************************************/

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <NimBLEDevice.h>
#include "driver/twai.h"
#include "driver/gpio.h"

// Super Mini pinout:
// https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/#pinout
#define CAN_TX_GPIO   GPIO_NUM_4
#define CAN_RX_GPIO   GPIO_NUM_5
#define BOOT_BTN_GPIO 9          // Super Mini BOOT, active-low, pulled up
#define LED_GPIO      8          // Super Mini LED, active-low
#define DEVICE_NAME   "C3-CAN"

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static uint32_t s_bitrate_bps = 500000;
static bool     s_filter_all  = true;
static uint32_t s_filter_id   = 0;
static bool     s_filter_ext  = false;
static bool     s_silent      = true;
static bool     s_diff        = true;   // default ON: BLE cannot dump a busy bus
static uint32_t s_frame_count = 0;
static uint32_t s_print_count = 0;
static uint32_t s_ble_drops   = 0;
static bool     s_twai_ok     = false;

static bool     s_ble_notify  = false;
static volatile bool s_ble_hello = false;
static NimBLECharacteristic *s_tx_char = nullptr;
static uint32_t s_ping_n = 0;

static volatile bool s_cmd_ready = false;
static char          s_cmd_buf[80];

struct IdHistory {
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
  bool     seen;
};
static IdHistory s_hist[64];
static int       s_hist_n = 0;

static IdHistory* find_hist(uint32_t id) {
  for (int i = 0; i < s_hist_n; i++) {
    if (s_hist[i].id == id) return &s_hist[i];
  }
  if (s_hist_n < 64) {
    s_hist[s_hist_n].id = id;
    s_hist[s_hist_n].seen = false;
    return &s_hist[s_hist_n++];
  }
  for (int i = 1; i < s_hist_n; i++) s_hist[i - 1] = s_hist[i];
  s_hist[s_hist_n - 1].id = id;
  s_hist[s_hist_n - 1].seen = false;
  return &s_hist[s_hist_n - 1];
}

static void led_blink() {
  digitalWrite(LED_GPIO, LOW);
  delay(30);
  digitalWrite(LED_GPIO, HIGH);
}

static void emit_raw(const char *s, size_t n) {
  Serial.write(s, n);
  if (!s_ble_notify || !s_tx_char || n == 0) return;
  // Stay under a conservative ATT payload even if MTU negotiation failed.
  const size_t chunk = 180;
  size_t off = 0;
  while (off < n) {
    size_t c = n - off;
    if (c > chunk) c = chunk;
    if (!s_tx_char->notify(reinterpret_cast<const uint8_t *>(s + off), c)) {
      s_ble_drops++;
      break;
    }
    off += c;
  }
}

static void emit(const char *s) {
  emit_raw(s, strlen(s));
}

static void emitln(const char *s) {
  emit(s);
  emit("\n");
}

static twai_timing_config_t timing_for(uint32_t bps) {
  twai_timing_config_t t;
  switch (bps) {
    case 50000:   t = TWAI_TIMING_CONFIG_50KBITS(); break;
    case 125000:  t = TWAI_TIMING_CONFIG_125KBITS(); break;
    case 250000:  t = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 800000:  t = TWAI_TIMING_CONFIG_800KBITS(); break;
    case 1000000: t = TWAI_TIMING_CONFIG_1MBITS(); break;
    default:      t = TWAI_TIMING_CONFIG_500KBITS(); break;
  }
  return t;
}

static twai_filter_config_t filter_for_current() {
  twai_filter_config_t f;
  if (s_filter_all) {
    f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    return f;
  }
  memset(&f, 0, sizeof(f));
  f.single_filter = true;
  if (s_filter_ext) {
    // Single-filter extended: ID in bits 31..3
    f.acceptance_code = (s_filter_id & TWAI_EXTD_ID_MASK) << 3;
    f.acceptance_mask = 0x7;   // match all 29 ID bits; ignore RTR/unused
  } else {
    // Single-filter standard: ID in bits 31..21
    f.acceptance_code = (s_filter_id & TWAI_STD_ID_MASK) << 21;
    f.acceptance_mask = 0x001FFFFF;
  }
  return f;
}

static bool can_install(uint32_t bitrate) {
  twai_stop();
  twai_driver_uninstall();

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX_GPIO, CAN_RX_GPIO,
      s_silent ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  g.rx_queue_len = 64;
  g.tx_queue_len = s_silent ? 0 : 8;
  g.alerts_enabled = TWAI_ALERT_NONE;

  twai_timing_config_t t = timing_for(bitrate);
  twai_filter_config_t f = filter_for_current();

  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err != ESP_OK) {
    char buf[64];
    snprintf(buf, sizeof(buf), "[err] twai install %s", esp_err_to_name(err));
    emitln(buf);
    s_twai_ok = false;
    return false;
  }
  err = twai_start();
  if (err != ESP_OK) {
    char buf[64];
    snprintf(buf, sizeof(buf), "[err] twai start %s", esp_err_to_name(err));
    emitln(buf);
    twai_driver_uninstall();
    s_twai_ok = false;
    return false;
  }
  s_bitrate_bps = bitrate;
  s_twai_ok = true;
  return true;
}

static void print_banner() {
  emitln("");
  emitln("=============================================");
  emitln(" can_sniffer_c3  (TWAI + SN65HVD230 + NUS)");
  char buf[96];
  snprintf(buf, sizeof(buf), " bitrate: %lu kbit/s",
           (unsigned long)(s_bitrate_bps / 1000));
  emitln(buf);
  if (s_filter_all) {
    emitln(" filter : ALL IDs");
  } else {
    snprintf(buf, sizeof(buf), " filter : ID 0x%lX%s",
             (unsigned long)s_filter_id, s_filter_ext ? " (ext)" : "");
    emitln(buf);
  }
  snprintf(buf, sizeof(buf), " mode   : %s",
           s_silent ? "SILENT (listen only)" : "ACTIVE (may ACK/TX)");
  emitln(buf);
  snprintf(buf, sizeof(buf), " diff   : %s",
           s_diff ? "ON (changed payloads only)" : "OFF");
  emitln(buf);
  emitln(" BLE    : C3-CAN  (NUS; phone: Serial Bluetooth Terminal)");
  emitln("=============================================");
  emitln(" cmds: all | <id> | <kbps> | s=silent d=diff r=reset st=status h=help");
  if (s_filter_all && !s_diff) {
    emitln("[warn] ALL + diff OFF will overflow BLE on a busy car bus");
  }
  emitln("");
}

static void print_status() {
  twai_status_info_t st = {};
  if (!s_twai_ok || twai_get_status_info(&st) != ESP_OK) {
    emitln("[err] twai status unavailable");
    return;
  }
  const char *state = "?";
  switch (st.state) {
    case TWAI_STATE_STOPPED:    state = "STOPPED"; break;
    case TWAI_STATE_RUNNING:    state = "RUNNING"; break;
    case TWAI_STATE_BUS_OFF:    state = "BUS_OFF"; break;
    case TWAI_STATE_RECOVERING: state = "RECOVERING"; break;
  }
  char buf[160];
  snprintf(buf, sizeof(buf),
           "[st] %s  tec=%lu rec=%lu  rxq=%lu missed=%lu overrun=%lu  "
           "frames=%lu printed=%lu ble_drops=%lu",
           state,
           (unsigned long)st.tx_error_counter,
           (unsigned long)st.rx_error_counter,
           (unsigned long)st.msgs_to_rx,
           (unsigned long)st.rx_missed_count,
           (unsigned long)st.rx_overrun_count,
           (unsigned long)s_frame_count,
           (unsigned long)s_print_count,
           (unsigned long)s_ble_drops);
  emitln(buf);
}

static void handle_cmd(const char *raw_in) {
  char raw[80];
  strncpy(raw, raw_in, sizeof(raw) - 1);
  raw[sizeof(raw) - 1] = 0;

  // trim
  char *s = raw;
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  size_t n = strlen(s);
  while (n && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' || s[n - 1] == '\n')) {
    s[--n] = 0;
  }
  if (n == 0) return;

  if (s[0] == 'h' || s[0] == '?') {
    emitln("cmds: all | <id> | 50|125|250|500|800 | s(silent) | d(diff) | r(eset) | st | p(ping) | h");
    emitln("BOOT button = ping. No CAN frames until the bus is wired.");
    return;
  }
  if (s[0] == 'p' && n == 1) {
    s_ping_n++;
    char buf[80];
    snprintf(buf, sizeof(buf), "[ping] n=%lu  up=%lus  ble=%s  twai=%s",
             (unsigned long)s_ping_n,
             (unsigned long)(millis() / 1000),
             s_ble_notify ? "notify" : "no-sub",
             s_twai_ok ? "up" : "down");
    emitln(buf);
    led_blink();
    return;
  }
  if (strcmp(s, "st") == 0) {
    print_status();
    return;
  }
  if (s[0] == 'r' && n == 1) {
    s_frame_count = 0;
    s_print_count = 0;
    s_ble_drops = 0;
    s_hist_n = 0;
    emitln("[ok] counters reset + diff history cleared");
    return;
  }
  if (s[0] == 'd' && n == 1) {
    s_diff = !s_diff;
    s_hist_n = 0;
    emit(s_diff ? "[ok] diff=ON\n" : "[ok] diff=OFF\n");
    if (!s_diff && s_filter_all) {
      emitln("[warn] ALL + diff OFF will overflow BLE on a busy car bus");
    }
    return;
  }
  if (s[0] == 's' && n == 1) {
    s_silent = !s_silent;
    can_install(s_bitrate_bps);
    print_banner();
    return;
  }
  if (strcasecmp(s, "all") == 0) {
    s_filter_all = true;
    s_filter_id = 0;
    s_filter_ext = false;
    can_install(s_bitrate_bps);
    print_banner();
    return;
  }

  char *end = nullptr;
  unsigned long v = strtoul(s, &end, 0);
  if (end != s && *end == 0) {
    static const uint32_t kbps_list[] = {50, 125, 250, 500, 800, 1000};
    bool is_kbps = false;
    for (uint32_t k : kbps_list) {
      if (v == k) {
        is_kbps = true;
        break;
      }
    }
    if (is_kbps) {
      can_install((uint32_t)v * 1000);
      print_banner();
    } else {
      s_filter_all = false;
      s_filter_id = (uint32_t)v;
      s_filter_ext = (s_filter_id > TWAI_STD_ID_MASK);
      can_install(s_bitrate_bps);
      print_banner();
    }
    return;
  }

  emit("[?] unknown: ");
  emitln(raw_in);
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *srv, NimBLEConnInfo &info) override {
    srv->updateConnParams(info.getConnHandle(), 12, 24, 0, 200);
    emitln("[ble] connected");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    s_ble_notify = false;
    emitln("[ble] disconnected");
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override {
    char buf[40];
    snprintf(buf, sizeof(buf), "[ble] mtu %u", (unsigned)mtu);
    emitln(buf);
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
    NimBLEAttValue v = c->getValue();
    size_t n = v.size();
    if (n >= sizeof(s_cmd_buf)) n = sizeof(s_cmd_buf) - 1;
    memcpy((void *)s_cmd_buf, v.data(), n);
    s_cmd_buf[n] = 0;
    s_cmd_ready = true;
  }
};

// Subscribe is on the TX/notify characteristic, not RX. Watching RX
// left s_ble_notify false, so the phone saw a connection and no text.
class TxCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t sub) override {
    s_ble_notify = (sub & 0x1) != 0;
    if (s_ble_notify) s_ble_hello = true;
  }
};

static ServerCB s_server_cb;
static RxCB     s_rx_cb;
static TxCB     s_tx_cb;

static void ble_begin() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(9);
  NimBLEDevice::setMTU(247);

  NimBLEServer *srv = NimBLEDevice::createServer();
  srv->setCallbacks(&s_server_cb);

  NimBLEService *svc = srv->createService(NUS_SERVICE_UUID);
  s_tx_char = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  s_tx_char->setCallbacks(&s_tx_cb);
  NimBLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(&s_rx_cb);

  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setName(DEVICE_NAME);
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->enableScanResponse(true);
  adv->start();
}

static void format_frame(char *buf, size_t buflen, const twai_message_t &msg) {
  int n = snprintf(buf, buflen, "%lu  %lX  %s  %u",
                   (unsigned long)s_frame_count,
                   (unsigned long)msg.identifier,
                   msg.rtr ? "RTR" : "   ",
                   (unsigned)msg.data_length_code);
  if (n < 0) {
    buf[0] = 0;
    return;
  }
  if (!msg.rtr) {
    uint8_t dlc = msg.data_length_code;
    if (dlc > 8) dlc = 8;
    for (uint8_t i = 0; i < dlc; i++) {
      int m = snprintf(buf + n, buflen - n, " %02X", msg.data[i]);
      if (m < 0) break;
      n += m;
      if ((size_t)n >= buflen) break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH);   // LED off (active-low)
  pinMode(BOOT_BTN_GPIO, INPUT_PULLUP);

  can_install(s_bitrate_bps);
  ble_begin();
  print_banner();
  if (s_twai_ok) emitln("[ok] TWAI running, BLE advertising as C3-CAN");
  emitln("[ok] type p or click BOOT for a ping (no CAN data until the bus is live)");
}

static void send_hello_if_needed() {
  if (!s_ble_hello) return;
  s_ble_hello = false;
  emitln("[ble] notify ok — type p or click BOOT");
}

static void poll_boot_button() {
  static uint32_t last_ms = 0;
  static int last = HIGH;
  if (millis() < 1500) return;          // ignore BOOT used for download
  int now = digitalRead(BOOT_BTN_GPIO);
  if (last == HIGH && now == LOW && (millis() - last_ms) > 80) {
    last_ms = millis();
    handle_cmd("p");
  }
  last = now;
}

void loop() {
  send_hello_if_needed();
  poll_boot_button();

  if (s_cmd_ready) {
    s_cmd_ready = false;
    handle_cmd(s_cmd_buf);
  }

  static String usb_line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (usb_line.length()) {
        handle_cmd(usb_line.c_str());
        usb_line = "";
      }
    } else if (usb_line.length() < 78) {
      usb_line += c;
    }
  }

  if (!s_twai_ok) {
    delay(50);
    return;
  }

  int n = 0;
  while (n < 32) {
    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) break;
    n++;
    s_frame_count++;

    if (!s_filter_all) {
      if (msg.identifier != s_filter_id) continue;
      if ((bool)msg.extd != s_filter_ext) continue;
    }

    if (s_diff && !msg.rtr) {
      IdHistory *h = find_hist(msg.identifier);
      uint8_t dlc = msg.data_length_code;
      if (dlc > 8) dlc = 8;
      bool changed = !h->seen || h->len != dlc || memcmp(h->data, msg.data, dlc) != 0;
      if (!changed) continue;
      memcpy(h->data, msg.data, dlc);
      h->len = dlc;
      h->seen = true;
    }

    char line[96];
    format_frame(line, sizeof(line), msg);
    emitln(line);
    s_print_count++;
  }

  static uint32_t last_recov;
  if (millis() - last_recov > 1000) {
    last_recov = millis();
    twai_status_info_t st = {};
    if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
      emitln("[twai] bus-off, recovering");
      twai_initiate_recovery();
    } else if (st.state == TWAI_STATE_STOPPED) {
      twai_start();
    }
  }
}
