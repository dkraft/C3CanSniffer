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
 *   One letter = toggle streaming diffs for that sensor (phone-friendly).
 *   Two letters = system (so they never steal a sensor key).
 *
 *   w s y g v x h o l r z    toggle sensor stream
 *   n   names     c  classify now     d  dump-all changed frames
 *   si  silent    rs reset            st status
 *   p   ping      ?  help             all   classify, no dump
 *   <id>  lock one ID and dump it     50|125|250|500|800  kbit/s
 *
 * Super Mini BOOT button (GPIO9): ping. LED blinks on ping and new names.
 *
 * Boot starts from scratch: watch the bus, name IDs, do not log frames.
 *   [name] 2E1  10ms  n=340  b0=COUNTER b2=W b7=CSUM
 *   [Y] 2E1  b2=12  00 00 00 0C ...     (only while that sensor watch is on)
 *
 * BLE cannot carry an unfiltered 500 kbit/s bus. Dump-all is off at boot.
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
static bool     s_dump        = false;  // default OFF: classify, don't log
static bool     s_diff        = true;   // when dumping all IDs, only changed payloads
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

static void emit(const char *s);
static void emitln(const char *s);

// ---------------------------------------------------------------------------
// Classify on the C3 from a fresh boot. Do not log the bus.
// Byte roles first, then 16-bit fields, then vehicle logic:
//   wheels cluster; steer moves while wheels are quiet;
//   yaw follows steer when moving; GPS is the slow walker.
// One-letter BLE keys toggle streaming diffs of a named sensor.
// ---------------------------------------------------------------------------
#define CLS_MAX_IDS     96
#define CLS_MIN_FRAMES  24
#define CLS_EMIT_PER    4
#define CLS_PERIOD_MS   2000
#define CLS_MAX_CAND    32

enum FieldKind : uint8_t {
  FK_UNKNOWN = 0,
  FK_STATIC,
  FK_COUNTER,
  FK_CSUM,
  FK_MUX,
  FK_ANALOG,
  FK_W,   // wheels
  FK_S,   // steering
  FK_Y,   // yaw
  FK_G,   // GPS
  FK_V,   // vehicle speed
  FK_X,   // forward accel
  FK_H,   // heading
  FK_O,   // odometer
  FK_L,   // lateral accel
  FK_R,   // roll
  FK_Z,   // vertical / G
  FK_COUNT
};

struct ByteStat {
  uint8_t  last, minv, maxv;
  uint16_t changes, up, down, wrap, d1;
  bool     seen;
};

struct PairStat {
  uint16_t last_le, last_be;
  uint16_t sm_le, sm_be;     // |delta| < 64
  uint16_t ch_le, ch_be;
  uint8_t  dn_le, dn_be;     // decreases (odo vs gps)
  bool     seen;
};

struct IdStat {
  uint32_t  id;
  bool      used;
  bool      extd;
  bool      dirty;
  uint8_t   dlc;
  uint32_t  frames;
  uint32_t  last_ms;
  uint16_t  period_ms;
  ByteStat  b[8];
  PairStat  p[7];
  FieldKind kind[8];
  uint8_t   width[8];        // 2 = lo byte of a 16-bit field
  uint8_t   be[8];
  uint8_t   changed;
};

struct SigCand {
  uint32_t  id;
  bool      extd, used;
  uint8_t   byte, be, id_idx;
  uint16_t  last, sm, ch, period_ms;
  uint16_t  s_quiet;         // changed while wheels quiet
  uint16_t  y_ag, y_dg;      // yaw vs steer sign
  uint16_t  x_ag, x_dg;      // vs wheel accel
  uint16_t  l_ag, l_dg;      // vs yaw/steer while moving
  FieldKind named;
};

static bool kind_is_vehicle(FieldKind k) {
  return k >= FK_W && k < FK_COUNT;
}

static const char *kind_name(FieldKind k) {
  switch (k) {
    case FK_STATIC:  return "STATIC";
    case FK_COUNTER: return "COUNTER";
    case FK_CSUM:    return "CSUM";
    case FK_MUX:     return "MUX";
    case FK_ANALOG:  return "ANALOG";
    case FK_W:       return "W";
    case FK_S:       return "S";
    case FK_Y:       return "Y";
    case FK_G:       return "G";
    case FK_V:       return "V";
    case FK_X:       return "X";
    case FK_H:       return "H";
    case FK_O:       return "O";
    case FK_L:       return "L";
    case FK_R:       return "R";
    case FK_Z:       return "Z";
    default:         return "?";
  }
}

static IdStat   s_ids[CLS_MAX_IDS];
static SigCand  s_cands[CLS_MAX_CAND];
static bool     s_report[FK_COUNT];
static uint32_t s_led_until    = 0;
static int      s_emit_cursor  = 0;
static int16_t  s_w_prev       = 0;

static void sat16(uint16_t *p) {
  if (*p < 0xFFFF) (*p)++;
}

static int cls_find(uint32_t id, bool extd, bool add) {
  int free_i = -1;
  for (int i = 0; i < CLS_MAX_IDS; i++) {
    if (!s_ids[i].used) {
      if (free_i < 0) free_i = i;
      continue;
    }
    if (s_ids[i].id == id && s_ids[i].extd == extd) return i;
  }
  if (!add || free_i < 0) return -1;
  memset(&s_ids[free_i], 0, sizeof(s_ids[free_i]));
  s_ids[free_i].used = true;
  s_ids[free_i].id = id;
  s_ids[free_i].extd = extd;
  return free_i;
}

static int cls_observe(const twai_message_t &msg) {
  if (msg.rtr) return -1;
  int idx = cls_find(msg.identifier, (bool)msg.extd, true);
  if (idx < 0) return -1;
  IdStat &s = s_ids[idx];
  uint32_t now = millis();
  if (s.frames > 0) {
    uint32_t dt = now - s.last_ms;
    if (dt > 0xFFFF) dt = 0xFFFF;
    if (s.period_ms == 0) s.period_ms = (uint16_t)dt;
    else s.period_ms = (uint16_t)((s.period_ms * 7u + (uint16_t)dt) / 8u);
  }
  s.last_ms = now;
  s.frames++;
  s.changed = 0;

  uint8_t dlc = msg.data_length_code;
  if (dlc > 8) dlc = 8;
  s.dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++) {
    ByteStat &b = s.b[i];
    uint8_t v = msg.data[i];
    if (!b.seen) {
      b.seen = true;
      b.last = b.minv = b.maxv = v;
      continue;
    }
    if (v != b.last) {
      s.changed |= (uint8_t)(1u << i);
      sat16(&b.changes);
    }
    if (v < b.minv) b.minv = v;
    if (v > b.maxv) b.maxv = v;
    uint8_t fwd = (uint8_t)(v - b.last);
    uint8_t back = (uint8_t)(b.last - v);
    if (fwd == 1) {
      sat16(&b.up);
      sat16(&b.d1);
      if (b.last > v) sat16(&b.wrap);
    } else if (back == 1) {
      sat16(&b.down);
      sat16(&b.d1);
      if (v > b.last) sat16(&b.wrap);
    }
    b.last = v;
  }
  for (uint8_t i = 0; i + 1 < dlc; i++) {
    PairStat &p = s.p[i];
    uint16_t le = (uint16_t)msg.data[i] | ((uint16_t)msg.data[i + 1] << 8);
    uint16_t be = ((uint16_t)msg.data[i] << 8) | (uint16_t)msg.data[i + 1];
    if (!p.seen) {
      p.seen = true;
      p.last_le = le;
      p.last_be = be;
      continue;
    }
    if (le != p.last_le) {
      sat16(&p.ch_le);
      int16_t d = (int16_t)(le - p.last_le);
      uint16_t ad = d < 0 ? (uint16_t)(-d) : (uint16_t)d;
      if (ad < 64) sat16(&p.sm_le);
      if ((int16_t)le < (int16_t)p.last_le && ad < 0x4000) {
        if (p.dn_le < 0xFF) p.dn_le++;
      }
    }
    if (be != p.last_be) {
      sat16(&p.ch_be);
      int16_t d = (int16_t)(be - p.last_be);
      uint16_t ad = d < 0 ? (uint16_t)(-d) : (uint16_t)d;
      if (ad < 64) sat16(&p.sm_be);
      if ((int16_t)be < (int16_t)p.last_be && ad < 0x4000) {
        if (p.dn_be < 0xFF) p.dn_be++;
      }
    }
    p.last_le = le;
    p.last_be = be;
  }
  return idx;
}

static FieldKind classify_byte(const IdStat &s, int i) {
  if (kind_is_vehicle(s.kind[i])) return s.kind[i];
  const ByteStat &b = s.b[i];
  if (!b.seen || s.frames < CLS_MIN_FRAMES) return FK_UNKNOWN;
  if (b.changes == 0) return FK_STATIC;
  uint16_t ch = b.changes;
  uint8_t rng = (uint8_t)(b.maxv - b.minv);
  bool mostly_inc = (b.up > (uint16_t)(b.down * 3u + 2u)) &&
                    (b.d1 >= (uint16_t)((ch * 3u) / 4u));
  if (mostly_inc) return FK_COUNTER;
  bool last = (i == (int)s.dlc - 1);
  if (last && ch * 3u > s.frames * 2u && b.d1 * 2u < ch) return FK_CSUM;
  if (i == 0 && rng >= 1 && rng <= 15 &&
      ch * 4u < s.frames && b.d1 * 2u <= ch) return FK_MUX;
  return FK_ANALOG;
}

static void cls_emit_id(const IdStat &s) {
  char buf[160];
  int n = snprintf(buf, sizeof(buf), "[name] %lX%s  %ums  n=%lu",
                   (unsigned long)s.id, s.extd ? "e" : "",
                   (unsigned)s.period_ms, (unsigned long)s.frames);
  if (n < 0) return;
  bool any = false;
  for (uint8_t i = 0; i < s.dlc && (size_t)n < sizeof(buf); i++) {
    if (s.kind[i] == FK_UNKNOWN) continue;
    any = true;
    int m = snprintf(buf + n, sizeof(buf) - (size_t)n, "  b%u=%s",
                     (unsigned)i, kind_name(s.kind[i]));
    if (m < 0) break;
    n += m;
  }
  if (!any && (size_t)n < sizeof(buf)) {
    snprintf(buf + n, sizeof(buf) - (size_t)n, "  (warming)");
  }
  emitln(buf);
}

static void cls_classify() {
  for (int i = 0; i < CLS_MAX_IDS; i++) {
    IdStat &s = s_ids[i];
    if (!s.used || s.frames < CLS_MIN_FRAMES) continue;
    bool changed = false;
    for (uint8_t b = 0; b < s.dlc; b++) {
      if (kind_is_vehicle(s.kind[b])) continue;
      FieldKind k = classify_byte(s, b);
      if (k != FK_UNKNOWN && k != s.kind[b]) {
        s.kind[b] = k;
        changed = true;
      }
    }
    if (changed) s.dirty = true;
  }
  rebuild_cands();
  cls_assign_vehicle();
}

static void cls_emit_pending() {
  int sent = 0;
  for (int n = 0; n < CLS_MAX_IDS && sent < CLS_EMIT_PER; n++) {
    int i = s_emit_cursor++;
    if (s_emit_cursor >= CLS_MAX_IDS) s_emit_cursor = 0;
    if (!s_ids[i].used || !s_ids[i].dirty) continue;
    cls_emit_id(s_ids[i]);
    s_ids[i].dirty = false;
    sent++;
    digitalWrite(LED_GPIO, LOW);
    s_led_until = millis() + 40;
  }
}

static void cls_print_names() {
  int shown = 0;
  for (int i = 0; i < CLS_MAX_IDS; i++) {
    if (!s_ids[i].used) continue;
    cls_emit_id(s_ids[i]);
    s_ids[i].dirty = false;
    shown++;
  }
  if (!shown) {
    emitln("[name] none yet (need a live bus for ~1 s)");
  } else {
    char buf[48];
    snprintf(buf, sizeof(buf), "[name] %d IDs", shown);
    emitln(buf);
  }
}

static int cls_id_count() {
  int n = 0;
  for (int i = 0; i < CLS_MAX_IDS; i++) if (s_ids[i].used) n++;
  return n;
}

static void cls_reset() {
  memset(s_ids, 0, sizeof(s_ids));
  memset(s_cands, 0, sizeof(s_cands));
  s_emit_cursor = 0;
  s_w_prev = 0;
}

static uint16_t dec_u16(const uint8_t *d, uint8_t i, uint8_t be) {
  if (be) return ((uint16_t)d[i] << 8) | d[i + 1];
  return (uint16_t)d[i] | ((uint16_t)d[i + 1] << 8);
}

static uint8_t better_be(const PairStat &p) {
  return (p.sm_be > p.sm_le) ? 1 : 0;
}

static void mark_pair(int id_idx, uint8_t byte, uint8_t be, FieldKind k) {
  if (id_idx < 0 || id_idx >= CLS_MAX_IDS) return;
  IdStat &s = s_ids[id_idx];
  if (byte + 1 >= s.dlc) return;
  if (kind_is_vehicle(s.kind[byte]) && s.kind[byte] != k) return;
  s.kind[byte] = k;
  s.kind[byte + 1] = k;
  s.width[byte] = 2;
  s.be[byte] = be;
  s.dirty = true;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    if (s_cands[i].used && s_cands[i].id_idx == (uint8_t)id_idx &&
        s_cands[i].byte == byte) {
      s_cands[i].named = k;
      s_cands[i].be = be;
    }
  }
}

static void rebuild_cands() {
  SigCand old[CLS_MAX_CAND];
  memcpy(old, s_cands, sizeof(old));
  memset(s_cands, 0, sizeof(s_cands));
  int n = 0;
  for (int i = 0; i < CLS_MAX_IDS && n < CLS_MAX_CAND; i++) {
    IdStat &s = s_ids[i];
    if (!s.used || s.frames < CLS_MIN_FRAMES || s.dlc < 2) continue;
    for (uint8_t b = 0; b + 1 < s.dlc && n < CLS_MAX_CAND; b++) {
      if (s.kind[b] == FK_COUNTER || s.kind[b] == FK_CSUM) continue;
      if (s.kind[b + 1] == FK_COUNTER || s.kind[b + 1] == FK_CSUM) continue;
      PairStat &p = s.p[b];
      if (!p.seen) continue;
      uint8_t be = better_be(p);
      uint16_t ch = be ? p.ch_be : p.ch_le;
      uint16_t sm = be ? p.sm_be : p.sm_le;
      if (ch < 1) continue;
      if (s.period_ms < 50 && ch < 3) continue;
      SigCand &c = s_cands[n];
      c.used = true;
      c.id = s.id;
      c.extd = s.extd;
      c.byte = b;
      c.be = be;
      c.id_idx = (uint8_t)i;
      c.last = be ? p.last_be : p.last_le;
      c.sm = sm;
      c.ch = ch;
      c.period_ms = s.period_ms;
      c.named = kind_is_vehicle(s.kind[b]) ? s.kind[b] : FK_UNKNOWN;
      for (int o = 0; o < CLS_MAX_CAND; o++) {
        if (!old[o].used) continue;
        if (old[o].id == c.id && old[o].byte == c.byte && old[o].be == c.be) {
          c.s_quiet = old[o].s_quiet;
          c.y_ag = old[o].y_ag;
          c.y_dg = old[o].y_dg;
          c.x_ag = old[o].x_ag;
          c.x_dg = old[o].x_dg;
          c.l_ag = old[o].l_ag;
          c.l_dg = old[o].l_dg;
          break;
        }
      }
      n++;
    }
  }
}

static int cand_wmean(int *nw) {
  int32_t sum = 0;
  int n = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    if (s_cands[i].used && s_cands[i].named == FK_W) {
      sum += s_cands[i].last;
      n++;
    }
  }
  if (nw) *nw = n;
  return n ? (int)(sum / n) : 0;
}

static void assign_wheels() {
  int idx[CLS_MAX_CAND];
  int n = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!c.used) continue;
    if (c.named != FK_UNKNOWN && c.named != FK_W) continue;
    if (c.period_ms < 2 || c.period_ms > 25) continue;
    if (c.ch < 8 || c.sm * 2 < c.ch) continue;
    idx[n++] = i;
  }
  if (n < 3) return;
  int best = 0, best_a = 0;
  for (int a = 0; a < n; a++) {
    int cnt = 0;
    uint16_t la = s_cands[idx[a]].last;
    for (int b = 0; b < n; b++) {
      uint16_t lb = s_cands[idx[b]].last;
      uint16_t d = la > lb ? la - lb : lb - la;
      uint16_t tol = la / 8;
      if (tol < 80) tol = 80;
      if (d <= tol) cnt++;
    }
    if (cnt > best) {
      best = cnt;
      best_a = a;
    }
  }
  if (best < 3) return;
  uint16_t center = s_cands[idx[best_a]].last;
  uint16_t tol = center / 8;
  if (tol < 80) tol = 80;
  int marked = 0;
  for (int a = 0; a < n && marked < 4; a++) {
    uint16_t lb = s_cands[idx[a]].last;
    uint16_t d = center > lb ? center - lb : lb - center;
    if (d > tol) continue;
    SigCand &c = s_cands[idx[a]];
    mark_pair(c.id_idx, c.byte, c.be, FK_W);
    marked++;
  }
}

static bool cand_free(const SigCand &c) {
  return c.used && (c.named == FK_UNKNOWN);
}

static void assign_speed() {
  int nw = 0;
  int wmean = cand_wmean(&nw);
  if (nw < 3) return;
  int best = -1;
  uint16_t best_d = 0xFFFF;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 25 || c.sm * 2 < c.ch) continue;
    uint16_t d = c.last > (uint16_t)wmean ? c.last - (uint16_t)wmean
                                          : (uint16_t)wmean - c.last;
    uint16_t tol = (uint16_t)wmean / 10;
    if (tol < 40) tol = 40;
    if (d <= tol && d < best_d) {
      best_d = d;
      best = i;
    }
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_V);
  }
}

static void assign_steer() {
  int best = -1;
  uint16_t best_q = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 40 || c.sm * 2 < c.ch) continue;
    if (c.s_quiet < 4) continue;
    if (c.s_quiet > best_q) {
      best_q = c.s_quiet;
      best = i;
    }
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_S);
  }
}

static void assign_yaw() {
  int best = -1;
  uint16_t best_ag = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 25) continue;
    if (c.y_ag < 8 || c.y_ag < (uint16_t)(c.y_dg * 2u + 4u)) continue;
    if (c.y_ag > best_ag) {
      best_ag = c.y_ag;
      best = i;
    }
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_Y);
  }
}

static void assign_gps() {
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms < 80 || c.ch < 2) continue;
    uint8_t dn = c.be ? s_ids[c.id_idx].p[c.byte].dn_be
                      : s_ids[c.id_idx].p[c.byte].dn_le;
    if (dn == 0) continue;   // monotonic is odo, not position
    mark_pair(c.id_idx, c.byte, c.be, FK_G);
  }
}

static void assign_odo() {
  int best = -1;
  uint16_t best_ch = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms < 15 || c.ch < 5) continue;
    uint8_t dn = c.be ? s_ids[c.id_idx].p[c.byte].dn_be
                      : s_ids[c.id_idx].p[c.byte].dn_le;
    if (dn != 0) continue;
    if (c.ch > best_ch) {
      best_ch = c.ch;
      best = i;
    }
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_O);
  }
}

static void assign_forward() {
  int best = -1;
  uint16_t best_ag = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 25) continue;
    if (c.x_ag < 8 || c.x_ag < (uint16_t)(c.x_dg * 2u + 4u)) continue;
    if (c.x_ag > best_ag) {
      best_ag = c.x_ag;
      best = i;
    }
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_X);
  }
}

static void assign_lateral() {
  int best = -1;
  uint16_t best_ag = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 25) continue;
    if (c.l_ag < 8 || c.l_ag < (uint16_t)(c.l_dg * 2u + 4u)) continue;
    if (c.l_ag > best_ag) {
      best_ag = c.l_ag;
      best = i;
    }
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_L);
  }
}

static void assign_heading() {
  int best = -1;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms < 40 || c.period_ms > 300) continue;
    if (c.ch < 4) continue;
    uint8_t dn = c.be ? s_ids[c.id_idx].p[c.byte].dn_be
                      : s_ids[c.id_idx].p[c.byte].dn_le;
    if (dn == 0) continue;
    best = i;
    break;
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_H);
  }
}

static void assign_vert() {
  int best = -1;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 25) continue;
    if (c.sm < 8 || c.sm < c.ch) continue;
    if (c.last < 1000 || c.last > 60000) continue;
    best = i;
    break;
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_Z);
  }
}

static void assign_roll() {
  int best = -1;
  uint16_t yper = 0;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    if (s_cands[i].used && s_cands[i].named == FK_Y) yper = s_cands[i].period_ms;
  }
  if (yper == 0) return;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!cand_free(c) || c.period_ms > 25) continue;
    int pd = (int)c.period_ms - (int)yper;
    if (pd < 0) pd = -pd;
    if (pd > 5) continue;
    uint8_t dn = c.be ? s_ids[c.id_idx].p[c.byte].dn_be
                      : s_ids[c.id_idx].p[c.byte].dn_le;
    if (dn == 0 || c.ch < 8) continue;
    best = i;
    break;
  }
  if (best >= 0) {
    SigCand &c = s_cands[best];
    mark_pair(c.id_idx, c.byte, c.be, FK_R);
  }
}

static void cls_assign_vehicle() {
  assign_wheels();
  assign_speed();
  assign_steer();
  assign_yaw();
  assign_gps();
  assign_odo();
  assign_forward();
  assign_lateral();
  assign_heading();
  assign_vert();
  assign_roll();
}

static void cls_score_live(const twai_message_t &msg) {
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!c.used || c.id != msg.identifier || c.extd != (bool)msg.extd) continue;
    if (c.byte + 1 >= msg.data_length_code) continue;
    c.last = dec_u16(msg.data, c.byte, c.be);
  }
  int nw = 0;
  int wmean = cand_wmean(&nw);
  int dw = wmean - s_w_prev;
  bool wquiet = nw >= 3 && dw > -3 && dw < 3;
  bool wmoving = nw >= 3 && wmean > 30;
  int16_t steer = 0;
  bool have_s = false;
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    if (s_cands[i].used && s_cands[i].named == FK_S) {
      steer = (int16_t)s_cands[i].last;
      have_s = true;
      break;
    }
  }
  for (int i = 0; i < CLS_MAX_CAND; i++) {
    SigCand &c = s_cands[i];
    if (!c.used || c.named != FK_UNKNOWN) continue;
    if (c.id != msg.identifier || c.extd != (bool)msg.extd) continue;
    uint8_t chm = s_ids[c.id_idx].changed;
    bool chg = (chm & (uint8_t)(1u << c.byte)) != 0 ||
               (chm & (uint8_t)(1u << (c.byte + 1))) != 0;
    int16_t sv = (int16_t)c.last;
    if (wquiet && chg) sat16(&c.s_quiet);
    if (wmoving && have_s) {
      bool same = ((sv >= 0) == (steer >= 0)) || (sv > -8 && sv < 8);
      if (same) sat16(&c.y_ag);
      else sat16(&c.y_dg);
      if ((sv >= 0) == (steer >= 0)) sat16(&c.l_ag);
      else sat16(&c.l_dg);
    }
    if (nw >= 3) {
      bool xs = ((sv >= 0) == (dw >= 0)) || (dw > -3 && dw < 3);
      if (xs) sat16(&c.x_ag);
      else sat16(&c.x_dg);
    }
  }
  s_w_prev = (int16_t)wmean;
}

static FieldKind kind_from_key(char c) {
  switch (c) {
    case 'W': case 'w': return FK_W;
    case 'S': case 's': return FK_S;
    case 'Y': case 'y': return FK_Y;
    case 'G': case 'g': return FK_G;
    case 'V': case 'v': return FK_V;
    case 'X': case 'x': return FK_X;
    case 'H': case 'h': return FK_H;
    case 'O': case 'o': return FK_O;
    case 'L': case 'l': return FK_L;
    case 'R': case 'r': return FK_R;
    case 'Z': case 'z': return FK_Z;
    default:            return FK_UNKNOWN;
  }
}

static int cls_count_kind(FieldKind k) {
  int n = 0;
  for (int i = 0; i < CLS_MAX_IDS; i++) {
    if (!s_ids[i].used) continue;
    for (uint8_t b = 0; b < s_ids[i].dlc; b++) {
      if (s_ids[i].kind[b] == k) n++;
    }
  }
  return n;
}

static bool cls_any_watch() {
  for (int i = 1; i < FK_COUNT; i++) {
    if (s_report[i]) return true;
  }
  return false;
}

static void emit_watch_status() {
  char buf[96];
  int n = snprintf(buf, sizeof(buf), " watch  :");
  bool any = false;
  static const FieldKind ks[] = {
    FK_W, FK_S, FK_Y, FK_G, FK_V, FK_X, FK_H, FK_O, FK_L, FK_R, FK_Z
  };
  for (unsigned i = 0; i < sizeof(ks) / sizeof(ks[0]); i++) {
    FieldKind k = ks[i];
    if (!s_report[k] || (size_t)n >= sizeof(buf)) continue;
    int m = snprintf(buf + n, sizeof(buf) - (size_t)n, " %s", kind_name(k));
    if (m < 0) break;
    n += m;
    any = true;
  }
  if (!any && (size_t)n < sizeof(buf)) snprintf(buf + n, sizeof(buf) - (size_t)n, " none");
  emitln(buf);
}

static void toggle_watch(FieldKind k) {
  if (k == FK_UNKNOWN || k >= FK_COUNT) return;
  s_report[k] = !s_report[k];
  char buf[80];
  snprintf(buf, sizeof(buf), "[watch] %s %s  fields=%d",
           kind_name(k), s_report[k] ? "ON" : "OFF", cls_count_kind(k));
  emitln(buf);
  if (s_report[k] && cls_count_kind(k) == 0) {
    emitln("[watch] no fields named that yet — reporting starts when classified");
  }
}

// Print a line when a watched named byte changed (diff of that sensor only).
static bool cls_report_watches(const IdStat &s, const twai_message_t &msg) {
  if (!s.changed || msg.rtr) return false;
  bool sent = false;
  uint8_t dlc = s.dlc;
  if (dlc > 8) dlc = 8;
  for (uint8_t i = 0; i < dlc; i++) {
    FieldKind k = s.kind[i];
    if (k == FK_UNKNOWN || k >= FK_COUNT || !s_report[k]) continue;
    if (i > 0 && s.width[i - 1] == 2) continue;
    uint8_t chbits = (uint8_t)(1u << i);
    if (s.width[i] == 2 && i + 1 < dlc) chbits |= (uint8_t)(1u << (i + 1));
    if ((s.changed & chbits) == 0) continue;
    char buf[128];
    int n;
    if (s.width[i] == 2 && i + 1 < dlc) {
      uint16_t v = dec_u16(msg.data, i, s.be[i]);
      if (k == FK_S || k == FK_Y || k == FK_X || k == FK_L || k == FK_R) {
        n = snprintf(buf, sizeof(buf), "[%s] %lX%s  b%u=%d ",
                     kind_name(k), (unsigned long)s.id, s.extd ? "e" : "",
                     (unsigned)i, (int)(int16_t)v);
      } else {
        n = snprintf(buf, sizeof(buf), "[%s] %lX%s  b%u=%u ",
                     kind_name(k), (unsigned long)s.id, s.extd ? "e" : "",
                     (unsigned)i, (unsigned)v);
      }
    } else {
      n = snprintf(buf, sizeof(buf), "[%s] %lX%s  b%u=%02X ",
                   kind_name(k), (unsigned long)s.id, s.extd ? "e" : "",
                   (unsigned)i, (unsigned)msg.data[i]);
    }
    if (n < 0) continue;
    for (uint8_t j = 0; j < dlc && (size_t)n < sizeof(buf); j++) {
      int m = snprintf(buf + n, sizeof(buf) - (size_t)n, " %02X", msg.data[j]);
      if (m < 0) break;
      n += m;
    }
    emitln(buf);
    s_print_count++;
    sent = true;
  }
  return sent;
}

static void cls_poll() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms >= CLS_PERIOD_MS) {
    last_ms = now;
    cls_classify();
  }
  cls_emit_pending();
  if (s_led_until && (int32_t)(now - s_led_until) >= 0) {
    digitalWrite(LED_GPIO, HIGH);
    s_led_until = 0;
  }
}

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
  snprintf(buf, sizeof(buf), " dump   : %s",
           s_dump ? (s_diff ? "ON (changed payloads)" : "ON (every frame)")
                  : "OFF (classify only)");
  emitln(buf);
  snprintf(buf, sizeof(buf), " names  : %d IDs watched", cls_id_count());
  emitln(buf);
  emit_watch_status();
  emitln(" BLE    : C3-CAN  (NUS; phone: Serial Bluetooth Terminal)");
  emitln("=============================================");
  emitln(" cmds: w s y g v x h o l r z  (toggle stream)  n names  c classify  d dump-all");
  emitln(" sys : si silent  rs reset  st status  p ping  ? help  all  <id>");
  if (s_filter_all && s_dump && !s_diff) {
    emitln("[warn] ALL + dump every frame will overflow BLE on a busy car bus");
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
           "frames=%lu printed=%lu ble_drops=%lu  ids=%d dump=%s",
           state,
           (unsigned long)st.tx_error_counter,
           (unsigned long)st.rx_error_counter,
           (unsigned long)st.msgs_to_rx,
           (unsigned long)st.rx_missed_count,
           (unsigned long)st.rx_overrun_count,
           (unsigned long)s_frame_count,
           (unsigned long)s_print_count,
           (unsigned long)s_ble_drops,
           cls_id_count(),
           s_dump ? "on" : "off");
  emitln(buf);
  emit_watch_status();
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

  if (s[0] == '?' || strcmp(s, "he") == 0) {
    emitln("stream: w wheels  s steer  y yaw  g gps  v speed  x forward");
    emitln("        h heading o odo    l lat   r roll z vert/G");
    emitln("sys: n names  c classify  d dump-all  si silent  rs reset  st  p  ?");
    emitln("boot names IDs from scratch. a letter toggles that sensor's diffs.");
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
  if (strcmp(s, "rs") == 0) {
    s_frame_count = 0;
    s_print_count = 0;
    s_ble_drops = 0;
    s_hist_n = 0;
    cls_reset();
    emitln("[ok] counters + names reset");
    return;
  }
  if (s[0] == 'd' && n == 1) {
    s_dump = !s_dump;
    s_hist_n = 0;
    emitln(s_dump ? "[ok] dump=ON (changed payloads)" : "[ok] dump=OFF (classify only)");
    if (s_dump && s_filter_all) {
      emitln("[warn] ALL + dump will overflow BLE on a busy car bus");
    }
    return;
  }
  if (s[0] == 'n' && n == 1) {
    cls_classify();
    cls_print_names();
    return;
  }
  if (s[0] == 'c' && n == 1) {
    cls_classify();
    for (int i = 0; i < CLS_MAX_IDS; i++) {
      if (s_ids[i].used && s_ids[i].frames >= CLS_MIN_FRAMES) s_ids[i].dirty = true;
    }
    emitln("[cls] queued name lines");
    return;
  }
  if (strcmp(s, "si") == 0) {
    s_silent = !s_silent;
    can_install(s_bitrate_bps);
    print_banner();
    return;
  }
  if (n == 1) {
    FieldKind wk = kind_from_key(s[0]);
    if (wk != FK_UNKNOWN) {
      toggle_watch(wk);
      return;
    }
  }

  if (strcasecmp(s, "all") == 0) {
    s_filter_all = true;
    s_filter_id = 0;
    s_filter_ext = false;
    s_dump = false;
    s_diff = true;
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
      s_dump = true;
      s_diff = false;   // every frame of the locked ID
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
  emitln("[ok] classify on — w s y g toggle streams  n=names  ?=help");
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
  cls_poll();

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

  int cap = (s_dump || cls_any_watch()) ? 32 : 128;
  int n = 0;
  while (n < cap) {
    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) break;
    n++;
    s_frame_count++;
    int idx = cls_observe(msg);
    if (idx >= 0) cls_score_live(msg);
    bool watched = (idx >= 0) && cls_report_watches(s_ids[idx], msg);

    bool want_dump = s_dump;
    if (!s_filter_all) {
      if (msg.identifier != s_filter_id) continue;
      if ((bool)msg.extd != s_filter_ext) continue;
      want_dump = true;
    }
    if (watched) continue;   // already emitted a sensor-diff line
    if (!want_dump) continue;

    if (s_filter_all && s_diff && !msg.rtr) {
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
