// web_drive.h - drive the robot from a phone with no PC: this board serves a
// page with a stick on http://<board>/ and takes wheel commands over a
// WebSocket on ws://<board>/ws. No ROS involved; the sonar reflex and the
// command timeout still apply, because the commands go through the same
// hoverboard path as HBC1.
//
//   phone --HTTP GET /------> webTask (core 1) --> web_page.h
//   phone <==WebSocket /ws==> webTask --mailbox--> hoverboardTask (core 0)
//                                     <--status---
//
// Who drives: the web stick wins while it is held. Every web command (zeros
// included) claims the wheels for WEB_DRIVE_TIMEOUT_MS; HBC1 wheel targets
// from ROS are ignored meanwhile (the buzzer flag still applies). When the
// web goes quiet, the wheels are zeroed once and HBC1 takes over again - so
// cancel a Nav2 goal first if you don't want it to resume.
//
// Minimal server, by design: one HTTP request or one WebSocket at a time, a
// new WebSocket replaces the old one, text frames up to WS_MAX_FRAME bytes,
// no fragmentation, no TLS. During an OTA upload (ota.h) the wheels are held
// at zero and the stick is ignored. Anyone on the Wi-Fi can drive; set
// WEB_DRIVE_ENABLED 0 in config.h on a network you don't trust.
//
// Messages, page -> board (text):
//   "D <rpm_left> <rpm_right>"   wheel targets, + = forward, clamped to
//                                +-WEB_DRIVE_MAX_RPM
//   "S"                          stop
// board -> page, every WEB_STATUS_PERIOD_MS, one JSON object:
//   {"mb":1,"b":36.4,"bat":"ok","rl":0.0,"rr":0.0,"sf":512,"sl":0,"sr":0,
//    "near":0,"rfx":"","own":"idle","max":60}
//   bat = "ok", "low", "critical" or "unknown" (battery.h)
//   own = "web" (stick), "ROS" (HBC1 in the last second) or "idle".

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include "mbedtls/base64.h"
#include "mbedtls/sha1.h"

#include "config.h"
#include "web_page.h"

// ------------------------------------------------------- shared with core 0
// The web task only ever writes web_cmd and reads web_status; the hoverboard
// task does the reverse. Both under web_mux, both tiny.
struct WebCmd    { float l, r; uint32_t seq; };
struct WebStatus { bool mb, near; float batt, rpm_l, rpm_r; uint16_t sf, sl, sr; const char * reflex; const char * own; const char * bat; };

static portMUX_TYPE web_mux = portMUX_INITIALIZER_UNLOCKED;
static WebCmd    web_cmd = {0.0f, 0.0f, 0};
static WebStatus web_status = {false, false, 0, 0, 0, 0, 0, 0, "", "idle", "unknown"};

static void webPost(float l, float r) {
  l = constrain(l, -WEB_DRIVE_MAX_RPM, WEB_DRIVE_MAX_RPM);
  r = constrain(r, -WEB_DRIVE_MAX_RPM, WEB_DRIVE_MAX_RPM);
  portENTER_CRITICAL(&web_mux);
  web_cmd.l = l; web_cmd.r = r; web_cmd.seq++;
  portEXIT_CRITICAL(&web_mux);
}

// ---------------------------------------------------- hoverboardTask side
static uint32_t web_seen_seq = 0;
static uint32_t web_last_ms = 0;
static bool     web_owns = false;
static uint32_t ros_last_ms = 0;     // last HBC1, for the "own" field

// Called every hoverboardTask cycle, before the reflex. Applies a new web
// command, or hands the wheels back once the web has been quiet.
static void webDriveApply(Hoverboard & hb) {
#if WEB_DRIVE_ENABLED
  portENTER_CRITICAL(&web_mux);
  const WebCmd c = web_cmd;
  portEXIT_CRITICAL(&web_mux);
  if (c.seq != web_seen_seq) {
    web_seen_seq = c.seq;
    hb.setWheelRPM(c.l, c.r);
    web_last_ms = millis();
    web_owns = true;
  } else if (web_owns && millis() - web_last_ms > WEB_DRIVE_TIMEOUT_MS) {
    web_owns = false;
    hb.setWheelRPM(0.0f, 0.0f);   // never hand ROS a wheel still turning for the web
  }
#else
  (void)hb;
#endif
}

static bool webDriveOwns() { return web_owns; }

static void webDrivePublish(Hoverboard & hb, bool near, const char * reflex, uint16_t sf, uint16_t sl, uint16_t sr) {
#if WEB_DRIVE_ENABLED
  WebStatus s;
  s.mb = hb.isConnected(); s.near = near;
  s.batt = hb.getBatteryVolts();
  s.rpm_l = hb.getWheelRPM(HB_LEFT); s.rpm_r = hb.getWheelRPM(HB_RIGHT);
  s.sf = sf; s.sl = sl; s.sr = sr;
  s.reflex = reflex;
  s.own = web_owns ? "web" : (millis() - ros_last_ms < 1000 ? "ROS" : "idle");
  s.bat = batteryStateName();
  portENTER_CRITICAL(&web_mux);
  web_status = s;
  portEXIT_CRITICAL(&web_mux);
#else
  (void)hb; (void)near; (void)reflex; (void)sf; (void)sl; (void)sr;
#endif
}

#if WEB_DRIVE_ENABLED
// ------------------------------------------------------------ web task side
static WiFiServer web_server(WEB_DRIVE_PORT);
static WiFiClient ws_client;
static bool       ws_open = false;
static const size_t WS_MAX_FRAME = 256;
static uint8_t    ws_rx[WS_MAX_FRAME + 14];
static size_t     ws_rx_len = 0;

static void wsSend(uint8_t opcode, const char * data, size_t len) {
  uint8_t hdr[4];
  size_t h = 2;
  hdr[0] = 0x80 | opcode;                      // FIN + opcode, server frames unmasked
  if (len < 126) {
    hdr[1] = (uint8_t)len;
  } else {
    hdr[1] = 126; hdr[2] = (uint8_t)(len >> 8); hdr[3] = (uint8_t)len; h = 4;
  }
  ws_client.write(hdr, h);
  if (len) ws_client.write((const uint8_t *)data, len);
}

static void wsDrop(const char * why) {
  if (ws_open) Serial.printf("web: drive socket closed (%s)\n", why);
  if (ws_client) ws_client.stop();
  ws_open = false;
  ws_rx_len = 0;
  webPost(0.0f, 0.0f);       // a lost stick is a released stick
}

static void wsHandleText(char * p) {
  if (p[0] == 'D') {
    float l = 0, r = 0;
    if (sscanf(p + 1, "%f %f", &l, &r) == 2 && isfinite(l) && isfinite(r)) webPost(l, r);
  } else if (p[0] == 'S') {
    webPost(0.0f, 0.0f);
  }
}

// Parse every complete frame in ws_rx. Clients must mask; anything we don't
// handle (binary, fragments, oversized) closes the socket.
static void wsParse() {
  for (;;) {
    if (ws_rx_len < 2) return;
    const uint8_t op = ws_rx[0] & 0x0F;
    const bool fin = ws_rx[0] & 0x80, masked = ws_rx[1] & 0x80;
    size_t len = ws_rx[1] & 0x7F, hl = 2;
    if (len == 126) {
      if (ws_rx_len < 4) return;
      len = ((size_t)ws_rx[2] << 8) | ws_rx[3]; hl = 4;
    } else if (len == 127) { wsDrop("frame too big"); return; }
    if (!masked || !fin || len > WS_MAX_FRAME) { wsDrop("unsupported frame"); return; }
    if (ws_rx_len < hl + 4 + len) return;      // wait for the rest
    const uint8_t * mask = ws_rx + hl;
    char * payload = (char *)ws_rx + hl + 4;
    for (size_t i = 0; i < len; i++) payload[i] ^= mask[i & 3];
    const char keep = payload[len];
    payload[len] = 0;

    if (op == 0x1) wsHandleText(payload);
    else if (op == 0x9) wsSend(0xA, payload, len);          // ping -> pong
    else if (op == 0x8) { wsSend(0x8, nullptr, 0); wsDrop("closed by page"); return; }
    else if (op != 0xA) { wsDrop("unsupported opcode"); return; }

    payload[len] = keep;
    const size_t used = hl + 4 + len;
    memmove(ws_rx, ws_rx + used, ws_rx_len - used);
    ws_rx_len -= used;
  }
}

static bool wsAcceptKey(const String & key, char * out, size_t out_len) {
  const String s = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  uint8_t sha[20];
#if ESP_IDF_VERSION_MAJOR >= 5
  if (mbedtls_sha1((const uint8_t *)s.c_str(), s.length(), sha) != 0) return false;
#else
  if (mbedtls_sha1_ret((const uint8_t *)s.c_str(), s.length(), sha) != 0) return false;
#endif
  size_t n = 0;
  if (mbedtls_base64_encode((uint8_t *)out, out_len - 1, &n, sha, sizeof(sha)) != 0) return false;
  out[n] = 0;
  return true;
}

// One new TCP connection: read its request head, then serve the page, open
// the WebSocket, or 404.
static void webHandleNew(WiFiClient c) {
  c.setNoDelay(true);
  String path, key, line;
  bool first = true;
  const uint32_t t0 = millis();
  while (c.connected() && millis() - t0 < 1500) {
    if (!c.available()) { vTaskDelay(1); continue; }
    const char ch = c.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (line.length() < 300) line += ch; continue; }
    if (line.length() == 0) break;             // end of headers
    if (first) {                               // "GET /path HTTP/1.1"
      const int a = line.indexOf(' '), b = line.indexOf(' ', a + 1);
      if (line.startsWith("GET ") && b > a) path = line.substring(a + 1, b);
      first = false;
    } else {
      String lower = line; lower.toLowerCase();
      if (lower.startsWith("sec-websocket-key:")) { key = line.substring(18); key.trim(); }
    }
    line = "";
  }

  if (path == "/ws" && key.length()) {
    char accept[40];
    if (!wsAcceptKey(key, accept, sizeof(accept))) { c.stop(); return; }
    if (ws_open) wsDrop("replaced by a new page");
    c.printf("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
             "Sec-WebSocket-Accept: %s\r\n\r\n", accept);
    ws_client = c;
    ws_open = true;
    ws_rx_len = 0;
    Serial.printf("web: drive socket from %s\n", c.remoteIP().toString().c_str());
  } else if (path == "/" || path == "/index.html") {
    const size_t n = strlen_P(WEB_PAGE);
    c.printf("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
             "Content-Length: %u\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n", (unsigned)n);
    for (size_t off = 0; off < n; off += 1024) c.write((const uint8_t *)WEB_PAGE + off, min((size_t)1024, n - off));
    c.stop();
  } else {
    c.print("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    c.stop();
  }
}

static void webSendStatus() {
  portENTER_CRITICAL(&web_mux);
  const WebStatus s = web_status;
  portEXIT_CRITICAL(&web_mux);
  char buf[256];
  const int n = snprintf(buf, sizeof(buf),
      "{\"mb\":%d,\"b\":%.1f,\"bat\":\"%s\",\"rl\":%.1f,\"rr\":%.1f,\"sf\":%u,\"sl\":%u,\"sr\":%u,"
      "\"near\":%d,\"rfx\":\"%s\",\"own\":\"%s\",\"max\":%.0f}",
      s.mb, s.batt, s.bat, s.rpm_l, s.rpm_r, s.sf, s.sl, s.sr, s.near, s.reflex, s.own, WEB_DRIVE_MAX_RPM);
  if (n > 0) wsSend(0x1, buf, (size_t)min(n, (int)sizeof(buf) - 1));
}

static void webTask(void * arg) {
  (void)arg;
  bool started = false;
  uint32_t last_status_ms = 0;
  for (;;) {
    const bool up = WiFi.status() == WL_CONNECTED;
    if (up && !started) {
      web_server.begin();
      started = true;
    }
    // mDNS (hoverboard.local) is announced by the WiFi code in the .ino.

    if (started) {
      WiFiClient c = web_server.accept();
      if (c) webHandleNew(c);
    }

    if (ws_open) {
      if (!ws_client.connected()) {
        wsDrop("page gone");
      } else {
        while (ws_client.available() && ws_rx_len < sizeof(ws_rx)) {
          const int n = ws_client.read(ws_rx + ws_rx_len, sizeof(ws_rx) - ws_rx_len);
          if (n <= 0) break;
          ws_rx_len += (size_t)n;
        }
        wsParse();
        if (ws_open && millis() - last_status_ms >= WEB_STATUS_PERIOD_MS) {
          last_status_ms = millis();
          webSendStatus();
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
#endif  // WEB_DRIVE_ENABLED

static void webDriveBegin() {
#if WEB_DRIVE_ENABLED
  xTaskCreatePinnedToCore(webTask, "web", 6144, NULL, 1, NULL, 1);
  Serial.printf("web    drive page on port %u, max %.0f rpm, hands back after %lu ms\n",
                (unsigned)WEB_DRIVE_PORT, WEB_DRIVE_MAX_RPM, (unsigned long)WEB_DRIVE_TIMEOUT_MS);
#endif
}
