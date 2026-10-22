// =============================================================================
//  mgmt_wsserver.h — WebSocket command endpoint for the WCB management relay
// =============================================================================
//
//  A second mouth for the SAME protocol MgmtRelay's USB serial port speaks. Every
//  line that arrives here goes to relaySerialLine(), the identical dispatcher the
//  USB reader in loop() feeds, so there is exactly one implementation of the relay
//  command surface and the two transports cannot drift.
//
//  ── DELIBERATELY THE SAME WIRE PROTOCOL AS NaviCore ─────────────────────────
//  NaviCore exposes ws://<softAPIP>/ws carrying newline-delimited UTF-8, reassembled
//  per socket, with the reply stream being a CONSOLE MIRROR rather than strict
//  request/response (NaviCore docs/PROTOCOLS.md:19, navicore_wsserver.h). This
//  endpoint is the same URI, the same framing and the same mirror semantics, so a
//  host app writes ONE client and points it at either box. The payloads
//  differ — NaviCore speaks JSON verbs, this speaks the WCB serial grammar
//  (";w<id>,<cmd>", bare text = broadcast) — but the transport does not.
//
//  ── THE CONCURRENCY RULE THIS FILE EXISTS TO OBEY ───────────────────────────
//  The httpd task is NOT the loop task. Its handler runs on Core 0 alongside the
//  ESP-NOW receive callback, and the same prohibition applies there as everywhere
//  else in this sketch: the WiFi/httpd side may only copy, enqueue and return.
//  relaySerialLine() calls wcb.send(), which blocks on ETM retries and touches the
//  ESP-NOW driver — exactly what must not happen beside the receive callback. So
//  the handler enqueues and drain() runs the command from loop() on Core 1. That is
//  the same split relayOutQueue already uses for inbound mesh lines, for the same
//  reason, and it is why replies go out through httpd_ws_send_frame_async() (the
//  API documented for sends "out of the scope of current request") rather than the
//  in-request send.
//
//  ── WHY A VALUE QUEUE, NOT A POINTER QUEUE ──────────────────────────────────
//  NaviCore queues by pointer out of PSRAM because a SET_CONFIG line approaches
//  98 KB. This relay has no such payload: its longest input is a relayed OTA
//  "?OTA,DATA,..." line at ~283 chars, which is why inBuf is 384 B in the .ino.
//  WS_LINE_MAX tracks that same cap, so a value queue costs 8 x 388 B and the
//  sketch keeps working on a plain ESP32 with no PSRAM. Keep the two in step: a
//  line this transport accepts but the USB path would reject is a silent
//  divergence between two mouths that are supposed to be identical.
//
//  ── THE LOGIN TOKEN (WCB_Client 1.19.0 example) ─────────────────────────────
//  The same protocol as a WCB's WebSocket and NaviCore's, so Intellex logs in to
//  all three the same way. With a token set (setToken), a session opens with the
//  line "[AUTH:REQUIRED]", is left out of the console mirror, and every line it
//  sends other than "?AUTH,<token>" is dropped unrun and answered "[AUTH:REQUIRED]".
//  "?AUTH,<token>" is compared in constant time: a match answers "[AUTH:OK]" and the
//  session is in for its lifetime (mirror on, commands run); a mismatch answers
//  "[AUTH:FAIL]", the session is closed once that line is out, and for 1 s after
//  any failure every ?AUTH on any session fails - one guess a second. With no token,
//  nothing is sent on open and "?AUTH,<anything>" answers "[AUTH:OK]", so a client
//  can always send it. A token set, changed or cleared applies to NEW sessions; one
//  already logged in keeps going. A session that has not logged in WS_LOGIN_MS (10 s)
//  after it opened is closed, and when the slots are full a new one evicts a session
//  that has not logged in before one that has.
//  A SESSION IS ITS SERIAL, NOT ITS FD. lwIP gives a closed socket's number to the next
//  connection - the LRU purge does exactly that under load - so a queued command or a
//  mirror subscription keyed on the fd alone would pass to whoever connects next, signed
//  in or not. Each queued line carries its session's serial; drain() runs it, and puts that
//  session on the mirror, only while the same session is still open and logged in.
//  The check runs here on the httpd task, inside feed(): it is a string compare,
//  and doing it before the queue means a line from a session that has not logged in
//  never reaches relaySerialLine() at all. The [AUTH:...] replies are owed per
//  session (WsAcc::notes) and sent by pump() on the loop task, the one place that
//  writes to the sockets.
// =============================================================================
#pragma once

#include <esp_http_server.h>
#include <WiFi.h>
#include <lwip/sockets.h>   // close() — the session-close hook must close the socket

// Defined in MgmtRelay.ino. Declared here because this header is included near the
// top of the sketch, long before the definition, and Arduino's auto-prototyping
// only covers the .ino itself.
void relaySerialLine(char* line);

namespace mgmtws {

// Keep in step with httpd_config_t::max_open_sockets in begin(). More clients than
// the server will accept is just dead slots; fewer means a connected client the
// sink never writes to, which is the silent-deafness bug this array exists to
// prevent (the same trap NaviCore hit with a single fd).
static const uint8_t  WS_MAX_CLIENTS     = 3;
// drain() runs ONE command per loop() pass, so a client that puts several lines in
// a single frame needs room. NaviCore measured 3 of 6 commands answered at depth 3
// with a zero enqueue timeout — the rest discarded in the handler with NOTHING sent
// back, which is the worst failure available here: the client waits forever for a
// reply to a command that was already thrown away.
static const uint8_t  WS_QUEUE_DEPTH     = 8;
// How long the handler waits for room rather than discarding. This is the httpd
// task, NOT loop, so blocking here costs a little latency on other sockets and
// nothing on the mesh. It turns a silent drop into ordinary backpressure.
static const uint32_t WS_ENQUEUE_WAIT_MS = 50;
// MUST match inBuf[] in MgmtRelay.ino — see the header note above.
static const size_t   WS_LINE_MAX        = 384;
// Output staging. Big enough to hold a burst of mesh lines between pump() calls
// without flushing mid-loop; small enough that one flush is ~one TCP segment.
static const size_t   WS_SINK_BUF        = 2048;
// Login token (see the header note). 8-32 characters; the sketch validates it.
static const size_t   WS_TOKEN_MAX       = 32;
static const uint32_t AUTH_LOCKOUT_MS    = 1000;   // every ?AUTH fails this long after a failure
enum : uint8_t { NOTE_REQUIRED = 0x01, NOTE_OK = 0x02, NOTE_FAIL = 0x04 };   // WsAcc::notes

// The token is written on the loop task (setToken) and read on the httpd task, so a
// copy is taken under this lock. The same lock guards each session's fd, notes and
// closing flag, which the httpd task sets and pump() reads on the loop task.
inline portMUX_TYPE authMux = portMUX_INITIALIZER_UNLOCKED;
inline char         authToken[WS_TOKEN_MAX + 1] = "";   // "" = no token
inline uint32_t     authFailMs = 0;                      // httpd task only: the last failed ?AUTH

// "" or nullptr = no token. Takes effect for the next session that opens.
inline void setToken(const char* t) {
  portENTER_CRITICAL(&authMux);
  strlcpy(authToken, t ? t : "", sizeof(authToken));
  portEXIT_CRITICAL(&authMux);
}
inline void tokenCopy(char* out) {   // out: WS_TOKEN_MAX + 1 bytes
  portENTER_CRITICAL(&authMux);
  memcpy(out, authToken, WS_TOKEN_MAX + 1);
  portEXIT_CRITICAL(&authMux);
}

// Constant time over the whole token length: the loop runs WS_TOKEN_MAX times and
// never exits early, so how long a guess takes says nothing about how much of it was
// right. (Its length is not secret enough to matter: 8-32 characters, and every
// failure costs a second.)
inline bool tokenEqual(const char* given, const char* token) {
  const size_t lg = strnlen(given, WS_LINE_MAX), lt = strnlen(token, WS_TOKEN_MAX);
  uint8_t diff = (uint8_t)(lg != lt);
  for (size_t i = 0; i < WS_TOKEN_MAX; i++) {
    const uint8_t g = i < lg ? (uint8_t)given[i] : 0, t = i < lt ? (uint8_t)token[i] : 0;
    diff |= (uint8_t)(g ^ t);
  }
  return diff == 0;
}

// The token part of "?AUTH,<token>" (or "" for a bare "?AUTH"), case-insensitive
// verb as everywhere on this console; nullptr for any other line.
inline const char* authArg(const char* s) {
  static const char k[] = "?auth";
  for (int i = 0; i < 5; i++) {
    char c = s[i];
    if (c >= 'A' && c <= 'Z') c += 32;
    if (c != k[i]) return nullptr;
  }
  if (s[5] == ',')  return s + 6;
  if (s[5] == '\0') return s + 5;
  return nullptr;
}

// A session that has not logged in by then is closed (pumpNotes): a socket that never
// sends ?AUTH must not hold one of the WS_MAX_CLIENTS slots for ever.
static const uint32_t WS_LOGIN_MS = 10000;

struct WsCmd {
  int      fd;                // client socket, for the async reply
  uint32_t serial;            // the SESSION that sent it (WsAcc::serial): an fd number is reused
  char     line[WS_LINE_MAX]; // NUL-terminated command
};

inline QueueHandle_t  wsQueue  = nullptr;
inline httpd_handle_t wsServer = nullptr;

// ── Reply sink ──────────────────────────────────────────────────────────────
// A Print that fans host-facing output out to every connected client. MgmtRelay.ino
// prints through `host` (Serial + this), so the handlers stay transport-unaware and
// a WebSocket client sees exactly what a USB console would.
//
// BUFFER, THEN SEND FROM pump(). Never send inside write(). write() runs in the
// middle of arbitrary printf calls on the loop task, and a TCP send there would put
// network I/O between two halves of a print. Buffering here and flushing at one
// known point in loop() keeps the blocking where it can be reasoned about.
//
// OVERFLOW DROPS WHOLE LINES. Truncating mid-line would hand the host half a JSON
// object (the mesh streams rc_hb/rc_ch telemetry through here), which fails at
// JSON.parse. Dropping to the next newline loses a sample instead.
class WsSink : public Print {
 public:
  void begin(int fd) {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) return;   // already known
    // FIRST client after a quiet period: start from a clean sheet. _dropping is set
    // when a pump() fails, which is exactly what happens as the last client leaves
    // mid-line — and without this reset it survives the disconnect and eats the
    // first whole line the NEXT client is sent. Only on the transition to live:
    // clearing while another client is connected would discard output buffered for it.
    if (!live()) { _len = 0; _dropping = false; }
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] < 0) { _fds[i] = fd; return; }
    _fds[0] = fd;   // full: evict the oldest rather than refuse the newcomer
  }
  void drop(int fd) { for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) _fds[i] = -1; }
  void end()        { for (int i = 0; i < WS_MAX_CLIENTS; i++) _fds[i] = -1; _len = 0; _dropping = false; }
  bool live() const {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] >= 0) return true;
    return false;
  }

  size_t write(uint8_t c) override {
    if (!live()) return 1;
    if (_dropping) { if (c == '\n') _dropping = false; return 1; }
    if (_len >= WS_SINK_BUF) {
      // FULL — flush here rather than drop. Only a line longer than the whole
      // buffer reaches this branch, i.e. a big config/stats dump, not the routine
      // telemetry that motivated the buffering. Losing a config reply is a hard
      // failure; a few ms of jitter on an explicit user action is not.
      if (!pump()) { _dropping = true; return 1; }   // send failed: client is gone
    }
    _buf[_len++] = (char)c;
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  using Print::write;

  // Called from loop() (and from write() when the buffer fills). Broadcasts to every
  // connected client; one that has gone away is dropped individually rather than
  // taking the others down with it. Returns false only when nobody is left.
  bool pump() {
    if (!_len || !wsServer) return live();
    // NEVER CUT THROUGH A UTF-8 SEQUENCE. This is a TEXT frame and RFC 6455 8.1
    // requires each one to be valid UTF-8 on its own. The buffer flushes on a BYTE
    // count, so a multi-byte character in a board alias straddles the boundary and
    // the frame ends in a bare continuation byte. A conforming client fails the
    // connection with 1007, the reconnect replays the same bytes at the same
    // alignment, and the link dies in a loop that looks like flaky WiFi. Hold the
    // partial character back for the next frame instead; it is at most 3 bytes.
    size_t send = _utf8SafeLen(_buf, _len);
    if (!send) send = _len;          // cannot improve it — send rather than stall
    httpd_ws_frame_t f = {};
    f.final   = true;
    f.type    = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t*)_buf;
    f.len     = send;
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
      if (_fds[i] < 0) continue;
      if (httpd_ws_send_frame_async(wsServer, _fds[i], &f) != ESP_OK) _fds[i] = -1;
    }
    const size_t left = _len - send;
    if (left) memmove(_buf, _buf + send, left);
    _len = left;
    return live();
  }

 private:
  // Longest prefix of b[0..n) that does not end part-way through a UTF-8 sequence.
  // Returns n when the tail is already complete — the overwhelmingly common case
  // (pure ASCII), so this costs a couple of compares per flush.
  static size_t _utf8SafeLen(const char* b, size_t n) {
    if (!n) return 0;
    size_t back = 0;
    while (back < 3 && back < n) {
      unsigned char c = (unsigned char)b[n - 1 - back];
      if ((c & 0xC0) == 0x80) { back++; continue; }        // continuation byte
      size_t need = (c & 0x80) == 0x00 ? 1 :
                    (c & 0xE0) == 0xC0 ? 2 :
                    (c & 0xF0) == 0xE0 ? 3 :
                    (c & 0xF8) == 0xF0 ? 4 : 1;
      return (back + 1 >= need) ? n : n - (back + 1);      // complete? keep : hold back
    }
    return n;
  }

  int    _fds[WS_MAX_CLIENTS] = { -1, -1, -1 };
  char   _buf[WS_SINK_BUF];
  size_t _len      = 0;
  bool   _dropping = false;
};

inline WsSink sink;

// ── Per-socket line accumulator ─────────────────────────────────────────────
// ONE PER SOCKET, never a shared static. A single function-level buffer fuses two
// clients' bytes into one garbage line whenever their frames interleave — and that
// is not a rare race, because a long line legitimately spans several frames, so a
// second client's command landing in that window destroys both. NaviCore reproduced
// exactly this on hardware (two valid commands, zero replies). Eviction policy is
// deliberately identical to WsSink::begin() so the two cannot drift.
// It is also the SESSION: whether it has logged in, and the [AUTH:...] lines it is owed.
// A session is its `serial`, never its fd: lwIP hands a closed socket's number to the next
// connection (the LRU purge does exactly that under load), so anything keyed on the fd alone
// - a queued command, a mirror subscription - would pass to whoever connects next.
struct WsAcc {
  int      fd  = -1;               // authMux (pump() reads it on the loop task)
  uint32_t serial = 0;             // this session, unique since boot (authMux)
  uint32_t openedMs = 0;           // when it opened: not logged in after WS_LOGIN_MS = closed
  size_t   len = 0;
  bool     overrun = false;
  bool     authed  = false;        // may run commands and see the mirror (written under authMux)
  bool     closing = false;        // [AUTH:FAIL] owed or sent, or its login timed out (pumpNotes): everything
                                   // else from it is dropped (authMux)
  uint8_t  notes   = 0;            // NOTE_* owed, sent by pump() (authMux)
  char     buf[WS_LINE_MAX];
};
inline WsAcc    accs[WS_MAX_CLIENTS];
inline uint32_t accSerial = 0;     // httpd task, under authMux: the last serial handed out

inline WsAcc* accFor(int fd) {
  // A session that is not in the table yet is a new one: logged in only if there is no
  // token to log in with (read before the lock - tokenCopy takes the same one).
  char tok[WS_TOKEN_MAX + 1];
  tokenCopy(tok);
  WsAcc* a = nullptr;
  int evicted = -1;
  portENTER_CRITICAL(&authMux);
  for (int i = 0; i < WS_MAX_CLIENTS && !a; i++) if (accs[i].fd == fd) a = &accs[i];
  if (!a) {
    for (int i = 0; i < WS_MAX_CLIENTS && !a; i++) if (accs[i].fd < 0) a = &accs[i];
    // Full (max_open_sockets matches, so only a session httpd has not closed yet): evict one
    // that has not logged in before one that has.
    for (int i = 0; i < WS_MAX_CLIENTS && !a; i++) if (!accs[i].authed) a = &accs[i];
    if (!a) a = &accs[0];
    if (a->fd >= 0) evicted = a->fd;
    a->fd = fd; a->len = 0; a->overrun = false;
    if (++accSerial == 0) accSerial = 1;
    a->serial = accSerial; a->openedMs = millis();
    a->authed = !tok[0]; a->closing = false; a->notes = 0;
  }
  portEXIT_CRITICAL(&authMux);
  if (evicted >= 0) sink.drop(evicted);   // its record is gone: it is no longer known to be logged in
  return a;
}
inline void releaseAcc(int fd) {
  portENTER_CRITICAL(&authMux);
  for (int i = 0; i < WS_MAX_CLIENTS; i++)
    if (accs[i].fd == fd) {
      accs[i].fd = -1; accs[i].serial = 0; accs[i].len = 0; accs[i].overrun = false;
      accs[i].authed = false; accs[i].closing = false; accs[i].notes = 0;
    }
  portEXIT_CRITICAL(&authMux);
}
inline void setAuthed(WsAcc* a, bool on) {
  portENTER_CRITICAL(&authMux);
  a->authed = on;
  portEXIT_CRITICAL(&authMux);
}
// The session a queued command came from, if it is still that session and logged in (loop
// task, drain). The socket's fd alone proves nothing: it may be a new connection's now.
inline bool sessionLive(int fd, uint32_t serial) {
  bool live = false;
  portENTER_CRITICAL(&authMux);
  for (int i = 0; i < WS_MAX_CLIENTS; i++)
    if (accs[i].fd == fd && accs[i].serial == serial && accs[i].authed && !accs[i].closing) live = true;
  portEXIT_CRITICAL(&authMux);
  return live;
}
inline void note(WsAcc* a, uint8_t bits) {
  portENTER_CRITICAL(&authMux);
  a->notes |= bits;
  portEXIT_CRITICAL(&authMux);
}
// "?AUTH,<given>" from session `a` (httpd task).
inline void authLine(WsAcc* a, const char* given) {
  char tok[WS_TOKEN_MAX + 1];
  tokenCopy(tok);
  if (!tok[0]) {                   // no token: always OK, so a client can always send it
    setAuthed(a, true);
    sink.begin(a->fd);
    note(a, NOTE_OK);
    return;
  }
  const uint32_t now    = millis();
  const bool     locked = authFailMs && (now - authFailMs) < AUTH_LOCKOUT_MS;
  const bool     match  = tokenEqual(given, tok);   // compared even when locked: the same work either way
  if (match && !locked) {
    setAuthed(a, true);
    sink.begin(a->fd);
    note(a, NOTE_OK);
    return;
  }
  authFailMs = now ? now : 1;      // any failure locks every session out for AUTH_LOCKOUT_MS
  setAuthed(a, false);
  sink.drop(a->fd);
  portENTER_CRITICAL(&authMux);
  a->notes  |= NOTE_FAIL;          // pump() sends it, then closes the session
  a->closing = true;
  portEXIT_CRITICAL(&authMux);
}

// Feed one received byte into a client's accumulator; enqueue on newline.
inline void feed(WsAcc* a, int fd, char c) {
  if (c == '\n' || c == '\r') {
    if (a->overrun)  { a->overrun = false; a->len = 0; return; }   // drop the WHOLE line
    if (!a->len)     return;                                       // blank line
    a->buf[a->len] = '\0';
    const size_t n = a->len;
    a->len = 0;
    if (a->closing) return;                                        // failed its login: closing
    // The login, before anything can run. ?AUTH is never queued: it means nothing to
    // the relay itself, and on the USB port it is a no-op (relaySerialLine ignores it).
    if (const char* given = authArg(a->buf)) { authLine(a, given); return; }
    if (!a->authed) { note(a, NOTE_REQUIRED); return; }            // dropped unrun
    WsCmd cmd;
    cmd.fd     = fd;
    cmd.serial = a->serial;                                        // drain() runs it only for this session
    memcpy(cmd.line, a->buf, n + 1);
    if (!wsQueue) return;
    // Backpressure, not silent loss — see WS_ENQUEUE_WAIT_MS.
    if (xQueueSend(wsQueue, &cmd, pdMS_TO_TICKS(WS_ENQUEUE_WAIT_MS)) != pdTRUE)
      Serial.println("[relay] ws: command queue full - dropped");
    return;
  }
  if (a->overrun) return;                                          // swallow to end-of-line
  if (a->len < WS_LINE_MAX - 1) { a->buf[a->len++] = c; return; }
  // Overrun. Suppress to end-of-line rather than restarting accumulation: zeroing
  // len mid-line would treat the TAIL as a fresh command, and an unprefixed tail
  // falls through relaySerialLine() to wcb.broadcast() — putting an arbitrary
  // mid-line fragment on EVERY board on the mesh. Same reasoning as inOverrun in
  // the .ino's USB reader; the two paths must fail identically.
  a->overrun = true;
  a->len     = 0;
  Serial.println("[relay] ws: line too long - dropped");
}

// ── httpd handler (Core 0 — copy, enqueue, return) ──────────────────────────
inline esp_err_t wsHandler(httpd_req_t* req) {
  const int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {          // handshake completed: a NEW session
    releaseAcc(fd);                       // nothing carries over from an earlier socket on this fd
    WsAcc* a = accFor(fd);
    if (a->authed) sink.begin(fd);        // no token: exactly as before
    else           note(a, NOTE_REQUIRED);
    return ESP_OK;
  }

  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  // Two-call pattern: first with max_len 0 to learn the length, then read.
  esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
  if (err != ESP_OK) return err;
  if (frame.type == HTTPD_WS_TYPE_CLOSE) { releaseAcc(fd); sink.drop(fd); return ESP_OK; }
  if (frame.len == 0) return ESP_OK;
  // Cap a single frame at four lines' worth. A client that sends more than that in
  // one frame cannot produce a valid command anyway, and this keeps the buffer
  // below bounded. SAY SO rather than returning quietly: a silently discarded frame
  // leaves the client waiting forever for a reply to a command we never read, which
  // is the same failure the enqueue timeout exists to avoid.
  if (frame.len > WS_LINE_MAX * 4) {
    Serial.printf("[relay] ws: frame of %u B exceeds %u B - dropped\n",
                  (unsigned)frame.len, (unsigned)(WS_LINE_MAX * 4));
    return ESP_OK;
  }

  // static, not a stack array: 1.5 KB would blow the httpd task stack. Safe because
  // esp_http_server services every socket from ONE task, so two handler invocations
  // can never overlap.
  static uint8_t buf[WS_LINE_MAX * 4];
  frame.payload = buf;
  err = httpd_ws_recv_frame(req, &frame, sizeof(buf));
  if (err != ESP_OK) return err;
  if (frame.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;

  // Re-arm the sink on traffic: a client that was evicted while another connected
  // would otherwise stay silent. Only a logged-in session: no mirror before the login.
  WsAcc* a = accFor(fd);
  if (a->closing) return ESP_OK;          // failed its login: nothing more is read from it
  if (a->authed) sink.begin(fd);
  for (size_t i = 0; i < frame.len; i++) feed(a, fd, (char)buf[i]);
  return ESP_OK;
}

// Session close: release the accumulator and the sink slot. Without this a departed
// client holds both until some later send happens to fail on it, and the unowned
// tail of a half-accumulated line eats the first command of the NEXT client.
inline void wsClose(httpd_handle_t /*hd*/, int sockfd) {
  releaseAcc(sockfd);
  sink.drop(sockfd);
  close(sockfd);          // httpd hands ownership of the fd to this hook
}

// ── Lifecycle ───────────────────────────────────────────────────────────────
// Self-contained: on any failure it says so and leaves everything inert, so a
// WebSocket that will not start can never take the relay down with it.
inline bool begin() {
  wsQueue = xQueueCreate(WS_QUEUE_DEPTH, sizeof(WsCmd));
  if (!wsQueue) { Serial.println("[relay] ws: queue alloc FAILED - WebSocket disabled"); return false; }

  httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
  cfg.max_open_sockets = WS_MAX_CLIENTS;   // keep in step with WS_MAX_CLIENTS above
  cfg.close_fn         = wsClose;
  cfg.lru_purge_enable = true;
  if (httpd_start(&wsServer, &cfg) != ESP_OK) {
    Serial.println("[relay] ws: httpd_start FAILED - WebSocket disabled");
    vQueueDelete(wsQueue); wsQueue = nullptr; wsServer = nullptr;
    return false;
  }

  httpd_uri_t uri  = {};
  uri.uri          = "/ws";
  uri.method       = HTTP_GET;
  uri.handler      = wsHandler;
  uri.user_ctx     = nullptr;
  uri.is_websocket = true;
  if (httpd_register_uri_handler(wsServer, &uri) != ESP_OK) {
    Serial.println("[relay] ws: URI register FAILED - WebSocket disabled");
    httpd_stop(wsServer); wsServer = nullptr;
    vQueueDelete(wsQueue); wsQueue = nullptr;
    return false;
  }
  return true;
}

// Run at most ONE queued command per loop() pass, on Core 1. One at a time is the
// honest ceiling: relaySerialLine() may block on ETM retries, and running a burst
// back-to-back would stall wcb.update() (heartbeats, offline detection, WDP).
inline void drain() {
  if (!wsQueue) return;
  WsCmd cmd;
  if (xQueueReceive(wsQueue, &cmd, 0) != pdTRUE) return;
  // Only a logged-in session's lines are queued, but the session may have ended since -
  // closed, failed a later ?AUTH, or gone with its fd already handed to a new, signed-out
  // connection. Its queued lines die with it (this is the purge on close), and nothing but
  // the session that sent a line is ever put on the mirror for it: the mirror carries config
  // pulls, the mesh password and this relay's own replies.
  if (!sessionLive(cmd.fd, cmd.serial)) return;
  sink.begin(cmd.fd);            // ensure the reply reaches the client that asked
  relaySerialLine(cmd.line);
}

// One line to one session, outside the mirror: the [AUTH:...] replies.
inline void sendTo(int fd, const char* line) {
  httpd_ws_frame_t f = {};
  f.final   = true;
  f.type    = HTTPD_WS_TYPE_TEXT;
  f.payload = (uint8_t*)line;
  f.len     = strlen(line);
  httpd_ws_send_frame_async(wsServer, fd, &f);
}

// The [AUTH:...] lines each session is owed, then the mirror. A [AUTH:FAIL] is sent
// and then the session is closed: httpd_sess_trigger_close() queues the close on the
// httpd task, behind the frame this task has just written. A session that has not logged
// in within WS_LOGIN_MS is closed too, so a silent socket cannot hold a slot.
inline void pumpNotes() {
  if (!wsServer) return;
  const uint32_t now = millis();
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    int     fd;
    uint8_t n;
    bool    stale;
    portENTER_CRITICAL(&authMux);
    fd = accs[i].fd;
    n  = accs[i].notes;
    accs[i].notes = 0;
    stale = fd >= 0 && !accs[i].authed && !accs[i].closing && now - accs[i].openedMs >= WS_LOGIN_MS;
    if (stale) accs[i].closing = true;
    portEXIT_CRITICAL(&authMux);
    if (fd < 0) continue;
    if (n & NOTE_REQUIRED) sendTo(fd, "[AUTH:REQUIRED]\n");
    if (n & NOTE_OK)       sendTo(fd, "[AUTH:OK]\n");
    if (n & NOTE_FAIL)   { sendTo(fd, "[AUTH:FAIL]\n"); httpd_sess_trigger_close(wsServer, fd); }
    else if (stale)        httpd_sess_trigger_close(wsServer, fd);
  }
}

inline void pump() { pumpNotes(); sink.pump(); }

}  // namespace mgmtws
