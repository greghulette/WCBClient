#include "WCB_Client.h"
#include "WCBStream.h"
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <Preferences.h>   // learned-peer persistence (NVS)

// Singleton instance pointer — allows the static ESP-NOW callback to route
// received packets back to the active WCB_Client object. Only one WCB_Client
// instance is supported per sketch.
WCB_Client* WCB_Client::_instance = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
//
// Stores all network credentials and optional callbacks into private members.
// No hardware is touched here — ESP-NOW and WiFi are not initialised until
// begin() is called. This makes it safe to declare a WCB_Client at global scope
// before the Arduino runtime has started.
// ─────────────────────────────────────────────────────────────────────────────
WCB_Client::WCB_Client(uint8_t mac_oct2, uint8_t mac_oct3,
                     const char* password, uint8_t wcb_quantity, uint8_t device_id,
                     WCBCommandCallback commandCb, WCBStatusCallback statusCb) {
    _oct2     = mac_oct2;
    _oct3     = mac_oct3;
    _quantity = wcb_quantity;
    _deviceID = device_id;
    strncpy(_password, password, sizeof(_password) - 1);
    _password[sizeof(_password) - 1] = '\0';
    _commandCallback = commandCb;
    _statusCallback  = statusCb;
    // Set singleton immediately so WCBStream objects declared at global scope
    // (after this WCB_Client) can self-register without needing a reference passed in.
    _instance = this;
}

// ─────────────────────────────────────────────────────────────────────────────
// begin
//
// Validates configuration, sets the custom WiFi MAC address, initialises
// ESP-NOW, registers all WCB peers, and arms the heartbeat timer.
//
// Must be called once from setup() after Serial.begin().
// Returns true on success, false if validation or ESP-NOW init fails.
//
// Why we set a custom MAC:
//   Every WCB derives every peer's MAC from the shared oct2/oct3 and their ID.
//   If this device uses a random factory MAC the WCBs won't have it registered
//   and will silently drop all packets from it. Setting the MAC to the expected
//   scheme (02:oct2:oct3:00:00:ID) makes this device look identical to a real
//   WCB at the hardware level.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::begin() {

    // ── Validate device_id ───────────────────────────────────────────────────
    // device_id must be 1–WCB_MAX_BOARDS (hard-fail otherwise). A value ABOVE
    // wcb_quantity (and not the special slot 20) is ALLOWED but non-standard: the
    // floor boards (1..quantity) haven't pre-registered this MAC, so the device is
    // reachable INBOUND only once they auto-join it from its WDP advert — call
    // setIdentity() and keep auto-join on. begin() only WARNS in that case.
    if (_deviceID == 0 || _deviceID > WCB_MAX_BOARDS) {
        Serial.printf("[WCB_Client] ERROR: device_id %d is out of range (1–%d)\n",
                      _deviceID, WCB_MAX_BOARDS);
        return false;
    }
    // NON-FATAL: an id above wcb_quantity (and not the special slot 20) is allowed.
    // Index safety is already guaranteed by the out-of-range hard-fail above, and
    // every board array is sized WCB_MAX_BOARDS. The only consequence is inbound
    // reachability, which auto-join resolves once the floor boards hear this
    // device's WDP advert (setIdentity()).
    if (_deviceID != WCB_SPECIAL_ID && _deviceID > _quantity) {
        Serial.printf("[WCB_Client] WARNING: device_id %d is above wcb_quantity %d - "
                      "allowed, but the floor boards can't reach it until they auto-join "
                      "it from its WDP advert; call setIdentity() and keep auto-join on.\n",
                      _deviceID, _quantity);
    }

    // ── Reset internal state ─────────────────────────────────────────────────
    _seqCounter      = 0;
    _bootAnnouncesLeft  = 3;                       // see _sendBootAnnounce()
    _nextBootAnnounceMs = 0;                       // first one goes out on the next update()
    _nextHeartbeatMs = 0;                          // Triggers heartbeat immediately on
                                                   // first update() call
    memset(_boards,  0, sizeof(_boards));
    memset(_pending, 0, sizeof(_pending));
    _csReset();                                    // channel sync: no move, no search; boot grace from now

    // ── WiFi setup ───────────────────────────────────────────────────────────
    // ESP-NOW requires WiFi to have station mode enabled. esp_now_init() fails
    // outright if the WiFi driver is not running, so verify the mode change
    // took before we get there — that failure is the usual root cause of a
    // crash on the first heartbeat.
    //
    // If the sketch has already brought up a SoftAP (e.g. hosting a web UI),
    // preserve it — AP and STA share a single radio channel on the ESP32, so
    // ESP-NOW simply rides whatever channel the AP is already on instead of
    // forcing STA-only and tearing the AP down. WiFi.disconnect() only drops
    // a STA association (default wifioff=false), so it's already AP-safe on
    // its own.
    wifi_mode_t priorMode = WiFi.getMode();
    bool apActive = (priorMode == WIFI_MODE_AP || priorMode == WIFI_MODE_APSTA);
    wifi_mode_t targetMode = apActive ? WIFI_AP_STA : WIFI_STA;
    if (!WiFi.mode(targetMode)) {
        Serial.printf("[WCB_Client] WARNING: WiFi.mode(%s) reported failure "
                      "— ESP-NOW init will likely fail below\n",
                      apActive ? "WIFI_AP_STA" : "WIFI_STA");
    }
    WiFi.disconnect();

    if (apActive) {
        // Modem sleep is far more likely to bite once the AP is actively
        // serving clients — a missed ESP-NOW heartbeat here reads as this
        // device going offline to the rest of the mesh.
        esp_wifi_set_ps(WIFI_PS_NONE);
        Serial.println("[WCB_Client] SoftAP detected — running WIFI_AP_STA, "
                       "ESP-NOW sharing the AP's radio channel");
    }

    // ── Region (channel sync, 1.19.0) ────────────────────────────────────────
    // After WiFi is up (the country call needs the driver) and BEFORE the channel
    // is set: under the default "01" a channel above 11 is refused. "01" makes no
    // call (see _csApplyRegion), so a sketch that never calls setRegion() runs as
    // before. Then the region's real top channel decides what _meshChannel may be -
    // a persisted 12 under a region that no longer allows it falls back to the
    // default rather than leave the radio wherever the failed set left it.
    _csApplyRegion();
    _csMaxCh = _csReadMaxCh();
    if (!chansync::validChannel(_meshChannel, _csMaxCh)) {
        Serial.printf("[WCB_Client] WARNING: mesh channel %u is outside region \"%s\" (channels 1-%u) - "
                      "using channel %u\n", _meshChannel, _csRegion, _csMaxCh, WCB_MESH_CHANNEL);
        _meshChannel = WCB_MESH_CHANNEL;
    }

    // ── Mesh channel ─────────────────────────────────────────────────────────
    // ESP-NOW peers are channel-0 ("current channel"), so this device is only
    // heard if its radio sits on the mesh channel (_meshChannel, settable via
    // setMeshChannel()). With no SoftAP we pin it deterministically; with a
    // SoftAP active the AP owns the channel, so we can't move it without yanking
    // the AP — we only WARN on a mismatch, and re-check in update() to catch a
    // SoftAP that's brought up after begin().
    if (!apActive) {
        esp_err_t chErr = esp_wifi_set_channel(_meshChannel, WIFI_SECOND_CHAN_NONE);
        if (chErr != ESP_OK) {
            Serial.printf("[WCB_Client] WARNING: esp_wifi_set_channel(%u) failed (err %d) - "
                          "the radio may be on the wrong channel; region \"%s\" allows 1-%u.\n",
                          _meshChannel, chErr, _csRegion, _csMaxCh);
        }
    } else {
        _checkMeshChannel();
    }

    // ── Build MAC table ──────────────────────────────────────────────────────
    // Must happen BEFORE esp_wifi_set_mac() so _wcbMACs[] is populated.
    _buildMACs();

    // ── Set custom MAC address ───────────────────────────────────────────────
    // Set this ESP32's WiFi MAC to the WCB network scheme so other WCBs
    // recognise it as a valid peer. Without this, every packet sent by this
    // device arrives from an unknown MAC and is dropped.
    if (esp_wifi_set_mac(WIFI_IF_STA, _wcbMACs[_deviceID - 1]) != ESP_OK) {
        // Non-fatal — some ESP32 variants restrict MAC changes. Packets may
        // still be received, but WCBs will not accept incoming packets from us.
        Serial.printf("[WCB_Client] WARNING: could not set custom MAC for device ID %d\n",
                      _deviceID);
    } else {
        Serial.printf("[WCB_Client] MAC set to %02X:%02X:%02X:%02X:%02X:%02X\n",
                      _wcbMACs[_deviceID-1][0], _wcbMACs[_deviceID-1][1],
                      _wcbMACs[_deviceID-1][2], _wcbMACs[_deviceID-1][3],
                      _wcbMACs[_deviceID-1][4], _wcbMACs[_deviceID-1][5]);
    }

    // ── Initialise ESP-NOW ───────────────────────────────────────────────────
    // If this fails, _started stays false and update() becomes a safe no-op
    // (see the guard there) instead of crashing on the first heartbeat. Print
    // the exact error name so the cause is obvious from the boot log.
    esp_err_t enErr = esp_now_init();
    if (enErr != ESP_OK) {
        Serial.printf("[WCB_Client] ERROR: esp_now_init() failed: %s (0x%x). "
                      "Device will NOT join the mesh; check that WiFi started.\n",
                      esp_err_to_name(enErr), enErr);
        return false;
    }

    // Store the instance pointer and register the receive callback.
    // The callback is a static function; it uses _instance to call the
    // non-static _handleReceive() on the correct object.
    _instance = this;
    esp_now_register_recv_cb(_espNowReceiveCB);

    // ── Register peers ───────────────────────────────────────────────────────
    // Pre-register every WCB MAC and the broadcast MAC as ESP-NOW peers.
    // ESP-NOW requires a peer to be registered before you can send to it.
    _registerPeers();

    // Restore auto-joined peers persisted by a previous session — they are
    // permanent members until the app forgets them (forgetPeer/clearLearnedPeers).
    _loadLearnedPeers();
    _csLoadNeverMiss();          // channel sync: ids a search never misses (after the fingerprint check above)
    _started = true;             // enableSpecialPeer() after begin() registers immediately

    Serial.printf("[WCB_Client] Joined WCB network as device ID %d "
                  "(quantity=%d, oct2=0x%02X, oct3=0x%02X)\n",
                  _deviceID, _quantity, _oct2, _oct3);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// update
//
// Call every iteration of loop(). Performs two tasks:
//
//   1. Heartbeat  — fires a broadcast HEARTBEAT packet at the configured
//                   interval so all WCBs know this device is still alive.
//                   WCBs that stop seeing heartbeats will mark this device
//                   offline and may stop routing messages to it.
//
//   2. Offline detection — scans the board status table and marks any WCB
//                          offline if its last heartbeat is older than
//                          (heartbeatIntervalSec * missedBeforeOffline) seconds.
//                          Fires the status callback when a transition occurs.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::update() {
    // If begin() never succeeded (e.g. esp_now_init() failed), ESP-NOW is not
    // initialised. Calling esp_now_send() in that state dereferences a NULL
    // driver global and hard-crashes the chip (Guru Meditation LoadProhibited)
    // — an S3 with a failed init would boot-loop here. Do nothing until begin()
    // has actually brought ESP-NOW up.
    if (!_started) return;

    unsigned long now = millis();

    // Channel sync first (1.19.0). It owns the radio while a search is away from the
    // home channel, and nothing else may go out then - not a heartbeat, an advert, a
    // retry or a watchdog probe: on the channel it is visiting, any of them would
    // announce this device to a group it is not part of. All of it resumes here once
    // the search is home again (about 1.5-2 s), a few seconds late at most.
    _csTick(now);
    if (_csOffHome) return;

    // Boot announces first: three, ~1200 ms apart, matching the firmware. Driven from here
    // rather than begin() so joining is never blocked by them.
    if (_bootAnnouncesLeft > 0 && now >= _nextBootAnnounceMs) {
        _sendBootAnnounce();
        _bootAnnouncesLeft--;
        _nextBootAnnounceMs = now + 1200;
    }
    if (now >= _nextHeartbeatMs) {
        _sendHeartbeat();
        _nextHeartbeatMs = now + (_heartbeatIntervalSec * 1000UL);
        _checkMeshChannel();   // catch a SoftAP that moved the radio off the mesh channel
    }
    _txWatchTick(now);         // prove the mesh still hears us (setTxWatchdog; off by default)

    // Abandon a fragment session whose sender stopped mid-command, so a lost
    // tail cannot hold the buffer against the next one. Cheap scalar test, and
    // it runs here on the app task rather than on the receive callback.
    if (_mgmtSession != 0xFFFF && (long)(now - _mgmtDeadline) >= 0) {
        _mgmtSession = 0xFFFF; _mgmtMask = 0; _mgmtTotal = 0; _mgmtLastLen = 0;
    }

    _checkOfflineBoards();
    _processMonitors();
    _processFragJob();   // drain a pending fragmented send, one chunk per tick
    _bulkDrain(now);     // process inbound bulk-transfer frames (storage callbacks fire HERE)
    _bulkTick(now);      // bulk session timeout + completed-session cache expiry
    _wdpTick();          // WDP device-identity advert cadence (boot burst + periodic)
    _ageNeighbors(now);  // expire WDP neighbors we haven't heard from recently
    _seqTick(now);       // deliver/abandon a pending sequence-name request (callback fires HERE)

    // Drain deferred auto-joins on the LOOP task. _handleWdpAdvert (which flags
    // these) runs in the ESP-NOW receive callback (WiFi task), where
    // _addLearnedPeer()'s esp_now_add_peer + NVS flash write would overflow the
    // small callback stack and crash. Doing them here makes them safe.
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (_pendingJoin[i]) { _pendingJoin[i] = false; _addLearnedPeer((uint8_t)(i + 1)); }
        // A peer that (now) advertises "temporary" must not remain a PERMANENT learned
        // member — downgrade it here on the loop task (forgetPeer does esp_now_del_peer +
        // an NVS write, unsafe on the RX callback). Covers a device that flips to temporary
        // and a relay left in NVS from before it advertised the flag. forgetPeer clears
        // _learnedPeer, so this is idempotent; the !nb.temporary auto-join gate keeps it out.
        else if (_neighbors[i].valid && _neighbors[i].temporary && _learnedPeer[i])
            forgetPeer((uint8_t)(i + 1));
    }

    // Register transient reply-peers flagged on the RX callback (an authenticated
    // above-floor sender we couldn't unicast an ACK/reply to yet). Same discipline
    // as auto-join above — the bool is set on Core 0, the esp_now_add_peer runs
    // HERE on the loop task — but with NO NVS write: this is not a learned member,
    // just enough of a peer to answer whoever just talked to us. Auto-join still
    // persists genuine members separately. Lands within ~1 loop, so the sender's
    // next command (the config tool re-pings 6x/500ms on connect) gets its reply.
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (!_pendingReplyPeer[i]) continue;
        _pendingReplyPeer[i] = false;
        if (esp_now_is_peer_exist(_wcbMACs[i])) continue;
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[i], 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    // Service the pending table. _pending[] is mutated cross-core (the ACK handler
    // and a Core-0 _sendPacket claim BOTH take _pendingMux), so every read/write of
    // a slot here runs UNDER the lock too — otherwise this Core-1 loop would tear a
    // slot the RX task is concurrently claiming/acking. The blocking _transmit()
    // (esp_now_send) must NOT run inside the spinlock, so we collect the boards to
    // retransmit + a snapshot of the command/seq under the lock and transmit AFTER
    // releasing it (a stale retransmit is harmless — receivers dedup by seq).
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        uint8_t  retryBoards[WCB_MAX_BOARDS];
        int      nRetry  = 0;
        char     cmdCopy[sizeof(_pending[0].command)];
        uint16_t seqCopy = 0;

        portENTER_CRITICAL(&_pendingMux);
        WCBPending& p = _pending[i];
        if (p.active) {
            if (p.ensured) {
                // ── Ensured delivery: PER-BOARD UNICAST retries ──────────────
                // Mirrors the WCB firmware's processETMAcksAndRetries(): retry as a
                // UNICAST to each expected board that hasn't ACK'd (reusing the seq
                // so it's deduped), up to ETM_MAX_RETRIES per board.
                if (_ensuredComplete(p)) {            // every expected board acked / gone
                    // STATS: this is the DOMINANT real-world failure path, not an
                    // edge case. _checkOfflineBoards() ran at the top of update()
                    // and the offline window is 50 s against a ~2 s retry budget,
                    // so a target that has been powered off for any length of time
                    // is already !online here — the slot completes with
                    // retryCount == 0 and never reaches the give-up branch below.
                    _settleSlot(p);
                    p.active = false;
                } else if ((now - p.sentMs) >= ETM_RETRY_INTERVAL_MS) {
                    for (int b = 0; b < WCB_MAX_BOARDS; b++) {
                        if (!p.expected[b] || p.ackReceived[b]) continue;
                        if (!_boards[b].online) {
                            // Dropped after being online (lastSeenMs!=0) -> give up.
                            // Never-online learned peer (lastSeenMs==0, advert-only
                            // client) -> keep retrying up to ETM_MAX_RETRIES.
                            // STATS: credit the give-up HERE — clearing expected[b]
                            // destroys the evidence, so the _settleSlot() sweeps
                            // below would find nothing left to count for it.
                            if (_boards[b].lastSeenMs != 0) { _statFail((uint8_t)b); p.expected[b] = false; continue; }
                        }
                        if (p.retryCount[b] < ETM_MAX_RETRIES) {
                            retryBoards[nRetry++] = (uint8_t)(b + 1);   // _transmit after unlock
                            p.retryCount[b]++;
                            _stats[b].retries++;      // STATS: the only site with per-board
                                                      // attribution — the _transmit() at the
                                                      // bottom of the loop runs after the
                                                      // unlock and has none left.
                        } else {
                            _statFail((uint8_t)b);    // STATS: same reasoning as above
                            p.expected[b] = false;    // exhausted retries for this board
                        }
                    }
                    p.sentMs = now;                   // reset the retry window
                    // KNOWN PESSIMISM, for a NEVER-ONLINE peer only (lastSeenMs
                    // == 0 — an advert-only client that never heartbeats): the
                    // branch above may have just queued this board's FINAL retry
                    // and pushed retryCount[b] to ETM_MAX_RETRIES, which makes
                    // _ensuredComplete() true right here. So the slot is settled
                    // — crediting `failed` — in the same pass whose _transmit()
                    // runs a few lines below, after the unlock. That attempt
                    // gets no ACK window at all, and if the peer does answer,
                    // the ACK finds no active slot.
                    //
                    // Left alone deliberately: the slot was ALREADY freed at
                    // this point before counters existed, so the delivery
                    // behaviour is unchanged and the counter is merely reporting
                    // it. Holding the slot one more pass to give that attempt a
                    // real window would be a delivery change, and belongs in its
                    // own commit. For an ONLINE peer this cannot happen —
                    // _ensuredComplete() keeps the slot outstanding while the
                    // board is online, regardless of retryCount.
                    if (_ensuredComplete(p)) { _settleSlot(p); p.active = false; }
                    if (nRetry) { memcpy(cmdCopy, p.command, sizeof(cmdCopy)); seqCopy = p.seqNum; }
                }
            } else {
                // Best-effort unicast: sent ONCE, no retransmit — free a slot whose
                // ACK never arrived so the table can't leak (1 s >> ETM round-trip).
                // STATS: an un-ACKd best-effort unicast is still a delivery we gave
                // up on; _settleSlot credits it via the targetID rule in
                // _statExpects (expected[] is empty for best-effort slots).
                if ((now - p.sentMs) > 1000UL) { _settleSlot(p); p.active = false; }
            }
        }
        portEXIT_CRITICAL(&_pendingMux);

        for (int k = 0; k < nRetry; k++) _transmit(retryBoards[k], cmdCopy, seqCopy);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// send
//
// Send a text command to one specific WCB.
// Internally calls _sendPacket() which handles CRC appending and ACK tracking.
// Commands longer than one packet are fragmented automatically (1.3.0) —
// previously they were silently truncated by strncpy, so a long ?SEQ,SAVE
// arrived cut short and a corrupt sequence got stored on the board.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::send(uint8_t target_wcb, const char* command, bool ensured) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (target_wcb < 1 || target_wcb > WCB_MAX_BOARDS) return false;
    if (!command) return false;
    _csNoteBusy();                 // channel sync: a search waits for the traffic to stop
    if (strlen(command) > _maxSingleCommandLen()) {
        return _sendFragmented(target_wcb, command, ensured);
    }
    return _sendPacket(target_wcb, command, ensured);
}

// ─────────────────────────────────────────────────────────────────────────────
// broadcast
//
// Send a text command to ALL WCBs simultaneously via the shared broadcast MAC.
// Every WCB on the network receives and processes the same packet.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::broadcast(const char* command, bool ensured) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (!command) return false;
    // Fragmentation is unicast-only (the target-side reassembly session is
    // per-board) — an oversized broadcast must fail LOUDLY, never truncate.
    if (strlen(command) > _maxSingleCommandLen()) {
        Serial.printf("[WCB_Client] broadcast: command too long (%u > %u chars) — "
                      "fragmentation is unicast-only. send() it to each board instead.\n",
                      (unsigned)strlen(command), (unsigned)_maxSingleCommandLen());
        return false;
    }
    // A JSON broadcast is best-effort BY PROTOCOL: the WCB deliberately does not ACK a target-0
    // '{' frame (it is consumed by the telemetry relay, never run as a command), so an ensured
    // one can never complete - it times out and retries as unicast, and every board then
    // processes the payload a second time. Force best-effort rather than let ensured=true, the
    // default, quietly mean 'deliver twice'.
    if (command[0] == '{') ensured = false;
    // Channel sync waits for quiet, but not for best-effort broadcasts: NaviCore streams
    // rc_hb/rc_ch telemetry at 5 Hz, and counting it would mean it never searches. A
    // second of telemetry lost to a search is the cost of a missing board coming back.
    if (ensured) _csNoteBusy();
    return _sendPacket(WCB_TARGET_BROADCAST, command, ensured);
}

// ─────────────────────────────────────────────────────────────────────────────
// _maxSingleCommandLen
//
// Longest command that fits ONE packet. Derived from the actual struct field
// (not magic numbers) so this threshold can never drift from what _sendPacket
// physically packs: structCommand holds N-1 chars + NUL; when checksum is
// enabled the "|CRC%08X" suffix (12 chars) shares the same field.
// ─────────────────────────────────────────────────────────────────────────────
static const size_t kCmdFieldChars  = sizeof(((wcb_packet_etm_t*)0)->structCommand) - 1; // 199
static const size_t kCrcSuffixChars = 12;  // strlen("|CRC") + 8 hex digits

size_t WCB_Client::_maxSingleCommandLen() const {
    return _checksumEnabled ? (kCmdFieldChars - kCrcSuffixChars) : kCmdFieldChars;
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendFragmented
//
// Deliver an oversized UNICAST command via the WCB firmware's MGMT
// fragmentation protocol (the same one the web config tool uses):
//   • split into WCB_MGMT_CHUNK_LEN-char chunks (max WCB_MGMT_MAX_CHUNKS)
//   • broadcast each chunk as a wcb_packet_mgmt_t (226 bytes — the firmware
//     dispatches by size); only the addressed targetWCB reassembles it
//   • when all chunks arrive, the target executes the WHOLE command through
//     its normal parser (chained ^ commands split correctly)
//
// Reliability: the MGMT layer has NO per-chunk ACK, so the full chunk set is
// transmitted in multiple passes (3 when ensured, 2 otherwise). Duplicates
// are harmless — the target stores each chunk once and remembers completed
// sessionIds, so repeat passes after completion are silently discarded.
//
// NON-BLOCKING (1.3.1): this only QUEUES the job — chunks are transmitted one
// at a time from update() with ~10 ms pacing, so it is safe to call from
// loop() or even the ESP-NOW receive callback (the old implementation ran
// delay(10) loops in the caller, which could stall the WiFi task for ~0.5 s).
// The target's 15 s reassembly window comfortably covers the spread passes.
//
// Returns true = queued. The transmission result (every chunk accepted by
// ESP-NOW at least once across the passes, or not) is logged on completion.
// NOTE: acceptance is not delivery — there is no ACK at this layer.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_sendFragmented(uint8_t target_wcb, const char* command, bool ensured) {
    size_t len = strlen(command);

    // Bound-check on LENGTH before any narrow cast: computing a uint8_t chunk
    // count first wraps at 256 chunks (~45.8 KB) and would bypass this guard —
    // silently truncating or sending nothing.
    if (len > WCB_MGMT_MAX_COMMAND_LEN) {
        Serial.printf("[WCB_Client] send: command too long even for fragmentation "
                      "(%u chars > %u max). Not sent.\n",
                      (unsigned)len, (unsigned)WCB_MGMT_MAX_COMMAND_LEN);
        return false;
    }
    if (_fragJob.active) {
        Serial.println("[WCB_Client] send: a fragmented send is already in flight — "
                       "wait for it to finish (max ~0.5 s) and retry.");
        return false;
    }

    char* copy = (char*)malloc(len + 1);
    if (!copy) {
        Serial.println("[WCB_Client] send: out of memory for fragmented command.");
        return false;
    }
    memcpy(copy, command, len + 1);

    // Fresh sessionId per command. 0xFFFF is reserved (the firmware's
    // completed-session ring buffer initialises to 0xFFFF, which would make
    // the very first session look like a duplicate and be discarded).
    uint16_t sessionId = (uint16_t)esp_random();
    if (sessionId == 0xFFFF) sessionId = 0xFFFE;

    _fragJob.targetWCB    = target_wcb;
    _fragJob.sessionId    = sessionId;
    _fragJob.totalChunks  = (uint8_t)((len + WCB_MGMT_CHUNK_LEN - 1) / WCB_MGMT_CHUNK_LEN);
    _fragJob.passes       = ensured ? 3 : 2;
    _fragJob.pass         = 0;
    _fragJob.chunk        = 0;
    _fragJob.acceptedMask = 0;
    _fragJob.nextSendMs   = 0;       // first chunk goes out on the next update()
    _fragJob.cmd          = copy;
    _fragJob.len          = len;
    _fragJob.active       = true;    // set LAST — update() may run on the other core

    Serial.printf("[WCB_Client] send: %u chars > single-packet limit — fragmenting "
                  "to WCB%d (%d chunks, session %04X, %d passes, via update())\n",
                  (unsigned)len, target_wcb, _fragJob.totalChunks, sessionId,
                  _fragJob.passes);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// _processFragJob — transmit the next pending fragment. Called from update().
// One chunk per call, ~10 ms apart: a maximal ensured job (16 chunks × 3
// passes) completes in ~0.5 s of wall time without ever blocking loop().
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_processFragJob() {
    if (!_fragJob.active) return;
    unsigned long now = millis();
    if (now < _fragJob.nextSendMs) return;

    wcb_packet_mgmt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    pkt.packetType  = WCB_MGMT_PACKET_TYPE_FRAG_UNICAST;  // unicast semantics on 6.1.1+
    pkt.targetWCB   = _fragJob.targetWCB;
    pkt.sessionId   = _fragJob.sessionId;
    pkt.chunkIdx    = _fragJob.chunk;
    pkt.totalChunks = _fragJob.totalChunks;

    size_t off = (size_t)_fragJob.chunk * WCB_MGMT_CHUNK_LEN;
    size_t n   = _fragJob.len - off;
    if (n > WCB_MGMT_CHUNK_LEN) n = WCB_MGMT_CHUNK_LEN;
    memcpy(pkt.payload, _fragJob.cmd + off, n);   // payload[180] zeroed above → NUL-safe

    if (_espSend(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK) {
        _fragJob.acceptedMask |= (uint16_t)(1u << _fragJob.chunk);
    }
    _fragJob.nextSendMs = now + 10;   // pacing: don't flood the ESP-NOW TX queue

    // Advance chunk → pass → done.
    if (++_fragJob.chunk >= _fragJob.totalChunks) {
        _fragJob.chunk = 0;
        if (++_fragJob.pass >= _fragJob.passes) {
            // Success = every chunk accepted by ESP-NOW at least once across
            // the passes (per-chunk, NOT per-pass: the target dedups chunks,
            // so a union of clean chunks completes the session).
            uint16_t fullMask = (uint16_t)((1u << _fragJob.totalChunks) - 1);
            if (_fragJob.acceptedMask == fullMask) {
                Serial.printf("[WCB_Client] fragmented send to WCB%d complete "
                              "(session %04X, %d chunks x %d passes)\n",
                              _fragJob.targetWCB, _fragJob.sessionId,
                              _fragJob.totalChunks, _fragJob.passes);
            } else {
                Serial.printf("[WCB_Client] fragmented send to WCB%d INCOMPLETE — "
                              "chunk mask %04X of %04X accepted; the command may "
                              "not have executed\n",
                              _fragJob.targetWCB, _fragJob.acceptedMask, fullMask);
            }
            free(_fragJob.cmd);
            _fragJob.cmd    = nullptr;
            _fragJob.active = false;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// sendRaw
//
// Unicast raw binary data to one specific WCB for forwarding to its serial port.
//
// How it works:
//   The target ID in the packet is set to WCB_TARGET_RAW_SERIAL (97).
//   When the receiving WCB sees this target ID it writes the raw bytes directly
//   to the specified serial port. No text parsing, no CRC — byte-for-byte.
//
// Use case — Pololu / Maestro (known target):
//   You know exactly which WCB has the servo controller wired to which port.
//   Build the binary command (e.g. a Maestro set-target packet) and call
//   sendRaw() to deliver it wirelessly as if this ESP32 were wired directly.
//
// Requirements:
//   - The target WCB just needs the servo controller wired to target_port.
//   - No KYBER,REMOTE config is required on the receiving WCB.
//   - data length is capped at 177 bytes (firmware raw-chunk limit).
//   - No CRC is added — the data is treated as opaque binary.
//
// See also: sendKyber() for broadcasting to ALL WCBs with Maestros at once.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::sendRaw(uint8_t target_wcb, uint8_t target_port,
                        const uint8_t* data, size_t len) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (target_wcb < 1 || target_wcb > WCB_MAX_BOARDS) return false;
    if (target_port < 1 || target_port > 5) return false;
    if (len == 0) return false;

    // WCB firmware caps raw chunks at 177 bytes — clip silently
    if (len > 177) len = 177;
    _csNoteBusy();

    // Raw serial packets MUST use the non-ETM struct (wcb_packet_t, 249 bytes).
    // The WCB firmware routes incoming packets by size: 252 bytes → ETM path,
    // 249 bytes → standard path. The raw serial handler (targetID=97) only
    // exists in the standard path — ETM-sized raw packets are dropped.
    // This matches exactly how WCB firmware sends raw serial via sendESPNowRawSerial().
    wcb_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);

    // Target ID 97 tells the receiving WCB to forward raw bytes to a serial port
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "%d", WCB_TARGET_RAW_SERIAL);

    pkt.structCommandIncluded = 1;

    // WCB firmware expects a 3-byte header before the data:
    //   [0]   target_port  — which serial port on the WCB (1–5)
    //   [1]   len low byte — little-endian uint16 length
    //   [2]   len high byte
    //   [3..] data bytes
    pkt.structCommand[0] = target_port;
    pkt.structCommand[1] = (uint8_t)(len & 0xFF);
    pkt.structCommand[2] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(pkt.structCommand + 3, data, len);

    return _espSend(_wcbMACs[target_wcb - 1],
                        (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// sendKyber
//
// Broadcast raw binary data to ALL WCBs on the network via the Kyber path.
//
// How it works:
//   The packet uses WCB_TARGET_KYBER (98) with the NON-ETM packet struct
//   (wcb_packet_t / 249 bytes) — see the long note in the function body for
//   why the ETM struct does not work here.
//   Any WCB that has Kyber_Remote enabled receives the broadcast and writes the
//   bytes to its locally wired Maestro serial port(s). WCBs without Kyber_Remote
//   ignore it. This mirrors exactly what the WCB firmware does when a physical
//   Kyber device is connected — no targeted addressing needed.
//
// Header format inside structCommand:
//   [0]   len low byte   — little-endian uint16 length of the data
//   [1]   len high byte
//   [2..] data bytes
// sendKyber() builds this header automatically.
//
// Use case — Maestro broadcast (multiple WCBs with Maestros):
//   You want the same Maestro command to reach every board with a servo
//   controller, or you don't know which WCB has the Maestro. One broadcast
//   packet reaches all of them in one ESP-NOW send.
//
// data : pointer to the binary byte array (Maestro/Pololu command bytes)
// len  : number of bytes (max 178 — 180 structCommand bytes minus 2-byte header)
// Returns true if ESP-NOW accepted the packet for transmission.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::sendKyber(const uint8_t* data, size_t len) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (len == 0) return false;
    // Firmware caps Kyber chunks at 178 bytes (structCommand[180] - 2-byte header)
    if (len > 178) len = 178;
    _csNoteBusy();

    // Kyber broadcast MUST use the non-ETM struct (wcb_packet_t, 249 bytes) —
    // NOT the ETM struct.  The WCB firmware routes incoming ESP-NOW packets by
    // size: 252-byte packets go down the ETM path, 249-byte packets down the
    // standard path.  The Kyber-broadcast handler (targetID==98) only exists in
    // the *standard* path, and the standard path has an explicit Kyber bypass
    // so a 249-byte targetID-98 frame falls straight through to it and is
    // written to every WCB's local Maestro port(s) — true one-send broadcast.
    //
    // An ETM-sized (252-byte) Kyber packet never reaches that handler: the ETM
    // path treats it as a PACKET_TYPE_COMMAND and drops it at the target gate
    // (targetID 98 is neither 0/broadcast nor this board's ID) before any Kyber
    // processing.  That made Kyber broadcast silently fail on every ETM-enabled
    // WCB (the default).  Using wcb_packet_t here is what the WCB firmware and
    // this library's own WCB_TARGET_KYBER header doc actually expect.
    wcb_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "%d", WCB_TARGET_KYBER);
    pkt.structCommandIncluded = 1;

    // 2-byte little-endian length header, then data
    pkt.structCommand[0] = (uint8_t)(len & 0xFF);
    pkt.structCommand[1] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(pkt.structCommand + 2, data, len);

    return _espSend(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// monitorRaw
//
// Registers a UART to watch for raw binary output from an attached device
// (e.g. a Pololu/Maestro servo controller driven by another library writing
// to a serial port on this same ESP32).
//
// Wiring: connect the source UART's TX pin to an unused UART's RX pin on
// this ESP32. Pass the receiving UART here. The library reads bytes as they
// arrive, buffers them, and sends via sendRaw() when the inter-frame gap
// (gap_ms of silence) signals a packet boundary.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::monitorRaw(HardwareSerial& port, uint8_t target_wcb,
                           uint8_t target_port, uint16_t gap_ms) {
    _monitorRawPort   = &port;
    _monitorRawTarget = target_wcb;
    _monitorRawTPort  = target_port;
    _monitorRawGapMs  = gap_ms;
    _monitorRawLen    = 0;
    _monitorRawLastMs = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// monitorSerial
//
// Registers a UART to watch for text command lines output by an attached
// device. Each line terminated by 'terminator' is forwarded to target_wcb
// via send() (or broadcast() if target_wcb == WCB_TARGET_BROADCAST).
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::monitorSerial(HardwareSerial& port, uint8_t target_wcb, char terminator) {
    _monitorSerialPort   = &port;
    _monitorSerialTarget = target_wcb;
    _monitorSerialTerm   = terminator;
    _monitorSerialLen    = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// _processMonitors
//
// Called by update() every loop iteration. Polls both serial monitors.
//
// Raw monitor:
//   Reads all available bytes into the buffer. When no new bytes have arrived
//   for _monitorRawGapMs milliseconds the buffer is considered a complete
//   packet and flushed via sendRaw(). The gap threshold is the key: Pololu
//   Mini SSC packets are 3 bytes; at 9600 baud that's ~0.3ms per packet.
//   A 2ms gap reliably falls between packets, not mid-packet.
//
// Text monitor:
//   Reads characters one by one. When the terminator is received the
//   accumulated string is dispatched via send() or broadcast() and the
//   buffer resets. Lines longer than the single-packet limit (199 chars,
//   187 with checksum) are truncated to that limit before dispatch.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_processMonitors() {
    unsigned long now = millis();

    // ── WCBStream flush ───────────────────────────────────────────────────────
    // Tick all registered WCBStream instances (one per Maestro controller).
    // tick() checks the inter-frame gap and calls sendRaw() if a complete
    // packet has accumulated since the last write().
    for (uint8_t i = 0; i < _wcbStreamCount; i++)
        _wcbStreams[i]->tick();

    // ── Raw binary monitor ────────────────────────────────────────────────────
    if (_monitorRawPort) {
        // Drain all available bytes into the buffer
        while (_monitorRawPort->available() && _monitorRawLen < sizeof(_monitorRawBuf)) {
            _monitorRawBuf[_monitorRawLen++] = (uint8_t)_monitorRawPort->read();
            _monitorRawLastMs = now;
        }
        // Flush when we have data and the inter-frame gap has elapsed.
        // target == 0 means Kyber broadcast to all WCBs; otherwise unicast.
        if (_monitorRawLen > 0 && (now - _monitorRawLastMs) >= _monitorRawGapMs) {
            if (_monitorRawTarget == 0)
                sendKyber(_monitorRawBuf, _monitorRawLen);
            else
                sendRaw(_monitorRawTarget, _monitorRawTPort, _monitorRawBuf, _monitorRawLen);
            _monitorRawLen = 0;
        }
    }

    // ── Text serial monitor ───────────────────────────────────────────────────
    if (_monitorSerialPort) {
        while (_monitorSerialPort->available()) {
            char c = (char)_monitorSerialPort->read();
            if (c == _monitorSerialTerm) {
                // Terminator received — dispatch whatever is in the buffer.
                // CLAMP to the single-packet limit first: monitor lines are
                // streaming spam, so an over-limit line is truncated (and
                // delivered) rather than dropped by broadcast()'s oversize
                // rejection or queued through the fragmented path — both of
                // which are wrong for high-rate monitor traffic.
                if (_monitorSerialLen > 0) {
                    size_t maxLen = _maxSingleCommandLen();
                    if (_monitorSerialLen > maxLen) _monitorSerialLen = maxLen;
                    _monitorSerialBuf[_monitorSerialLen] = '\0';
                    if (_monitorSerialTarget == WCB_TARGET_BROADCAST)
                        broadcast(_monitorSerialBuf);
                    else
                        send(_monitorSerialTarget, _monitorSerialBuf);
                    _monitorSerialLen = 0;
                }
            } else if (_monitorSerialLen < sizeof(_monitorSerialBuf) - 1) {
                _monitorSerialBuf[_monitorSerialLen++] = c;
            }
            // Bytes past the buffer limit are silently dropped
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// isOnline
//
// Returns true if the specified WCB has sent a heartbeat recently enough to
// be considered online. Useful for guarding send() calls so you don't fire
// commands at a board that has dropped off the network.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::isOnline(uint8_t wcb_number) {
    if (wcb_number < 1 || wcb_number > WCB_MAX_BOARDS) return false;
    return _boards[wcb_number - 1].online;
}

// ─────────────────────────────────────────────────────────────────────────────
// enableSpecialPeer / isSpecialPeerOnline / sendToSpecialPeer
//
// The "special peer" (NaviCore) is an out-of-band device (default ID 20) that
// lives outside the 1..wcb_quantity range. Receiving heartbeats/commands/ACKs
// from it already works; these helpers add the two things that DON'T work until
// the peer is explicitly enabled: (1) registering its MAC so we can SEND to it,
// and (2) including it in offline detection so its ETM status is tracked.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::enableSpecialPeer(uint8_t id) {
    if (id < 1 || id > WCB_MAX_BOARDS) {
        Serial.printf("[WCB_Client] enableSpecialPeer: id %d out of range (1–%d)\n",
                      id, WCB_MAX_BOARDS);
        return;
    }
    _specialPeerID = id;
    if (_started) _registerSpecialPeer();   // begin() already ran — register now
    Serial.printf("[WCB_Client] Special peer enabled at ID %d\n", id);
}

bool WCB_Client::isSpecialPeerOnline() {
    if (_specialPeerID < 1 || _specialPeerID > WCB_MAX_BOARDS) return false;
    return _boards[_specialPeerID - 1].online;
}

bool WCB_Client::sendToSpecialPeer(const char* command, bool ensured) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (_specialPeerID < 1) {
        Serial.println("[WCB_Client] sendToSpecialPeer: special peer not enabled "
                       "(call enableSpecialPeer() first)");
        return false;
    }
    return send(_specialPeerID, command, ensured);
}

// ─────────────────────────────────────────────────────────────────────────────
// onCommand / onStatusChange / setChecksum
// ─────────────────────────────────────────────────────────────────────────────

void WCB_Client::onCommand(WCBCommandCallback callback) {
    _commandCallback = callback;
}

void WCB_Client::onStatusChange(WCBStatusCallback callback) {
    _statusCallback = callback;
}

void WCB_Client::onRawPacket(WCBRawPacketCallback callback) {
    _rawPacketCallback = callback;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Bulk transfer + inbound COMMAND de-dup
//  ---------------------------------------------------------------------------
//  The RX task only PARSES and ENQUEUES (no session state, no flash). ALL session
//  state, the storage callbacks, and every wire reply happen on the LOOP task in
//  _bulkDrain / _bulkTick (from update()). So Core 0 is never blocked and the
//  consumer's callbacks may safely do flash I/O.  Wire (tool→RC, each ≤187 B):
//    BEGIN {"bb":sid,"n":,"t":,"g":"tag","h":}   CHUNK {"bc":sid,"q":,"s":"b64"}
//    DONE  {"bd":sid,"r":round}
//  Back-channel (RC→sender): STATUS {"bs":sid,"got":,"r":,"miss":[..]},
//    need-BEGIN {"bs":sid,"nb":1},  FINAL {"bs":sid,"done":1,"ok":,"hash":,"r":}
// ═════════════════════════════════════════════════════════════════════════════

void WCB_Client::onBulkBegin(WCBBulkBeginCallback callback)       { _bulkBeginCb    = callback; }
void WCB_Client::onBulkChunk(WCBBulkChunkCallback callback)       { _bulkChunkCb    = callback; }
void WCB_Client::onBulkComplete(WCBBulkCompleteCallback callback) { _bulkCompleteCb = callback; }

// ── COMMAND anti-replay window (RX task, lock-free: _boards seq fields are only
// touched here). Returns true if `seq` from `senderID` is a duplicate to drop. ──
bool WCB_Client::_cmdSeqDup(uint8_t senderID, uint16_t seq) {
    WCBBoardStatus& b = _boards[senderID - 1];
    if (!b.haveCmdSeq) { b.haveCmdSeq = true; b.cmdSeqHigh = seq; b.cmdSeqMask = 0; return false; }
    int16_t d = (int16_t)(seq - b.cmdSeqHigh);
    if (d == 0) return true;                     // exact current high → dup
    if (d > 0) {                                 // newer → advance the window
        if (d >= 32) b.cmdSeqMask = 0;
        else         b.cmdSeqMask = (b.cmdSeqMask << d) | (1u << (d - 1));  // old high now "seen"
        b.cmdSeqHigh = seq;
        return false;
    }
    uint16_t back = (uint16_t)(-d);              // older than high
    if (back <= 32) {
        uint32_t bit = 1u << (back - 1);
        if (b.cmdSeqMask & bit) return true;     // already seen within the window
        b.cmdSeqMask |= bit;
        return false;
    }
    return false;                                // too old to remember → accept
}

// ── Tiny helpers for the flat, machine-generated bulk envelopes (no ArduinoJson
// dependency; base64 values contain no quotes/colons so key-search is safe). ──
static bool _bulkJsonUint(const char* s, const char* key, uint32_t& out) {
    char pat[12];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return false;
    const char* p = strstr(s, pat);
    if (!p) return false;
    p += n;
    while (*p == ' ') p++;
    if (*p < '0' || *p > '9') return false;
    uint32_t v = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10u + (uint32_t)(*p - '0'); p++; }
    out = v;
    return true;
}
static size_t _bulkJsonStr(const char* s, const char* key, char* out, size_t outMax) {
    if (outMax == 0) return 0;
    char pat[12];
    int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return 0;
    const char* p = strstr(s, pat);
    if (!p) return 0;
    p += n;
    while (*p == ' ') p++;
    if (*p != '"') return 0;
    p++;
    size_t o = 0;
    while (*p && *p != '"' && o < outMax - 1) out[o++] = *p++;
    if (*p != '"') return 0;   // unterminated
    out[o] = '\0';
    return o;
}
static size_t _bulkB64Decode(const char* in, size_t inLen, uint8_t* out, size_t outMax) {
    auto val = [](char c) -> int {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 'a' + 26;
        if (c >= '0' && c <= '9') return c - '0' + 52;
        if (c == '+') return 62;
        if (c == '/') return 63;
        return -1;
    };
    if (inLen == 0 || (inLen % 4) != 0) return 0;
    size_t o = 0;
    for (size_t i = 0; i < inLen; i += 4) {
        char c2 = in[i + 2], c3 = in[i + 3];
        int a = val(in[i]), b = val(in[i + 1]);
        int c = (c2 == '=') ? 0 : val(c2);
        int d = (c3 == '=') ? 0 : val(c3);
        if (a < 0 || b < 0 || c < 0 || d < 0) return 0;
        uint32_t trip = ((uint32_t)a << 18) | ((uint32_t)b << 12) | ((uint32_t)c << 6) | (uint32_t)d;
        if (o < outMax)                 out[o++] = (trip >> 16) & 0xFF;
        if (c2 != '=' && o < outMax)    out[o++] = (trip >> 8)  & 0xFF;
        if (c3 != '=' && o < outMax)    out[o++] =  trip        & 0xFF;
    }
    return o;
}

// ── RX task: recognise + parse a bulk envelope, enqueue to the loop task ──────
bool WCB_Client::_maybeRouteBulk(uint8_t senderID, const char* cmd) {
    uint8_t type;
    if      (strncmp(cmd, "{\"bb\":", 6) == 0) type = 0;   // BEGIN
    else if (strncmp(cmd, "{\"bc\":", 6) == 0) type = 1;   // CHUNK
    else if (strncmp(cmd, "{\"bd\":", 6) == 0) type = 2;   // DONE
    else return false;                                     // not bulk → fall through to onCommand

    WCBBulkMsg m = {};
    m.type = type;
    m.senderID = senderID;

    if (type == 0) {
        uint32_t sid, nn, tt, hh;
        if (!_bulkJsonUint(cmd, "bb", sid) || !_bulkJsonUint(cmd, "n", nn) ||
            !_bulkJsonUint(cmd, "t", tt)   || !_bulkJsonUint(cmd, "h", hh)) return true;  // malformed → drop
        m.sid = (uint16_t)sid; m.total = (uint16_t)nn; m.totalLen = tt; m.hash = hh;
        _bulkJsonStr(cmd, "g", m.tag, sizeof(m.tag));
    } else if (type == 1) {
        uint32_t sid, q;
        char b64[176];
        if (!_bulkJsonUint(cmd, "bc", sid) || !_bulkJsonUint(cmd, "q", q)) return true;
        size_t bl = _bulkJsonStr(cmd, "s", b64, sizeof(b64));
        if (bl == 0) return true;
        size_t dl = _bulkB64Decode(b64, bl, m.data, sizeof(m.data));
        if (dl == 0) return true;
        m.sid = (uint16_t)sid; m.q = (uint16_t)q; m.len = (uint16_t)dl;
    } else {
        uint32_t sid, r = 0;
        if (!_bulkJsonUint(cmd, "bd", sid)) return true;
        _bulkJsonUint(cmd, "r", r);
        m.sid = (uint16_t)sid; m.round = (uint16_t)r;
    }
    _bulkRingPush(m);   // full → dropped; the missing chunk reappears in the next STATUS
    return true;
}

// ── SPSC ring: RX task pushes, loop task pops (guarded so a slot never tears) ──
bool WCB_Client::_bulkRingPush(const WCBBulkMsg& m) {
    bool ok = false;
    portENTER_CRITICAL(&_bulkMux);
    if ((uint16_t)(_bulkRingHead - _bulkRingTail) < WCB_BULK_RING_DEPTH) {
        _bulkRing[_bulkRingHead % WCB_BULK_RING_DEPTH] = m;
        _bulkRingHead++;
        ok = true;
    }
    portEXIT_CRITICAL(&_bulkMux);
    return ok;
}
bool WCB_Client::_bulkRingPop(WCBBulkMsg& out) {
    bool ok = false;
    portENTER_CRITICAL(&_bulkMux);
    if (_bulkRingHead != _bulkRingTail) {
        out = _bulkRing[_bulkRingTail % WCB_BULK_RING_DEPTH];
        _bulkRingTail++;
        ok = true;
    }
    portEXIT_CRITICAL(&_bulkMux);
    return ok;
}

void WCB_Client::_bulkDrain(unsigned long now) {
    WCBBulkMsg m;
    int budget = WCB_BULK_RING_DEPTH * 2;   // bound work per tick
    while (budget-- > 0 && _bulkRingPop(m)) {
        if      (m.type == 0) _bulkHandleBegin(m, now);
        else if (m.type == 1) _bulkHandleChunk(m, now);
        else                  _bulkHandleDone(m, now);
    }
}

void WCB_Client::_bulkHandleBegin(const WCBBulkMsg& m, unsigned long now) {
    // Size validation: 1<=n<=MAX and (n-1)*RAW < t <= n*RAW.
    bool valid = m.total >= 1 && m.total <= WCB_BULK_MAX_CHUNKS &&
                 m.totalLen >  (uint32_t)(m.total - 1) * WCB_BULK_CHUNK_RAW &&
                 m.totalLen <= (uint32_t)m.total       * WCB_BULK_CHUNK_RAW;
    if (!valid) { _bulkSendFinal(m.senderID, m.sid, 0, false, 0); return; }

    // Already completed (a re-BEGIN after a lost FINAL)? Re-answer, don't restart.
    WCBBulkDone* d = _bulkFindDone(m.senderID, m.sid);
    if (d) { _bulkSendFinal(m.senderID, m.sid, 0, d->ok, d->hash); return; }

    if (_bulkRx.active) {
        if (_bulkRx.senderID == m.senderID && _bulkRx.sid == m.sid) {
            if (_bulkRx.total == m.total && _bulkRx.hash == m.hash) {
                // Idempotent duplicate BEGIN — don't wipe progress; re-report it.
                _bulkRx.lastMs = now;
                _bulkSendStatus(m.senderID, m.sid, _bulkRx.round);
                return;
            }
            // Same sid, different payload → restart (fall through to re-open).
        } else {
            _bulkSendFinal(m.senderID, m.sid, 0, false, 0);   // single-flight: busy
            return;
        }
    }

    // Arm the sink. Consumer opens + pre-extends its tmp; false = reject.
    if (!_bulkChunkCb || !_bulkCompleteCb ||
        !_bulkBeginCb(m.senderID, m.sid, m.totalLen, m.total, m.hash, m.tag)) {
        _bulkSendFinal(m.senderID, m.sid, 0, false, 0);
        _bulkRx.active = false;
        return;
    }
    _bulkRx.active   = true;
    _bulkRx.senderID = m.senderID;
    _bulkRx.sid      = m.sid;
    _bulkRx.total    = m.total;
    _bulkRx.got      = 0;
    _bulkRx.round    = 0;
    _bulkRx.totalLen = m.totalLen;
    _bulkRx.hash     = m.hash;
    _bulkRx.lastMs   = now;
    memset(_bulkRx.mask, 0, sizeof(_bulkRx.mask));
}

void WCB_Client::_bulkHandleChunk(const WCBBulkMsg& m, unsigned long now) {
    if (!_bulkRx.active || _bulkRx.senderID != m.senderID || _bulkRx.sid != m.sid) {
        if (!_bulkFindDone(m.senderID, m.sid))      // already done → ignore
            _bulkSendNeedBegin(m.senderID, m.sid, now);
        return;
    }
    _bulkRx.lastMs = now;                            // keep-alive on ANY frame
    if (m.q >= _bulkRx.total) return;                // out of range
    uint16_t byteIdx = m.q >> 3;
    uint8_t  bit     = (uint8_t)(1u << (m.q & 7));
    if (_bulkRx.mask[byteIdx] & bit) return;         // duplicate chunk → idempotent skip
    // New chunk. Offset is derived from q (fixed size), never trusted from the wire.
    _bulkChunkCb(m.sid, (uint32_t)m.q * WCB_BULK_CHUNK_RAW, m.data, m.len);
    _bulkRx.mask[byteIdx] |= bit;
    _bulkRx.got++;
    if (_bulkRx.got >= _bulkRx.total) {              // all distinct chunks in (popcount==n)
        bool ok = _bulkCompleteCb(m.sid, true, _bulkRx.totalLen, _bulkRx.hash);
        _bulkCacheDone(m.senderID, m.sid, ok, _bulkRx.hash, now);
        _bulkSendFinal(m.senderID, m.sid, _bulkRx.round, ok, _bulkRx.hash);
        _bulkRx.active = false;
    }
}

void WCB_Client::_bulkHandleDone(const WCBBulkMsg& m, unsigned long now) {
    if (_bulkRx.active && _bulkRx.senderID == m.senderID && _bulkRx.sid == m.sid) {
        _bulkRx.round  = m.round;
        _bulkRx.lastMs = now;                        // keep-alive
        _bulkSendStatus(m.senderID, m.sid, m.round); // report the gaps
        return;
    }
    WCBBulkDone* d = _bulkFindDone(m.senderID, m.sid);
    if (d) { _bulkSendFinal(m.senderID, m.sid, m.round, d->ok, d->hash); return; }
    _bulkSendNeedBegin(m.senderID, m.sid, now);
}

void WCB_Client::_bulkTick(unsigned long now) {
    if (_bulkRx.active && (now - _bulkRx.lastMs) > WCB_BULK_TIMEOUT_MS) {
        if (_bulkCompleteCb) _bulkCompleteCb(_bulkRx.sid, false, _bulkRx.totalLen, _bulkRx.hash);
        _bulkSendFinal(_bulkRx.senderID, _bulkRx.sid, _bulkRx.round, false, 0);
        _bulkRx.active = false;
    }
    for (int i = 0; i < WCB_BULK_DONECACHE; i++)
        if (_bulkDone[i].used && (now - _bulkDone[i].ts) > 30000UL)
            _bulkDone[i].used = false;
}

// STATUS: first WCB_BULK_MISS_MAX unset chunk indices, scanned from 0 so the
// window advances as leading gaps fill (sender treats miss[] as advisory).
void WCB_Client::_bulkSendStatus(uint8_t senderID, uint16_t sid, uint16_t round) {
    char buf[200];
    int len = snprintf(buf, sizeof(buf), "{\"bs\":%u,\"got\":%u,\"r\":%u,\"miss\":[",
                       (unsigned)sid, (unsigned)_bulkRx.got, (unsigned)round);
    if (len < 0 || len >= (int)sizeof(buf)) return;
    int nMiss = 0;
    for (uint16_t q = 0; q < _bulkRx.total && nMiss < WCB_BULK_MISS_MAX; q++) {
        if (_bulkRx.mask[q >> 3] & (1u << (q & 7))) continue;
        int w = snprintf(buf + len, sizeof(buf) - len, nMiss ? ",%u" : "%u", (unsigned)q);
        if (w <= 0 || len + w > 180) break;          // keep buf (+CRC) under the 187 B relay cap
        len += w;
        nMiss++;
    }
    snprintf(buf + len, sizeof(buf) - len, "]}");
    send(senderID, buf);
}

void WCB_Client::_bulkSendNeedBegin(uint8_t senderID, uint16_t sid, unsigned long now) {
    if (now - _bulkNbLastMs < 400UL) return;         // rate-limit the NAK
    _bulkNbLastMs = now;
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"bs\":%u,\"nb\":1}", (unsigned)sid);
    send(senderID, buf);
}

void WCB_Client::_bulkSendFinal(uint8_t senderID, uint16_t sid, uint16_t round, bool ok, uint32_t hash) {
    char buf[80];
    snprintf(buf, sizeof(buf), "{\"bs\":%u,\"done\":1,\"ok\":%d,\"hash\":%u,\"r\":%u}",
             (unsigned)sid, ok ? 1 : 0, (unsigned)hash, (unsigned)round);
    send(senderID, buf);
}

void WCB_Client::_bulkCacheDone(uint8_t senderID, uint16_t sid, bool ok, uint32_t hash, unsigned long now) {
    WCBBulkDone* slot = _bulkFindDone(senderID, sid);
    if (!slot) {
        slot = &_bulkDone[0];
        for (int i = 1; i < WCB_BULK_DONECACHE; i++) {
            if (!_bulkDone[i].used) { slot = &_bulkDone[i]; break; }
            if (_bulkDone[i].ts < slot->ts) slot = &_bulkDone[i];
        }
    }
    slot->used = true; slot->senderID = senderID; slot->sid = sid;
    slot->ok = ok; slot->hash = hash; slot->ts = now;
}

WCB_Client::WCBBulkDone* WCB_Client::_bulkFindDone(uint8_t senderID, uint16_t sid) {
    for (int i = 0; i < WCB_BULK_DONECACHE; i++)
        if (_bulkDone[i].used && _bulkDone[i].senderID == senderID && _bulkDone[i].sid == sid)
            return &_bulkDone[i];
    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// onNeighbor / getNeighbor / neighborCount — WDP consumer API
//
// A neighbor is any board on the mesh whose WDP advert we've decoded: another
// WCB, or a WCB_Client device (isClient). Adverts arrive roughly every 30s and
// age out after WCB_WDP_NEIGHBOR_TTL_MS of silence.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::onNeighbor(WCBNeighborCallback callback) {
    _neighborCallback = callback;
}

const WCBNeighbor* WCB_Client::getNeighbor(uint8_t wcbNumber) {
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS) return nullptr;
    const WCBNeighbor& nb = _neighbors[wcbNumber - 1];
    return nb.valid ? &nb : nullptr;
}

uint8_t WCB_Client::neighborCount() {
    uint8_t n = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++)
        if (_neighbors[i].valid) n++;
    return n;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stored-sequence inventory  (requestSequenceNames / onSequenceNames)
//
// A pull, not a push: the names are NOT in the WDP advert, because ~16 bytes per
// name would evict the port labels from the advert's fixed 200-byte payload. What
// WDP carries instead is a 4-byte fingerprint (WCBNeighbor::seqHash) that tells a
// consumer WHEN to call this.
// ─────────────────────────────────────────────────────────────────────────────

// The firmware routes ESP-NOW by packet SIZE before it ever looks at packetType
// (CONFIG_REQ and MGMT_FRAG_UNICAST are both type 5, told apart only by 43 vs 226
// bytes). If a compiler ever pads these, the packets would route into the wrong
// handler — or be dropped with no diagnostic at either end. Fail the build instead.
static_assert(sizeof(wcb_packet_seq_req_t)  == 43,
              "wcb_packet_seq_req_t must match the firmware's espnow_struct_config_req (43 B)");
static_assert(sizeof(wcb_packet_seq_frag_t) == 230,
              "wcb_packet_seq_frag_t must match the firmware's espnow_struct_config_frag (230 B)");
static_assert(sizeof(wcb_packet_seqval_req_t) == 59,
              "wcb_packet_seqval_req_t must match the firmware's espnow_struct_seqval_req (59 B)");
static_assert(sizeof(wcb_packet_seqpage_req_t) == seqn::PAGE_REQ_LEN &&
              offsetof(wcb_packet_seqpage_req_t, page) == seqn::PAGE_REQ_PAGE_AT,
              "wcb_packet_seqpage_req_t must match the firmware's espnow_struct_seqpage_req (59 B, page at 43)");
static_assert(WCB_PACKET_SEQ_REQ == seqn::PT_SEQ_REQ && WCB_PACKET_SEQ_FRAG == seqn::PT_SEQ_FRAG &&
              WCB_PACKET_SEQ_PAGE_REQ == seqn::PT_PAGE_REQ && WCB_PACKET_SEQ_PAGE == seqn::PT_PAGE,
              "WCB_SeqNamesWire.h packet types != WCB_Client.h's");

#define WCB_SEQ_TIMEOUT_MS 4000   // generous: 3x REQ + up to 16 frags at 20 ms apart (= seqn::WAIT_FIRST_MS)

void WCB_Client::onSequenceNames(WCBSequenceNamesCallback callback) {
    _seqNamesCallback = callback;
}

void WCB_Client::onSequenceValue(WCBSequenceValueCallback callback) {
    _seqValueCallback = callback;
}

// Shared key validation for every sequence call. A comma is rejected because the
// firmware takes a stored key as everything BEFORE the first comma — a key with one
// would be silently truncated on save and then never match on read.
static bool wcbSeqKeyValid(const char* key) {
    if (!key) return false;
    size_t n = strlen(key);
    if (n == 0 || n > WCB_SEQ_KEY_MAX) return false;
    return strchr(key, ',') == nullptr;
}

// Detach the reassembly buffer from the RX task, then free it. The detach must be
// inside the lock so _maybeHandleSeqFrag can't be mid-memcpy when the free lands;
// the free itself is deliberately OUTSIDE it (heap work in a critical section
// blocks the other core).
void WCB_Client::_seqReset() {
    portENTER_CRITICAL(&_seqMux);
    char* doomed = _seqChunks;
    _seqChunks   = nullptr;
    _seqPending  = false;
    portEXIT_CRITICAL(&_seqMux);
    if (doomed) free(doomed);
    if (_seqNames) free(_seqNames);
    _seqNames        = nullptr;
    _seqNamesCap     = 0;
    _seqAsm.reset();
    _seqWantFrag     = 0;
    _seqReqKey[0]    = '\0';
    _seqTargetWCB    = 0;
    _seqSessionId    = 0;
    _seqTotalChunks  = 0;
    _seqReceivedMask = 0;
    _seqFragType     = 0;
    _seqDoneSession  = 0;
    _seqTries        = 0;
    _seqRestarts     = 0;
    _seqHeardMs      = 0;
    _seqComplete     = false;
}

// Ask `_seqTargetWCB` for one page of its names (WCB_SeqNamesWire.h): SEQ_PAGE_REQ x3, and for
// the first request of a pull the old SEQ_REQ x3 behind it, for a WCB that predates paged
// names (it drops the 59-byte 28 at its receive filter and answers a 13). A WCB with paged
// names answers the first 28 and takes the 13s for copies. Broadcast 3x — a first frame to a
// given board is routinely dropped on a busy mesh, and the target dedups repeats of one
// (requester, page) inside 1.5 s, so this is still one reply. Broadcast (not unicast) so this
// works without the target being a peer yet.
void WCB_Client::_seqSendNames(unsigned page, bool withOld) {
    wcb_packet_seqpage_req_t pr;
    memset(&pr, 0, sizeof(pr));
    strncpy(pr.structPassword, _password, sizeof(pr.structPassword) - 1);
    pr.packetType   = WCB_PACKET_SEQ_PAGE_REQ;
    pr.targetWCB    = _seqTargetWCB;
    pr.requesterWCB = (uint8_t)_deviceID;
    pr.page         = (uint8_t)page;
    for (int i = 0; i < 3; i++) {
        _espSend(_broadcastMAC, (const uint8_t*)&pr, sizeof(pr));
        delay(15);
    }
    if (withOld) {
        wcb_packet_seq_req_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
        pkt.packetType   = WCB_PACKET_SEQ_REQ;
        pkt.targetWCB    = _seqTargetWCB;
        pkt.requesterWCB = (uint8_t)_deviceID;
        for (int i = 0; i < 3; i++) {
            _espSend(_broadcastMAC, (const uint8_t*)&pkt, sizeof(pkt));
            delay(15);
        }
    }
    _seqAskedMs = millis();
}

// Loop task: clear the reassembly slot for the next page, keeping its buffer. The RX task
// writes nothing while _seqComplete is set, so the slot is ours until that is cleared here.
// `doneSession` is the page just taken: the rest of its second pass is dropped.
void WCB_Client::_seqRearm(uint16_t doneSession) {
    portENTER_CRITICAL(&_seqMux);
    if (_seqChunks) memset(_seqChunks, 0, (size_t)WCB_SEQ_MAX_CHUNKS * (WCB_SEQ_PAYLOAD_SIZE + 1));
    _seqSessionId    = 0;
    _seqTotalChunks  = 0;
    _seqReceivedMask = 0;
    _seqFragType     = 0;
    _seqDoneSession  = doneSession;
    _seqComplete     = false;
    portEXIT_CRITICAL(&_seqMux);
}

bool WCB_Client::requestSequenceNames(uint8_t wcbNumber) {
    if (!_started)                                    return false;
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS)  return false;
    if (_seqPending)                                  return false;   // one at a time
    if (!_seqNamesCallback)                           return false;   // nowhere to deliver

    const size_t bufSize = (size_t)WCB_SEQ_MAX_CHUNKS * (WCB_SEQ_PAYLOAD_SIZE + 1);
    _seqChunks = (char*)calloc(1, bufSize);
    if (!_seqChunks) return false;

    _seqPending      = true;
    _seqWantFrag     = WCB_PACKET_SEQ_FRAG;   // names: a whole list (14) or a page (29)
    _seqReqKey[0]    = '\0';
    _seqTargetWCB    = wcbNumber;
    _seqSessionId    = 0;      // adopt whatever session the first frag carries
    _seqTotalChunks  = 0;
    _seqReceivedMask = 0;
    _seqFragType     = 0;
    _seqDoneSession  = 0;
    _seqAsm.reset();
    _seqTries        = 1;
    _seqRestarts     = 0;
    _seqHeardMs      = 0;
    _seqComplete     = false;
    _seqStartMs      = millis();
    _seqDeadlineMs   = _seqStartMs + WCB_SEQ_TIMEOUT_MS;

    _seqSendNames(0, true);
    return true;
}

bool WCB_Client::requestSequence(uint8_t wcbNumber, const char* key) {
    if (!_started)                                    return false;
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS)  return false;
    if (!wcbSeqKeyValid(key))                         return false;
    if (_seqPending)                                  return false;   // shares the one slot
    if (!_seqValueCallback)                           return false;

    const size_t bufSize = (size_t)WCB_SEQ_MAX_CHUNKS * (WCB_SEQ_PAYLOAD_SIZE + 1);
    _seqChunks = (char*)calloc(1, bufSize);
    if (!_seqChunks) return false;

    _seqPending      = true;
    _seqWantFrag     = WCB_PACKET_SEQVAL_FRAG;
    strncpy(_seqReqKey, key, sizeof(_seqReqKey) - 1);
    _seqReqKey[sizeof(_seqReqKey) - 1] = '\0';
    _seqTargetWCB    = wcbNumber;
    _seqSessionId    = 0;
    _seqTotalChunks  = 0;
    _seqReceivedMask = 0;
    _seqFragType     = 0;
    _seqDoneSession  = 0;
    _seqComplete     = false;
    _seqDeadlineMs   = millis() + WCB_SEQ_TIMEOUT_MS;

    wcb_packet_seqval_req_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    pkt.packetType   = WCB_PACKET_SEQVAL_REQ;
    pkt.targetWCB    = wcbNumber;
    pkt.requesterWCB = (uint8_t)_deviceID;
    strncpy(pkt.key, key, sizeof(pkt.key) - 1);

    // 3x, deduped target-side on (requester, key) — see requestSequenceNames().
    for (int i = 0; i < 3; i++) {
        _espSend(_broadcastMAC, (const uint8_t*)&pkt, sizeof(pkt));
        delay(15);
    }
    return true;
}

// ── Writing ─────────────────────────────────────────────────────────────────
// Both writes go out as ordinary commands: send() already fragments anything over
// one packet and the firmware already has a `?SEQ,SAVE` parser that knows not to
// split on the '^' delimiters inside a sequence value. There is no bespoke write
// packet, and there should not be — a second path would drift from the one the
// Wizard and the console use.

bool WCB_Client::saveSequence(uint8_t wcbNumber, const char* key, const char* value) {
    if (!_started)                                    return false;
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS)  return false;
    if (!wcbSeqKeyValid(key))                         return false;
    if (!value || *value == '\0')                     return false;   // firmware rejects an empty value

    String cmd = "?SEQ,SAVE,";
    cmd += key;
    cmd += ",";
    cmd += value;
    if (cmd.length() > WCB_MGMT_MAX_COMMAND_LEN) return false;   // send() would drop it
    return send(wcbNumber, cmd.c_str());
}

bool WCB_Client::deleteSequence(uint8_t wcbNumber, const char* key) {
    if (!_started)                                    return false;
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS)  return false;
    if (!wcbSeqKeyValid(key))                         return false;

    String cmd = "?SEQ,CLEAR,";
    cmd += key;
    return send(wcbNumber, cmd.c_str());
}

// WiFi/RX task. Keep this to memcpy + flag work only — the user callback fires
// from _seqTick() on the loop task, where doing real work is safe.
bool WCB_Client::_maybeHandleSeqFrag(const uint8_t* data, int len) {
    if (!_seqPending || !_seqChunks)                     return false;
    if (len != (int)sizeof(wcb_packet_seq_frag_t))       return false;

    wcb_packet_seq_frag_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';

    // Must be the kind of reply this slot is waiting for. A frag of the OTHER kind
    // (a late leftover from a previous request) would otherwise be reassembled into
    // the current one and delivered as garbage. Names come whole (14) or as a page (29).
    const bool names = _seqWantFrag == WCB_PACKET_SEQ_FRAG;
    if (names ? (pkt.packetType != WCB_PACKET_SEQ_FRAG && pkt.packetType != WCB_PACKET_SEQ_PAGE)
              : pkt.packetType != _seqWantFrag)                    return false;
    if (strncmp(pkt.structPassword, _password,
                sizeof(pkt.structPassword) - 1) != 0)              return false;
    if (pkt.requesterWCB != (uint8_t)_deviceID)                    return false;
    if (pkt.sourceWCB   != _seqTargetWCB)                          return false;
    // Bound-check BEFORE indexing: totalChunks==0 would make expectedMask 0xFFFF
    // and the session never complete; an out-of-range chunkIdx would write past
    // the buffer.
    if (pkt.totalChunks == 0 || pkt.totalChunks > WCB_SEQ_MAX_CHUNKS) return true;
    if (pkt.chunkIdx >= pkt.totalChunks)                             return true;

    // Guarded: _seqTick() may be freeing this buffer, or clearing it for the next page,
    // on the loop task right now. Re-test _seqChunks INSIDE the lock — the check at the
    // top of this function is only a cheap early-out. The session is adopted inside it
    // too, so a re-arm for the next page cannot land between the test and the adoption.
    portENTER_CRITICAL(&_seqMux);
    if (_seqChunks && !_seqComplete && pkt.sessionId != _seqDoneSession) {
        if (_seqSessionId == 0) {          // first frag of the answer — adopt its session
            _seqSessionId   = pkt.sessionId;
            _seqTotalChunks = pkt.totalChunks;
            _seqFragType    = pkt.packetType;
        }
        // A frag of another session (a late copy of an earlier answer) is consumed but ignored.
        if (pkt.sessionId == _seqSessionId && pkt.packetType == _seqFragType &&
            pkt.totalChunks == _seqTotalChunks && !(_seqReceivedMask & (1 << pkt.chunkIdx))) {
            char* slot = _seqChunks + (size_t)pkt.chunkIdx * (WCB_SEQ_PAYLOAD_SIZE + 1);
            memcpy(slot, pkt.payload, WCB_SEQ_PAYLOAD_SIZE);
            slot[WCB_SEQ_PAYLOAD_SIZE] = '\0';
            _seqReceivedMask |= (1 << pkt.chunkIdx);
            _seqHeardMs = millis();
            if (_seqReceivedMask == (uint16_t)((1 << _seqTotalChunks) - 1)) _seqComplete = true;
        }
    }
    portEXIT_CRITICAL(&_seqMux);
    return true;
}

// Loop task: one page of a names list that did not fit one reply (WCB_SeqNamesWire.h). Checks
// it, joins it into _seqNames, and asks for the next page, starts over, or delivers the list.
// The reassembly slot is still held (_seqComplete); every path re-arms it, resets, or delivers.
void WCB_Client::_seqNamesPage(const String& full, unsigned long now) {
    (void)now;
    const uint16_t sid = _seqSessionId;
    seqn::PageHeader h;
    size_t keysAt = 0;
    if (!seqn::parsePageHeader(full.c_str(), full.length(), h, keysAt)) {
        _seqRearm(sid);                    // not a page: keep waiting (the retry timer asks again)
        return;
    }
    seqn::Step st = _seqAsm.check(h);
    if (st == seqn::STEP_STALE || st == seqn::STEP_BAD) {
        _seqRearm(sid);
        return;
    }
    if (st == seqn::STEP_OK && h.page == 0) {
        if (_seqNames) free(_seqNames);
        _seqNamesCap = (size_t)seqn::wholeLen(h.count, h.keyBytes) + 1;
        _seqNames    = (char*)malloc(_seqNamesCap);
        if (!_seqNames || !_seqAsm.start(h, _seqNames, _seqNamesCap)) {
            Serial.printf("[WCB_Client] sequence names from WCB%u: %u names need %u bytes the heap does not have "
                          "- abandoned\n", (unsigned)_seqTargetWCB, h.count, (unsigned)_seqNamesCap);
            _seqReset();
            return;
        }
    }
    if (st == seqn::STEP_OK)
        st = _seqAsm.append(full.c_str() + keysAt, full.length() - keysAt, _seqNames, _seqNamesCap);
    if (st == seqn::STEP_BAD) {
        _seqRearm(sid);                    // malformed: keep waiting (the retry timer asks again)
        return;
    }
    if (st == seqn::STEP_RESTART) {
        if (_seqRestarts >= seqn::RESTARTS_MAX) {
            Serial.printf("[WCB_Client] sequence names from WCB%u changed %u times while they were read - "
                          "abandoned\n", (unsigned)_seqTargetWCB, (unsigned)_seqRestarts + 1);
            _seqReset();
            return;
        }
        _seqRestarts++;
        _seqAsm.reset();
        _seqRearm(sid);
        _seqTries = 1;
        _seqSendNames(0, false);           // it pages, so it is new enough for a 28
        return;
    }
    if (st == seqn::STEP_NEXT) {
        _seqRearm(sid);
        _seqTries = 1;
        _seqSendNames(_seqAsm.next, false);
        return;
    }
    // STEP_DONE: the whole list, once. Detach it from the slot before the reset so the
    // consumer may issue the next request from inside the callback.
    // _seqNames is "<8 hex>,<count>,k1,k2,...": the names start after the second comma.
    char*          list  = _seqNames;
    const size_t   hl    = (size_t)seqn::digits(_seqAsm.first.count) + 9;   // "<8 hex>,<count>"
    const uint32_t hash  = _seqAsm.first.hash;
    const uint16_t count = (uint16_t)_seqAsm.first.count;
    const uint8_t  src   = _seqTargetWCB;
    const char*    names = (count && _seqAsm.len > hl && list[hl] == ',') ? list + hl + 1 : "";
    _seqNames = nullptr;
    WCBSequenceNamesCallback cb = _seqNamesCallback;
    _seqReset();
    if (cb) cb(src, hash, count, names);
    free(list);
}

// Loop task: deliver a completed inventory, take a page, or abandon a request that timed out.
void WCB_Client::_seqTick(unsigned long now) {
    if (!_seqPending) return;

    if (_seqComplete) {
        // The RX task writes nothing while _seqComplete is set, so the chunks read below
        // are ours; the slot is re-armed for a next page, or reset.
        portENTER_CRITICAL(&_seqMux);
        char*   buf   = _seqChunks;
        uint8_t total = _seqTotalChunks;
        uint8_t ftype = _seqFragType;
        portEXIT_CRITICAL(&_seqMux);
        if (!buf) { _seqReset(); return; }

        String full;
        for (int i = 0; i < total; i++)
            full += (const char*)(buf + (size_t)i * (WCB_SEQ_PAYLOAD_SIZE + 1));

        uint8_t src  = _seqTargetWCB;
        uint8_t kind = _seqWantFrag;
        String  askedKey(_seqReqKey);

        // Both branches copy what the callback needs into locals FIRST, then clear
        // the in-flight state, then fire — so the consumer may issue the next
        // request from inside the callback. The locals outlive _seqReset() because
        // they are copies, not views into the freed buffer.
        if (kind == WCB_PACKET_SEQVAL_FRAG) {
            // "<key>,<status>,<value>" — value may itself contain commas, so take
            // everything after the SECOND comma verbatim.
            String   key    = askedKey;
            String   value  = "";
            WCBSeqStatus st = WCB_SEQ_NOTFOUND;

            int c1 = full.indexOf(',');
            int c2 = (c1 >= 0) ? full.indexOf(',', c1 + 1) : -1;
            if (c1 >= 0 && c2 > c1) {
                key = full.substring(0, c1);
                String s = full.substring(c1 + 1, c2);
                if      (s == "OK")     { st = WCB_SEQ_OK; value = full.substring(c2 + 1); }
                else if (s == "TOOBIG")   st = WCB_SEQ_TOOBIG;
                else                      st = WCB_SEQ_NOTFOUND;
            }

            WCBSequenceValueCallback cb = _seqValueCallback;
            _seqReset();
            if (cb) cb(src, key.c_str(), st, value.c_str());
            return;
        }

        if (ftype == WCB_PACKET_SEQ_PAGE) {
            _seqNamesPage(full, now);
            return;
        }

        // Names in one reply: "<hash8hex>,<count>[,<key>...]" — parsed here so every
        // consumer doesn't reimplement it, and checked whole: the count must be the
        // number of keys. Anything else is not an inventory - the "[ERROR] Result too
        // large to relay" text a WCB before paged names sends for a list that does not
        // fit, which 1.18.1 and earlier delivered as an EMPTY list - and is dropped.
        uint32_t hash = 0;
        unsigned count = 0;
        size_t   at = 0;
        const bool ok = seqn::parseWhole(full.c_str(), full.length(), hash, count, at);
        WCBSequenceNamesCallback cb = _seqNamesCallback;
        _seqReset();
        if (!ok) {
            Serial.printf("[WCB_Client] sequence names from WCB%u: the reply is not a list (\"%.40s\") - dropped\n",
                          (unsigned)src, full.c_str());
            return;
        }
        String nameList = full.substring(at);
        if (cb) cb(src, hash, (uint16_t)count, nameList.c_str());
        return;
    }

    if (_seqWantFrag == WCB_PACKET_SEQ_FRAG) {
        // Names. Page 0 of a first pull is asked once, as the names always were: silence
        // within WCB_SEQ_TIMEOUT_MS is a board that is not there. Once the board has paged
        // (a later page, or page 0 again after the list changed), a page that brings no
        // frag for seqn::RETRY_MS is asked again, up to seqn::PAGE_TRIES times.
        if ((long)(now - _seqStartMs) >= (long)seqn::PULL_MAX_MS) {
            Serial.printf("[WCB_Client] sequence names from WCB%u did not finish in %u s - abandoned\n",
                          (unsigned)_seqTargetWCB, (unsigned)(seqn::PULL_MAX_MS / 1000));
            _seqReset();
            return;
        }
        if (_seqAsm.next == 0 && _seqRestarts == 0) {
            if ((long)(now - _seqDeadlineMs) >= 0) _seqReset();   // silent, as before
            return;
        }
        const unsigned long heard = _seqHeardMs;
        const unsigned long last  = (long)(heard - _seqAskedMs) > 0 ? heard : _seqAskedMs;
        if ((long)(now - last) < (long)seqn::RETRY_MS) return;
        if (_seqTries >= seqn::PAGE_TRIES) {
            Serial.printf("[WCB_Client] sequence names from WCB%u: page %u never came (asked %u times) - abandoned\n",
                          (unsigned)_seqTargetWCB, _seqAsm.next + 1, (unsigned)_seqTries);
            _seqReset();
            return;
        }
        _seqTries++;
        _seqSendNames(_seqAsm.next, false);
        return;
    }

    if ((long)(now - _seqDeadlineMs) >= 0) {
        // Silent abandon — the caller polls sequenceNamesPending() or just retries.
        // No callback: firing one with a half-list would look like a real answer.
        _seqReset();
    }
}

// Enable/disable auto-join. Turning it OFF stops NEW boards from being learned;
// already-registered learned peers stay until they age out or the device
// restarts. Turning it ON lets subsequently-heard boards join.
void WCB_Client::setAutoJoin(bool enabled) {
    _autoJoin = enabled;
}

// Expire neighbors we haven't heard an advert from within the TTL. Fires
// onNeighbor once more with valid=false so the app can drop it from any UI.
void WCB_Client::_ageNeighbors(unsigned long now) {
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        WCBNeighbor& nb = _neighbors[i];
        if (!nb.valid) continue;
        // A temporary peer uses a shorter TTL so a powered-off relay clears from the roster
        // promptly instead of lingering the full ~3 min neighbor TTL. It adverts faster
        // (WCB_WDP_TEMP_ADVERT_MS) to keep this window from flapping on a dropped advert.
        const unsigned long ttl = nb.temporary ? WCB_WDP_TEMP_NEIGHBOR_TTL_MS : WCB_WDP_NEIGHBOR_TTL_MS;
        if ((long)(now - nb.lastSeenMs) >= (long)ttl) {
            nb.valid = false;
            portENTER_CRITICAL(&_csMux);
            _csPeer[i].has = false;          // channel sync: its key is derived again until it re-advertises
            portEXIT_CRITICAL(&_csMux);
            if (_neighborCallback) _neighborCallback(nb);
        }
    }
}

// Unicast a raw buffer to a WCB's MAC (computed scheme). Registers the peer on
// demand so a custom protocol (e.g. OTA) can reach any WCB without relying on
// _registerPeers() having already added it. Returns true if ESP-NOW accepted it.
bool WCB_Client::sendRawPacket(uint8_t target_wcb, const uint8_t* data, size_t len) {
    if (!_started) return false;   // ESP-NOW not up (begin() failed / not called)
    if (target_wcb < 1 || target_wcb > WCB_MAX_BOARDS || !data || len == 0) return false;
    _csNoteBusy();                 // an OTA or relay protocol: no channel-sync search under it
    const uint8_t* mac = _wcbMACs[target_wcb - 1];
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) return false;
    }
    return _espSend(mac, data, len) == ESP_OK;
}

// Enable or disable CRC32 checksum. Must match the WCB network's ?ETM,CHKSM setting.
void WCB_Client::setChecksum(bool enabled) {
    _checksumEnabled = enabled;
}

// Set the ESP-NOW mesh channel this device expects (1..maxChannel()). See the header
// for the full contract. Out-of-range values are ignored (the current value is kept).
// _csMaxCh is 11 under "01" and 13 under any other region before begin() (setRegion),
// and the region's real top channel after it.
void WCB_Client::setMeshChannel(uint8_t channel) {
    if (!chansync::validChannel(channel, _csMaxCh)) {
        Serial.printf("[WCB_Client] setMeshChannel: %u out of range (1-%u, region \"%s\"), ignored\n",
                      channel, _csMaxCh, _csRegion);
        return;
    }
    _meshChannel = channel;
    // A channel-sync search is hopping the radio: it returns to this channel instead.
    if (_started && _csSearch != CS_SEARCH_IDLE) { _csHomeCh = channel; return; }
    // If begin() already ran and we own the radio (no SoftAP), re-pin it live so a
    // runtime change actually takes effect — otherwise this would only update the
    // expected value while the radio stayed put. With a SoftAP active the AP owns
    // the channel; we don't move it (update()'s _checkMeshChannel warns instead).
    if (_started) {
        wifi_mode_t mode = WiFi.getMode();
        bool apActive = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
        if (!apActive) esp_wifi_set_channel(_meshChannel, WIFI_SECOND_CHAN_NONE);
        // A host that took a channel-sync move (onChannelMove returned true) and applies it
        // live, without a restart, does it with this call: the device has arrived.
        uint8_t primary = 0;
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
        if (_csHostMoveCh && channel == _csHostMoveCh &&
            esp_wifi_get_channel(&primary, &second) == ESP_OK && primary == channel) {
            _csHostMoveCh = 0;
            _csArrived(channel, millis());
        }
    }
}

// Warn (rate-limited) if the radio isn't on the expected mesh channel — this device
// can't reach the mesh when off-channel, so surface it loudly instead of failing
// silently. The advice depends on WHY the radio moved: a hosted SoftAP owns the
// channel (bring the AP up on the mesh channel), otherwise a STA association or a
// channel change moved it. We only warn (never force) so we don't yank a
// deliberately-chosen SoftAP channel.
//
// NOTE: this can only catch a radio that drifted OFF _meshChannel. It cannot detect
// a _meshChannel that is itself wrong (the fleet moved via ?WCBCH but this sketch was
// never updated) — an ESP-NOW device on the wrong channel hears nothing, so it has no
// way to learn the fleet's real channel. Keep _meshChannel in sync via setMeshChannel().
void WCB_Client::_checkMeshChannel() {
    // A channel-sync search moves the radio off _meshChannel on purpose, for ~1.5 s.
    if (_csOffHome || _csSearch != CS_SEARCH_IDLE) return;
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) != ESP_OK) return;
    if (primary == _meshChannel) { _lastWarnedChannel = 0; return; }  // on-channel: reset so a later drift re-warns

    unsigned long now = millis();
    // Warn on first detection / a change of wrong channel; otherwise throttle to 30s.
    if (primary == _lastWarnedChannel && (now - _lastChannelWarnMs) < 30000UL) return;
    _lastWarnedChannel = primary;
    _lastChannelWarnMs = now;
    wifi_mode_t mode = WiFi.getMode();
    bool apActive = (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA);
    if (apActive) {
        Serial.printf("[WCB_Client] WARNING: radio is on channel %u but the WCB mesh is on "
                      "channel %u - this device will NOT reach the mesh. A hosted SoftAP owns "
                      "the radio channel; bring the AP up on the mesh channel: "
                      "WiFi.softAP(ssid, pass, %u).\n",
                      primary, _meshChannel, _meshChannel);
    } else {
        Serial.printf("[WCB_Client] WARNING: radio is on channel %u but the WCB mesh is on "
                      "channel %u - this device will NOT reach the mesh. A WiFi STA association "
                      "or a channel change may have moved it; call setMeshChannel(%u) or reboot.\n",
                      primary, _meshChannel, _meshChannel);
    }
}

// =============================================================================
// Private helpers
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// _buildMACs
//
// Computes the MAC address for every possible WCB slot (1–WCB_MAX_BOARDS)
// and for the network broadcast address using the stored oct2/oct3 values.
//
// WCB MAC scheme:
//   02 : locally administered, unicast bit set — required for custom MACs
//   oct2, oct3 : shared octets that identify this WCB network
//   00, 00 : reserved / always zero
//   ID : WCB number (1-based, stored in the last octet)
//
// Broadcast MAC:
//   FF:oct2:oct3:FF:FF:FF
//   The non-standard broadcast (not FF:FF:FF:FF:FF:FF) scopes the broadcast
//   to this network's oct2/oct3, so different WCB networks on the same channel
//   don't interfere with each other.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_buildMACs() {
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        _wcbMACs[i][0] = 0x02;
        _wcbMACs[i][1] = _oct2;
        _wcbMACs[i][2] = _oct3;
        _wcbMACs[i][3] = 0x00;
        _wcbMACs[i][4] = 0x00;
        _wcbMACs[i][5] = (uint8_t)(i + 1);  // WCB number, 1-based
    }

    _broadcastMAC[0] = 0xFF;
    _broadcastMAC[1] = _oct2;
    _broadcastMAC[2] = _oct3;
    _broadcastMAC[3] = 0xFF;
    _broadcastMAC[4] = 0xFF;
    _broadcastMAC[5] = 0xFF;
}

// ─────────────────────────────────────────────────────────────────────────────
// _registerPeers
//
// Adds each WCB (1 through wcb_quantity) and the broadcast MAC as ESP-NOW
// peers. ESP-NOW requires peers to be pre-registered before you can send to
// them; receiving does not require peer registration.
//
// This device's own slot is skipped (you can't send to yourself).
// The special slot (ID 20) outside the 1–quantity range is not registered here
// because WCBs with specialPeerEnabled already have that MAC registered on
// their side, allowing bidirectional communication.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_registerPeers() {
    for (int i = 0; i < _quantity; i++) {
        if ((i + 1) == _deviceID) continue;  // Skip our own slot

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[i], 6);
        peer.channel = 0;       // 0 = use the current WiFi channel
        peer.encrypt = false;   // WCB network uses password auth, not ESP-NOW encryption
        esp_now_add_peer(&peer);
    }

    // Register the broadcast MAC so broadcast() can send via esp_now_send()
    esp_now_peer_info_t bcast = {};
    memcpy(bcast.peer_addr, _broadcastMAC, 6);
    bcast.channel = 0;
    bcast.encrypt = false;
    esp_now_add_peer(&bcast);

    // Register the out-of-band special peer (NaviCore) if one was enabled.
    _registerSpecialPeer();
}

// ─────────────────────────────────────────────────────────────────────────────
// _registerSpecialPeer
//
// Registers the special peer's MAC as an ESP-NOW peer so send()/sendToSpecialPeer()
// can reach it. Only needed when the special peer sits OUTSIDE the 1..quantity
// range (IDs within that range are already registered by _registerPeers()).
// No-op when no special peer is enabled, when it's this device's own ID, or when
// it's already a normal peer.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_registerSpecialPeer() {
    if (_specialPeerID < 1 || _specialPeerID > WCB_MAX_BOARDS) return;
    if (_specialPeerID == _deviceID)  return;   // that's us
    if (_specialPeerID <= _quantity)  return;   // already registered as a normal peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, _wcbMACs[_specialPeerID - 1], 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

// ─────────────────────────────────────────────────────────────────────────────
// _addLearnedPeer
//
// Register a regular WCB (learned via a WDP advert) as an ESP-NOW peer, live,
// and remember it in NVS. Uses the same derived MAC scheme as _buildMACs(), so
// no address is learned — only the FACT that the board exists. Idempotent, and
// guarded against self, the special peer, ids already covered by the
// 1..quantity floor, and the ~20-peer ESP-NOW cap. A learned peer is PERMANENT:
// it is restored on every begin() and is always expected to be on and ready.
// Offline-detection tracks its heartbeat liveness, but membership never
// self-evicts — only forgetPeer()/clearLearnedPeers() remove it. If the peer
// table gets crowded, clearing it up is deliberately the user's call.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_addLearnedPeer(uint8_t id) {
    if (id < 1 || id > WCB_MAX_BOARDS)  return false;
    if (id == _deviceID)                return false;   // that's us
    if (id == _specialPeerID || id == WCB_SPECIAL_ID)
                                        return false;   // controller slot — never learn/persist it
    if (id <= _quantity)                return false;   // already a floor peer
    int idx = id - 1;
    if (_learnedPeer[idx])              return true;    // already joined

    if (!esp_now_is_peer_exist(_wcbMACs[idx])) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[idx], 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.printf("[WCB_Client] auto-join: could not add WCB%d (peer table full?)\n", id);
            return false;
        }
    }
    _learnedPeer[idx] = true;
    _saveLearnedPeers();     // joining is a rare, one-time event — persist immediately
    Serial.printf("[WCB_Client] auto-joined WCB%d\n", id);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Learned-peer persistence (NVS namespace "wcb_peers")
//
// Only MEMBERSHIP is stored, as a 20-bit mask — identity re-learns from adverts
// and MACs are always derived, so there is nothing else to keep. The blob is
// fingerprinted with the network octets: change oct2/oct3 (a different mesh)
// and the old membership is discarded instead of deriving wrong-group MACs.
// Mirrors the WCB firmware's "learned_peers" scheme.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_saveLearnedPeers() {
    uint32_t mask = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++)
        if (_learnedPeer[i]) mask |= (1UL << i);
    Preferences prefs;
    prefs.begin("wcb_peers", false);
    prefs.putUChar("ver", 1);
    prefs.putUChar("o2", _oct2);
    prefs.putUChar("o3", _oct3);
    prefs.putUInt("mask", mask);
    prefs.end();
}

void WCB_Client::_loadLearnedPeers() {
    Preferences prefs;
    prefs.begin("wcb_peers", true);
    uint8_t  ver  = prefs.getUChar("ver", 0);
    uint8_t  o2   = prefs.getUChar("o2", 0);
    uint8_t  o3   = prefs.getUChar("o3", 0);
    uint32_t mask = prefs.getUInt("mask", 0);
    prefs.end();

    if (ver != 1) return;                       // nothing saved (or unknown schema)
    if (o2 != _oct2 || o3 != _oct3) {           // saved under a different network
        Preferences wipe;
        wipe.begin("wcb_peers", false);
        wipe.clear();
        wipe.end();
        return;
    }

    int restored = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (!(mask & (1UL << i))) continue;
        uint8_t id = i + 1;
        // Ids now covered by the floor/special/self are skipped (e.g. the app
        // raised wcb_quantity since the save) — _addLearnedPeer re-derives the
        // guards, so a stale bit simply doesn't re-join.
        if (_addLearnedPeer(id)) restored++;
    }
    if (restored > 0)
        Serial.printf("[WCB_Client] restored %d learned peer(s)\n", restored);
}

// Forget one auto-joined peer: deregister it, drop it from NVS, mark it offline.
// Floor peers (1..wcb_quantity) and the special peer are unaffected.
void WCB_Client::forgetPeer(uint8_t id) {
    if (id < 1 || id > WCB_MAX_BOARDS) return;
    // Never deregister the special peer's ESP-NOW MAC. clearLearnedPeers()
    // funnels through here, and _specialPeerID can still be 0 when the special
    // peer is enabled AFTER begin(), so guard the reserved controller slot
    // unconditionally — otherwise a routine table cleanup would silently break
    // sendToSpecialPeer() by deleting the peer whose MAC it shares.
    if (id == _specialPeerID || id == WCB_SPECIAL_ID) return;
    int idx = id - 1;
    if (!_learnedPeer[idx]) return;
    _learnedPeer[idx] = false;
    _advertCount[idx] = 0;   // reset so it must be heard >=2 times again before it can re-join
    if (esp_now_is_peer_exist(_wcbMACs[idx])) esp_now_del_peer(_wcbMACs[idx]);
    _boards[idx].online = false;
    // Also drop the WDP discovery record so getNeighbor(id) goes null immediately —
    // otherwise a just-forgotten peer keeps showing (via getNeighbor) for up to
    // WCB_WDP_NEIGHBOR_TTL_MS until the advert ages out. It repopulates on its next
    // advert (and re-joins via auto-join after >=2 adverts), which is intended.
    _neighbors[idx] = WCBNeighbor{};
    portENTER_CRITICAL(&_csMux);
    _csPeer[idx].has = false;
    portEXIT_CRITICAL(&_csMux);
    _saveLearnedPeers();
    Serial.printf("[WCB_Client] forgot WCB%d\n", id);
}

// Forget ALL auto-joined peers (the user-cleanup handle for a crowded table).
void WCB_Client::clearLearnedPeers() {
    for (int i = 0; i < WCB_MAX_BOARDS; i++)
        if (_learnedPeer[i]) forgetPeer(i + 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendHeartbeat
//
// Broadcasts a HEARTBEAT packet to the network. Heartbeats carry no command
// payload (structCommandIncluded = 0); they exist purely to tell every other
// WCB "I am alive." WCBs that don't see a heartbeat within the expected window
// will mark this device offline and may stop routing messages to it.
// ─────────────────────────────────────────────────────────────────────────────
// Announce our boot the way a WCB does. A WCB clears a sender's duplicate ring ONLY on that
// sender's boot announce; without this, a client that reboots reuses sequence numbers the WCB
// still holds, so its first commands are ACKed and then dropped as duplicates - invisible at
// both ends. Unacknowledged, hence sent redundantly (the firmware does the same).
void WCB_Client::_sendBootAnnounce() {
    if (!_started) return;

    wcb_packet_etm_t ba;
    memset(&ba, 0, sizeof(ba));

    strncpy(ba.structPassword, _password, sizeof(ba.structPassword) - 1);
    snprintf(ba.structSenderID, sizeof(ba.structSenderID), "%d", _deviceID);
    snprintf(ba.structTargetID, sizeof(ba.structTargetID), "0");   // 0 = broadcast
    ba.structCommandIncluded = 0;
    ba.structPacketType      = WCB_PACKET_ETM_BOOT;
    ba.structSequenceNumber  = 0;

    _espSend(_broadcastMAC, (uint8_t*)&ba, sizeof(ba));
}

void WCB_Client::_sendHeartbeat() {
    if (!_started) return;   // ESP-NOW not up — never touch esp_now_send()

    wcb_packet_etm_t hb;
    memset(&hb, 0, sizeof(hb));

    strncpy(hb.structPassword, _password, sizeof(hb.structPassword) - 1);
    snprintf(hb.structSenderID, sizeof(hb.structSenderID), "%d", _deviceID);
    snprintf(hb.structTargetID, sizeof(hb.structTargetID), "0");  // 0 = broadcast
    hb.structCommandIncluded = 0;         // No command payload
    hb.structPacketType      = WCB_PACKET_HEARTBEAT;
    hb.structSequenceNumber  = 0;         // Heartbeats don't use sequence numbers

    _espSend(_broadcastMAC, (uint8_t*)&hb, sizeof(hb));
}

// ─────────────────────────────────────────────────────────────────────────────
// WDP device-identity advert
//
// Broadcasts this device's identity (set via setIdentity) on the mesh so every
// WCB discovers it through its Wireless Discovery Protocol neighbor table. The
// wire format MUST match the WCB firmware's WCB_WDP: a 2-byte header (magic 'W',
// proto 0x01) then [type][len][value] TLVs, carried raw in the ETM packet's
// structCommand with structPacketType = WCB_PACKET_WDP. No CRC — WDP carries TLV
// bytes, not a text command.
// ─────────────────────────────────────────────────────────────────────────────
#define WCB_WDP_MAGIC        'W'
#define WCB_WDP_PROTO        0x01
#define WCB_WDP_TLV_END      0x00
#define WCB_WDP_TLV_FWVER    0x03   // string (shared with WCB adverts)
#define WCB_WDP_TLV_DEVTYPE  0x0B   // string — canonical device type name
#define WCB_WDP_TLV_HWREV    0x0C   // string — hardware revision
#define WCB_WDP_TLV_CAPTAGS  0x0D   // string — space-separated capability tags
#define WCB_WDP_TLV_FLAGS    0x12   // [flags:1] — advert flags bitmap (bit0 = TEMPORARY)
#define WCB_WDP_ADVFLAG_TEMPORARY 0x01  // "adopt me TEMPORARY" — WCBs keep it live but NOT
                                        // persisted (dropped on silence/reboot). See setTemporary().
// Decode-only TLVs (WCBs advertise these; the consumer below reads them).
#define WCB_WDP_TLV_SOLICIT   0x11  // len 0 — "everyone advertise now" (?WDP,POLL). Carries no
                                    // facts, so it must NOT be decoded as an advert.
#define WCB_WDP_TLV_ALIAS     0x01  // string — WCB alias
#define WCB_WDP_TLV_HWVER     0x04  // uint8  — WCB numeric hardware version
#define WCB_WDP_TLV_CAPFLAGS  0x05  // uint16 LE — WCB capability bitmap
#define WCB_WDP_TLV_MAESTRO   0x06  // bytes  — local Maestro IDs
#define WCB_WDP_TLV_PORTLABEL 0x09  // [port][label] — one per labeled serial port
#define WCB_WDP_TLV_CTRLID    0x0A  // uint8  — controller (special-peer) ID
#define WCB_WDP_TLV_MAESTRO_CFG 0x0E // [id][baudCode] records — rich Maestro id+baud
#define WCB_WDP_TLV_SEQHASH   0x13  // [hash:4 LE] — stored-sequence inventory fingerprint.
                                    // A change means "re-pull my sequence names"
                                    // (requestSequenceNames). Names are deliberately not
                                    // advertised — they would not fit the 200 B payload.

// Ordered, APPEND-ONLY Maestro baud table. The array INDEX is the wire code
// carried in WCB_WDP_TLV_MAESTRO_CFG, so this must stay byte-identical to the
// firmware's WDP_BAUD_TABLE (WCB_WDP.cpp) — reordering or removing an entry
// silently drifts every advertised baud on the mesh.
static const uint32_t WCB_WDP_BAUD_TABLE[] = {
    0, 110, 300, 600, 1200, 2400, 9600, 14400,
    19200, 38400, 57600, 115200, 128000, 256000
};
static uint8_t wcbWdpBaudToCode(uint32_t baud) {
    for (uint8_t i = 0; i < sizeof(WCB_WDP_BAUD_TABLE) / sizeof(WCB_WDP_BAUD_TABLE[0]); i++)
        if (WCB_WDP_BAUD_TABLE[i] == baud) return i;
    return 0xFF;   // not in the table → advertise "unknown"; receivers won't auto-config it
}

// Append one TLV; returns the new offset (unchanged if it wouldn't fit).
static int wcbPutTLV(uint8_t* buf, int o, int max, uint8_t type,
                     const uint8_t* val, int len) {
    if (len < 0)   len = 0;
    if (len > 255) len = 255;
    if (o + 2 + len > max) return o;   // no room — skip this TLV
    buf[o++] = type;
    buf[o++] = (uint8_t)len;
    for (int i = 0; i < len; i++) buf[o++] = val[i];
    return o;
}

void WCB_Client::setIdentity(const char* type, const char* fw,
                             const char* hwRev, const char* caps) {
    strncpy(_wdpType,  type  ? type  : "", sizeof(_wdpType)  - 1);  _wdpType[sizeof(_wdpType)  - 1] = '\0';
    strncpy(_wdpFw,    fw    ? fw    : "", sizeof(_wdpFw)    - 1);  _wdpFw[sizeof(_wdpFw)      - 1] = '\0';
    strncpy(_wdpHwRev, hwRev ? hwRev : "", sizeof(_wdpHwRev) - 1);  _wdpHwRev[sizeof(_wdpHwRev) - 1] = '\0';
    strncpy(_wdpCaps,  caps  ? caps  : "", sizeof(_wdpCaps)  - 1);  _wdpCaps[sizeof(_wdpCaps)  - 1] = '\0';

    // Arm the cadence so the new identity goes out promptly (short boot burst),
    // then settles to a staggered periodic backstop. Works whether called before
    // or after begin(): the send happens from update() once ESP-NOW is up.
    _wdpBootLeft     = (_wdpType[0]) ? 3 : 0;
    _wdpNextBootMs   = millis() + 300;
    _wdpNextAdvertMs = millis() + (_wdpTemporary ? WCB_WDP_TEMP_ADVERT_MS : 60000UL) + (unsigned long)((_deviceID % 16) * 500);

    if (_wdpType[0])
        Serial.printf("[WCB_Client] WDP identity set: type=\"%s\" fw=\"%s\"\n", _wdpType, _wdpFw);
}

void WCB_Client::setPortLabel(uint8_t port, const char* label) {
    if (port < 1 || port > 5) return;
    char* dst = _wdpPortLabels[port - 1];
    char nv[25];
    if (label) { strncpy(nv, label, sizeof(nv) - 1); nv[sizeof(nv) - 1] = '\0'; }
    else       nv[0] = '\0';
    if (strcmp(dst, nv) == 0) return;           // unchanged — don't churn the advert
    strcpy(dst, nv);
    // Changed → push the new label out promptly (short burst), like the WCB boards'
    // on-change re-advert, so the Wizard/mesh reflect it in ~1 s instead of ~60 s.
    if (_wdpType[0]) { _wdpBootLeft = 2; _wdpNextBootMs = millis() + 200; }
}

void WCB_Client::setMaestroIds(const uint8_t* ids, uint8_t count, const uint32_t* bauds) {
    if (!ids) count = 0;
    if (count > WCB_WDP_MAX_MAESTRO) count = WCB_WDP_MAX_MAESTRO;

    uint8_t nvIds[WCB_WDP_MAX_MAESTRO]   = {0};
    uint8_t nvCodes[WCB_WDP_MAX_MAESTRO] = {0};
    for (uint8_t i = 0; i < count; i++) {
        nvIds[i]   = ids[i];
        nvCodes[i] = bauds ? wcbWdpBaudToCode(bauds[i]) : 0xFF;
    }

    // Unchanged → don't churn the advert (same discipline as setPortLabel).
    if (count == _wdpMaestroCount &&
        memcmp(nvIds,   _wdpMaestroIds,   count) == 0 &&
        memcmp(nvCodes, _wdpMaestroCodes, count) == 0) return;

    memcpy(_wdpMaestroIds,   nvIds,   sizeof(nvIds));
    memcpy(_wdpMaestroCodes, nvCodes, sizeof(nvCodes));
    _wdpMaestroCount = count;

    if (_wdpType[0]) { _wdpBootLeft = 2; _wdpNextBootMs = millis() + 200; }
}

int WCB_Client::_buildWdpPayload(uint8_t* buf, int max) {
    int o = 0;
    if (max < 3) return 0;
    buf[o++] = WCB_WDP_MAGIC;
    buf[o++] = WCB_WDP_PROTO;
    o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_DEVTYPE, (const uint8_t*)_wdpType, strlen(_wdpType));
    if (_wdpFw[0])    o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_FWVER,   (const uint8_t*)_wdpFw,    strlen(_wdpFw));
    if (_wdpHwRev[0]) o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_HWREV,   (const uint8_t*)_wdpHwRev, strlen(_wdpHwRev));
    if (_wdpCaps[0])  o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_CAPTAGS, (const uint8_t*)_wdpCaps,  strlen(_wdpCaps));
    // Channel sync (1.19.0): TLV 0x14 CHAN, [channel][rank][flags][cc0][cc1], in every
    // advert - with SYNC clear when it is off, so the mesh still knows this device's key.
    // Right after the identity, ahead of the labels, so a full advert drops a label first.
    // Older receivers skip it by length. Never PINNED: a client does not join a network.
    {
        chansync::ChanFacts cf;
        cf.channel = currentChannel();
        cf.rank    = chanSyncRank();
        cf.flags   = _csEnabled ? chansync::F_SYNC : 0;
        cf.cc[0]   = _csRegion[0]; cf.cc[1] = _csRegion[1]; cf.cc[2] = '\0';
        o = chansync::putChanTlv(buf, o, max, cf);
    }
    // Local Maestros — the id-only TLV (what older receivers and the ?WDP display
    // read) plus the rich [id][baudCode] TLV a receiver needs to auto-configure a
    // remote proxy. Both, in this order, exactly as the WCB boards emit them.
    if (_wdpMaestroCount > 0) {
        o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_MAESTRO, _wdpMaestroIds, _wdpMaestroCount);
        uint8_t rec[WCB_WDP_MAX_MAESTRO * 2];
        for (uint8_t i = 0; i < _wdpMaestroCount; i++) {
            rec[i * 2]     = _wdpMaestroIds[i];
            rec[i * 2 + 1] = _wdpMaestroCodes[i];
        }
        o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_MAESTRO_CFG, rec, _wdpMaestroCount * 2);
    }
    // Per-serial-port labels — one PORTLABEL TLV each, value = [port][label]. Emitted
    // AFTER the identity TLVs so, if the advert runs tight on space, wcbPutTLV drops
    // trailing labels rather than the identity (mirrors the WCB firmware's policy).
    for (uint8_t p = 1; p <= 5; p++) {
        const char* lbl = _wdpPortLabels[p - 1];
        if (!lbl[0]) continue;
        uint8_t pv[25];
        pv[0] = p;
        int L = (int)strlen(lbl); if (L > 24) L = 24;
        memcpy(pv + 1, lbl, L);
        o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_PORTLABEL, pv, 1 + L);
    }
    if (_wdpTemporary) {   // tell WCBs to adopt this device as a TEMPORARY (not permanent) peer
        uint8_t flags = WCB_WDP_ADVFLAG_TEMPORARY;
        o = wcbPutTLV(buf, o, max, WCB_WDP_TLV_FLAGS, &flags, 1);
    }
    if (o < max) buf[o++] = WCB_WDP_TLV_END;
    return o;
}

void WCB_Client::_sendWdpAdvert() {
    wcb_packet_etm_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "0");   // 0 = broadcast
    pkt.structCommandIncluded = 1;
    // Raw TLV bytes (may contain 0x00) go straight into structCommand — no CRC.
    _buildWdpPayload((uint8_t*)pkt.structCommand, sizeof(pkt.structCommand));
    pkt.structPacketType     = WCB_PACKET_WDP;
    pkt.structSequenceNumber = 0;
    _espSend(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt));
}

void WCB_Client::_wdpTick() {
    if (!_started || !_wdpType[0]) return;   // not running, or nothing to advertise
    unsigned long now = millis();
    if (_wdpBootLeft > 0 && now >= _wdpNextBootMs) {
        _sendWdpAdvert();
        _wdpBootLeft--;
        _wdpNextBootMs = now + 1300;
        // Channel sync: one SOLICIT after the first boot advert, as a WCB sends one
        // (WCB_WDP.cpp wdpTick), so every neighbor re-advertises and this device has their
        // CHAN facts (TLV 0x14) within seconds, not at their next 60 s backstop - until then
        // every key is derived, SYNC off, and this device takes itself for its group's
        // leader. After the first advert, not before it: the boot adverts still to come
        // repair the record of a WCB_Client older than 1.15.1, which takes a solicit for an
        // empty advert. (A device with no identity sends it from _csTick instead.)
        if (!_csBootSolicited) {
            _csBootSolicited = true;
            if (_csEnabled) _sendWdpSolicit();
        }
    }
    if ((long)(now - _wdpNextAdvertMs) >= 0) {   // rollover-safe
        _sendWdpAdvert();
        // A temporary device adverts faster so peers (incl. NaviCore's roster) notice it
        // leave within ~1 min instead of the 60s-cadence / 180s-TTL ~3 min lag.
        _wdpNextAdvertMs = now + (_wdpTemporary ? WCB_WDP_TEMP_ADVERT_MS : 60000UL);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _handleWdpAdvert — WDP consumer
//
// Decode a neighbor's advert payload (magic 'W' + proto + [type][len][value]
// TLVs, riding structCommand) into _neighbors[senderWCB-1] and fire onNeighbor.
// Mirrors the WCB firmware's decoder; a client's DEVTYPE doubles as the name and
// flags isClient. In-RAM parse only — safe to run inline in the receive callback.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_handleWdpAdvert(uint8_t senderWCB, const uint8_t* cmd) {
    if (senderWCB < 1 || senderWCB > WCB_MAX_BOARDS) return;
    if (cmd[0] != (uint8_t)WCB_WDP_MAGIC || cmd[1] != WCB_WDP_PROTO) return;

    // Solicitation check FIRST, exactly as the WCB firmware does (WCB_WDP.cpp). A bare
    // SOLICIT is indistinguishable from an advert at the packet layer — same magic, same
    // proto, same WDP packet type — but carries NO facts. Falling through would memset the
    // sending board’s neighbor record and rebuild it from an empty TLV list, so every
    // "Poll mesh" / ?WDP,POLL blanked the alias, port labels, capability flags, Maestro ids
    // and seqHash of the board that sent it, for up to 60 s until its next real advert.
    {
        int s = 2;
        while (s + 2 <= 200) {
            uint8_t ty = cmd[s];
            if (ty == WCB_WDP_TLV_END) break;
            int ln = cmd[s + 1];
            if (s + 2 + ln > 200) break;
            if (ty == WCB_WDP_TLV_SOLICIT) {
                // Answer it, but NOT from here. This function runs on the ESP-NOW receive
                // callback (WiFi task, small stack) — see the CRITICAL note below — and
                // _sendWdpAdvert() stack-allocates a 252-byte packet and calls esp_now_send.
                // Arm the boot-burst timer instead and let _wdpTick() send it from update(),
                // staggered by device id so a fleet does not answer in lockstep. Mirrors the
                // firmware’s wdpArmSolicitedAdvert(): arm if idle, or pull a pending burst
                // earlier, never lengthen a larger one. Without this the client stays deaf to
                // the poll and its row never refreshes in the mesh panel.
                if (_wdpType[0]) {
                    unsigned long now    = millis();
                    unsigned long jitter = (unsigned long)((_deviceID % 16) * 40);   // 0..600 ms
                    if (_wdpBootLeft < 1) { _wdpBootLeft = 1; _wdpNextBootMs = now + jitter; }
                    else if ((long)((now + jitter) - _wdpNextBootMs) < 0) _wdpNextBootMs = now + jitter;
                }
                return;   // do NOT touch the neighbor table
            }
            s += 2 + ln;
        }
    }

    WCBNeighbor& nb = _neighbors[senderWCB - 1];
    memset(&nb, 0, sizeof(nb));            // a board is the sole authority for its facts — replace wholesale
    nb.valid      = true;
    nb.wcbNumber  = senderWCB;
    nb.lastSeenMs = millis();
    chansync::ChanFacts cf;                // channel sync: this advert's CHAN TLV, if it has one
    bool hasChan = false;

    int o = 2;                             // skip magic + version
    while (o + 2 <= 200) {
        uint8_t type = cmd[o];
        if (type == WCB_WDP_TLV_END) break;
        int len = cmd[o + 1];
        const uint8_t* val = &cmd[o + 2];
        if (o + 2 + len > 200) break;      // truncated / malformed
        switch (type) {
            case WCB_WDP_TLV_ALIAS: {
                int L = len > 24 ? 24 : len; memcpy(nb.name, val, L); nb.name[L] = '\0'; break;
            }
            case WCB_WDP_TLV_DEVTYPE: {    // client device type doubles as the name
                int L = len > 24 ? 24 : len; memcpy(nb.name, val, L); nb.name[L] = '\0';
                nb.isClient = true; break;
            }
            case WCB_WDP_TLV_FWVER: {
                int L = len > 27 ? 27 : len; memcpy(nb.fw, val, L); nb.fw[L] = '\0'; break;
            }
            case WCB_WDP_TLV_HWVER:  if (len >= 1) nb.hwVer = val[0]; break;
            case WCB_WDP_TLV_HWREV: {
                int L = len > 15 ? 15 : len; memcpy(nb.hwRev, val, L); nb.hwRev[L] = '\0'; break;
            }
            case WCB_WDP_TLV_CAPFLAGS:
                if (len >= 2) nb.capFlags = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
                break;
            case WCB_WDP_TLV_CAPTAGS: {
                int L = len > 48 ? 48 : len; memcpy(nb.capTags, val, L); nb.capTags[L] = '\0'; break;
            }
            case WCB_WDP_TLV_CTRLID:  if (len >= 1) nb.ctrlId = val[0]; break;
            case WCB_WDP_TLV_SEQHASH:   // stored-sequence inventory fingerprint (LE)
                if (len >= 4) nb.seqHash = (uint32_t)val[0]         | ((uint32_t)val[1] << 8) |
                                           ((uint32_t)val[2] << 16) | ((uint32_t)val[3] << 24);
                break;
            case WCB_WDP_TLV_FLAGS:   // advert flags bitmap — currently just the "temporary" bit
                if (len >= 1) nb.temporary = (val[0] & WCB_WDP_ADVFLAG_TEMPORARY) != 0;
                break;
            case chansync::TLV_CHAN:  // channel sync key + channel + region (a longer value: future fields)
                if (chansync::parseChanTlv(val, (uint8_t)len, cf)) {
                    hasChan        = true;
                    nb.chanValid   = true;
                    nb.chanChannel = cf.channel;
                    nb.chanRank    = cf.rank;
                    nb.chanFlags   = cf.flags;
                    memcpy(nb.chanCc, cf.cc, sizeof(nb.chanCc));
                }
                break;
            case WCB_WDP_TLV_MAESTRO: {
                int L = len > 9 ? 9 : len; memcpy(nb.maestroIds, val, L); nb.maestroCount = (uint8_t)L; break;
            }
            case WCB_WDP_TLV_PORTLABEL: {
                if (len >= 1) {
                    int port = val[0];
                    if (port >= 1 && port <= 5) {
                        int L = len - 1; if (L > 24) L = 24;
                        memcpy(nb.portLabels[port - 1], val + 1, L);
                        nb.portLabels[port - 1][L] = '\0';
                    }
                }
                break;
            }
            default: break;                // unknown TLV — skipped by length (forward-compatible)
        }
        o += 2 + len;
    }

    // Channel sync reads the key on the loop task (_csKeyOf); copy it under the lock so
    // a judgement never sees half of one advert's facts and half of the next's. An
    // advert without the TLV (the board was reflashed to older firmware) derives again.
    portENTER_CRITICAL(&_csMux);
    _csPeer[senderWCB - 1].has = hasChan;
    if (hasChan) _csPeer[senderWCB - 1].f = cf;
    portEXIT_CRITICAL(&_csMux);
    // Never missed by a search (_csMemberMissing): a device that says it is temporary or
    // never leads. Sticky - the record saying so ages out when the device leaves - so it is
    // a bit here, saved to NVS from the loop task; an id that later advertises as anything
    // else (reassigned) loses it. Only this task writes the word.
    {
        const bool     never = nb.temporary || (hasChan && cf.rank == chansync::RANK_NEVER);
        const uint32_t bit   = 1UL << (senderWCB - 1);
        const uint32_t was   = _csNeverMiss;
        const uint32_t upd   = never ? (was | bit) : (was & ~bit);
        if (upd != was) { _csNeverMiss = upd; _csNeverMissDirty = true; }
    }

    // Auto-join: register a regular WCB (not a client device, not the special
    // peer) as an ESP-NOW peer once we've heard it advertise at least twice, so
    // a single stray/spoofed advert can't add a peer. The sender was bound to its
    // source MAC in _handleReceive, so senderWCB is trustworthy here.
    //
    // CRITICAL: this decode runs in the ESP-NOW RECEIVE CALLBACK (WiFi task) with
    // a small stack. _addLearnedPeer() does esp_now_add_peer() + an NVS flash
    // write, which must NEVER run here (overflows the callback stack → crash /
    // boot loop on the 2nd advert). Only FLAG the join; update() performs it on
    // the loop task.
    // Auto-join learns WCBs AND client devices (mesh monitors, other controllers,
    // command-accepting clients) so anything seen on the mesh becomes a PERSISTENT
    // peer you can send() to — registered on the loop task, saved to NVS, restored
    // every begin(). (_addLearnedPeer still skips self, the special peer, and the
    // 1..wcb_quantity floor.) A learned peer never self-evicts; forgetPeer() /
    // clearLearnedPeers() to drop one.
    // A peer advertising the WDP "temporary" flag (e.g. a management relay) is tracked
    // as a live neighbor while it advertises but is NEVER learned/persisted — it ages
    // out of the RAM table on silence (getNeighbor→null) and is gone on reboot, mirroring
    // how the WCB boards adopt it (addTemporaryPeer). Any stale permanent membership left
    // in NVS is dropped on the loop task in update() (forgetPeer is unsafe on this callback).
    if (_autoJoin && !nb.temporary) {
        if (_advertCount[senderWCB - 1] < 255) _advertCount[senderWCB - 1]++;
        if (_advertCount[senderWCB - 1] >= 2 && !_learnedPeer[senderWCB - 1])
            _pendingJoin[senderWCB - 1] = true;
    }

    if (_neighborCallback) _neighborCallback(nb);
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendAck
//
// Sends a WCB_PACKET_ACK back to the device that sent us a COMMAND.
// The ACK carries the same sequence number as the original COMMAND so the
// sender can match it to the correct pending slot and stop retrying.
//
// ACKs have no command payload (structCommandIncluded = 0).
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_sendAck(uint8_t targetID, uint16_t seqNum) {
    if (targetID < 1 || targetID > WCB_MAX_BOARDS) return;

    wcb_packet_etm_t ack;
    memset(&ack, 0, sizeof(ack));

    strncpy(ack.structPassword, _password, sizeof(ack.structPassword) - 1);
    snprintf(ack.structSenderID, sizeof(ack.structSenderID), "%d", _deviceID);
    snprintf(ack.structTargetID, sizeof(ack.structTargetID), "%d", targetID);
    ack.structCommandIncluded = 0;
    ack.structPacketType      = WCB_PACKET_ACK;
    ack.structSequenceNumber  = seqNum;  // Echo the original sequence number

    _espSend(_wcbMACs[targetID - 1], (uint8_t*)&ack, sizeof(ack));
}

// ─────────────────────────────────────────────────────────────────────────────
// _sendPacket
//
// Entry point for send() and broadcast(). Allocates the sequence number, sets
// up pending-table tracking (and, for ensured sends, snapshots the expected
// recipient set), then hands the actual packet build + transmit to _transmit().
// The CRC suffix and wire framing live in _transmit() so retransmits reuse it.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_sendPacket(uint8_t targetID, const char* command, bool ensured) {
    // Sequence 0 is RESERVED and must never go on the wire. The WCB firmware's
    // per-sender duplicate ring is zero-initialised and reads 0 as "empty slot"
    // ("Seq 0 is never sent, so 0 = empty slot", WCB.ino) — so a seq-0 COMMAND
    // matches an empty entry, gets ACKd (the firmware ACKs BEFORE the dup check)
    // and is then discarded as a duplicate. Every ensured retry reuses the same
    // seq and dies identically, so the pending slot completes on those ACKs and
    // the sender reports a clean ensured delivery for a command that executed on
    // no board at all. Our own receive path exempts seq 0 from de-dup
    // (_handleReceive), so the mirror case double-fires instead. Skipping 0 on
    // the ~65k wrap costs one compare and keeps both sides honest.
    uint16_t seq = ++_seqCounter;   // atomic; never reused by a retransmit
    if (seq == 0) seq = ++_seqCounter;

    // Decide whether to record this packet in the pending table:
    //   - unicast            → tracked (ACK info; freed on the target's ACK).
    //   - ensured broadcast  → tracked (update() retransmits until complete).
    //   - best-effort bcast  → NOT tracked. Its ACK slot would never free (the
    //                          ACK handler keeps broadcast slots open to collect
    //                          multi-board ACKs), so tracking an un-ensured one
    //                          would just leak the small WCB_PENDING_MAX table.
    bool track = ensured || (targetID != WCB_TARGET_BROADCAST);

    // Claim + fill the pending slot atomically vs the cross-core ACK handler.
    // _findFreePending assumes _pendingMux is held; keep only array work inside
    // the critical section (no esp_now/Serial) — the transmit happens after.
    portENTER_CRITICAL(&_pendingMux);
    int slot = track ? _findFreePending() : -1;
    if (slot >= 0) {
        WCBPending& p = _pending[slot];
        p.active   = true;
        p.seqNum   = seq;
        p.sentMs   = millis();
        p.targetID = targetID;
        p.ensured  = ensured;
        strncpy(p.command, command, sizeof(p.command) - 1);
        p.command[sizeof(p.command) - 1] = '\0';   // strncpy may not NUL-terminate
        memset(p.ackReceived, 0, sizeof(p.ackReceived));
        memset(p.expected,    0, sizeof(p.expected));
        memset(p.retryCount,  0, sizeof(p.retryCount));
        if (ensured) {
            // Snapshot who we'll wait for. For a broadcast that's every board we
            // currently believe is online (except ourself); for a unicast it's
            // just the target. A board that later drops offline stops blocking
            // completion (see _ensuredComplete), so we never retry forever to a
            // board that's simply gone.
            if (targetID == WCB_TARGET_BROADCAST) {
                for (int b = 0; b < WCB_MAX_BOARDS; b++)
                    if (_boards[b].online && (b + 1) != _deviceID)
                        p.expected[b] = true;
            } else {
                p.expected[targetID - 1] = true;
            }
        }
    }

    // ── Delivery statistics (see WCBPeerStats) ───────────────────────────────
    // Credit `sent` to every peer this delivery is actually aimed at. For an
    // ensured broadcast that is every board in expected[] — an ensured broadcast
    // IS a per-board guaranteed delivery (retried per board at :283-296, ACKd
    // per board), so each board it waits on owns a share of it. Crediting only
    // _statBroadcastSent would leave those boards' ackd/retries/failed with no
    // matching `sent` and break the per-peer invariant.
    //
    // _statBroadcastSent counts broadcast FRAMES on the air, ensured or not, and
    // is NOT a subset of any peer's `sent`. An ensured broadcast increments both.
    //
    // `degraded`: the saturation case handled just below — no slot was free, so
    // this send goes out once, untracked, and can never be ACKd or failed.
    // Counting it in `unguaranteed` is what keeps sent - (ackd + failed + unguaranteed)
    // meaning "still in flight" instead of silently accruing a permanent residue.
    //
    // Gated on `track`, NOT on `ensured`: a best-effort UNICAST is tracked too
    // (see `track` above), so it can also be denied a slot — and then it is
    // transmitted once, untracked, and no ACK can ever match it. Gating on
    // `ensured` would credit its `sent` and leave it forever unresolvable.
    // A best-effort BROADCAST has track == false, never asks for a slot, and
    // credits no per-peer `sent` at all, so it is unaffected either way.
    const bool degraded = (slot < 0 && track);
    if (targetID == WCB_TARGET_BROADCAST) {
        _statBroadcastSent++;
        if (ensured) {
            for (int b = 0; b < WCB_MAX_BOARDS; b++) {
                // Mirror expected[] exactly when we got a slot; re-evaluate the
                // identical predicate when saturation denied us one. Same lock,
                // same instant — the two cannot disagree.
                bool aimed = (slot >= 0) ? _pending[slot].expected[b]
                                         : (_boards[b].online && (b + 1) != _deviceID);
                if (!aimed) continue;
                _stats[b].sent++;
                if (degraded) _stats[b].unguaranteed++;
            }
        }
    } else if (targetID >= 1 && targetID <= WCB_MAX_BOARDS) {
        _stats[targetID - 1].sent++;
        if (degraded) _stats[targetID - 1].unguaranteed++;
    }
    portEXIT_CRITICAL(&_pendingMux);

    if (slot < 0 && ensured) {
        // Pending table saturated with still-outstanding ensured deliveries. We
        // couldn't get a slot to track/retry this one, and must NOT evict a
        // guaranteed in-flight command to make room (see _findFreePending). Send
        // it best-effort once and report non-guaranteed delivery so the caller
        // can retry rather than assume ensured semantics it didn't get.
        _transmit(targetID, command, seq);
        return false;
    }

    return _transmit(targetID, command, seq);
}

// ─────────────────────────────────────────────────────────────────────────────
// _transmit
//
// Build a COMMAND packet with the GIVEN sequence number and hand it to ESP-NOW.
// No _seqCounter increment and no pending bookkeeping — this is the shared
// transmit used by both the initial send (_sendPacket) and ensured retransmits
// (update()), which must reuse the original seq so receivers dedup the resend
// and their ACKs still match the pending slot.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_transmit(uint8_t targetID, const char* command, uint16_t seqNum) {
    wcb_packet_etm_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "%d", targetID);
    pkt.structCommandIncluded = 1;

    // Append CRC32 checksum if enabled — must match ?ETM,CHKSM setting on WCBs.
    if (_checksumEnabled) {
        uint32_t crc = _crc32(command, strlen(command));
        snprintf(pkt.structCommand, sizeof(pkt.structCommand),
                 "%s|CRC%08X", command, crc);
    } else {
        strncpy(pkt.structCommand, command, sizeof(pkt.structCommand) - 1);
    }
    pkt.structCommand[sizeof(pkt.structCommand) - 1] = '\0';

    pkt.structPacketType     = WCB_PACKET_COMMAND;
    pkt.structSequenceNumber = seqNum;

    uint8_t* mac = (targetID == WCB_TARGET_BROADCAST)
                   ? _broadcastMAC
                   : _wcbMACs[targetID - 1];

    return _espSend(mac, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// _ensuredComplete
//
// An ensured packet is "done" once nothing in its expected set is still
// outstanding — i.e. every expected board has either ACK'd this sequence or
// gone offline (so we stop waiting on a board that's simply no longer there).
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_ensuredComplete(const WCBPending& p) const {
    for (int b = 0; b < WCB_MAX_BOARDS; b++) {
        if (!p.expected[b] || p.ackReceived[b]) continue;
        // Still outstanding if the board is online, OR it is a peer that has
        // NEVER been online (lastSeenMs==0) and still has retries left — e.g. a
        // learned client that only adverts (WDP) and never heartbeats. Without
        // this, an ensured send to such a target would look instantly complete
        // and never retry. A board that WAS online and then dropped
        // (lastSeenMs!=0, !online) is NOT outstanding — stop waiting on a board
        // that's simply gone.
        if (_boards[b].online ||
            (_boards[b].lastSeenMs == 0 && p.retryCount[b] < ETM_MAX_RETRIES))
            return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Delivery statistics
//
// Counters are accumulated at the pending table's existing hook points rather
// than derived from _pending[] on read: a slot is freed the instant its delivery
// resolves, so by the time anyone asked, the history would already be gone. The
// table holds WCB_PENDING_MAX (10) in-flight COMMANDS — one slot per command,
// NOT one per peer, so a single ensured broadcast to 20 boards occupies one slot
// and tracks all 20 ACKs inside it.
//
// EVERY helper here assumes _pendingMux is HELD by the caller (see the header):
// they run inside the critical sections that already serialise the pending table
// between the loop task and the ESP-NOW receive task. None of them may take the
// lock, print, or touch esp_now.
// ─────────────────────────────────────────────────────────────────────────────
bool WCB_Client::_statExpects(const WCBPending& p, uint8_t boardIdx) const {
    if (p.ensured) return p.expected[boardIdx];
    // A TRACKED best-effort send is always a unicast — best-effort broadcasts
    // are never given a slot at all (see _sendPacket) — and it is waiting on
    // exactly its own target.
    return p.targetID != WCB_TARGET_BROADCAST &&
           p.targetID >= 1 && (uint8_t)(p.targetID - 1) == boardIdx;
}

void WCB_Client::_statFail(uint8_t boardIdx) {
    if (boardIdx < WCB_MAX_BOARDS) _stats[boardIdx].failed++;
}

void WCB_Client::_settleSlot(const WCBPending& p) {
    if (!p.active) return;                     // already settled — never count twice
    for (int b = 0; b < WCB_MAX_BOARDS; b++)
        if (_statExpects(p, (uint8_t)b) && !p.ackReceived[b]) _statFail((uint8_t)b);
}

// ── Public accessors ─────────────────────────────────────────────────────────
// Unlocked by design — see the header for why adding a lock here would be a
// regression, and for the snapshot caveat.

WCBPeerStats WCB_Client::getPeerStats(uint8_t wcbID) const {
    if (wcbID < 1 || wcbID > WCB_MAX_BOARDS) return WCBPeerStats{};
    return _stats[wcbID - 1];
}

WCBPeerStats WCB_Client::getAggregateStats() const {
    WCBPeerStats total{};
    for (int b = 0; b < WCB_MAX_BOARDS; b++) {
        total.sent    += _stats[b].sent;
        total.ackd    += _stats[b].ackd;
        total.retries += _stats[b].retries;
        total.failed  += _stats[b].failed;
        total.unguaranteed  += _stats[b].unguaranteed;
    }
    return total;
}

uint32_t WCB_Client::getBroadcastSent() const { return _statBroadcastSent; }

void WCB_Client::resetStats() {
    // A WRITE, racing the RX task's ackd increment — so unlike the readers this
    // one DOES take the lock. Pure array work inside, as always.
    portENTER_CRITICAL(&_pendingMux);
    memset(_stats, 0, sizeof(_stats));
    _statBroadcastSent = 0;

    // Re-credit `sent` for every delivery STILL IN FLIGHT. A bare zeroing would
    // leave the active slots armed to deliver `ackd`/`failed` against a `sent`
    // of 0, permanently inverting ackd + failed + unguaranteed <= sent — and these
    // counters never wrap back, so that inversion would outlive by far the
    // traffic that caused it. Re-crediting exactly the outstanding expectations
    // makes a reset mean "start counting from here" rather than "corrupt the
    // books for whatever happened to be in the air".
    //
    // Slots are deliberately NOT settled or freed here: a reset is a diagnostic
    // action and must never drop a guaranteed command. This is the one place a
    // stats call reads the pending table for something other than counting,
    // which is why it runs under the same lock as everything else.
    //
    // Bounded at WCB_PENDING_MAX x WCB_MAX_BOARDS (10 x 20) iterations of pure
    // array work. That is longer than the other critical sections here, but a
    // reset is a rare operator action, not a per-packet path.
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        const WCBPending& p = _pending[i];
        if (!p.active) continue;
        for (int b = 0; b < WCB_MAX_BOARDS; b++)
            if (_statExpects(p, (uint8_t)b) && !p.ackReceived[b]) _stats[b].sent++;
    }
    portEXIT_CRITICAL(&_pendingMux);
}

// ─────────────────────────────────────────────────────────────────────────────
// _checkOfflineBoards
//
// Scans the board status table on every update() call. If a WCB's last
// heartbeat timestamp is older than (heartbeatIntervalSec * missedBeforeOffline)
// seconds it is marked offline and the status callback is fired.
//
// The threshold matches the WCB firmware's own ETM offline detection logic so
// both sides agree on when a board should be considered gone.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_checkOfflineBoards() {
    unsigned long now       = millis();
    unsigned long threshold = (unsigned long)_heartbeatIntervalSec
                              * _missedBeforeOffline * 1000UL;

    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        // Sweep floor peers (1..quantity) AND auto-joined learned peers; a learned
        // board above the floor must still transition offline when its heartbeats
        // stop, so ensured retries to it stop and the status callback fires.
        if (i >= _quantity && !_learnedPeer[i]) continue;
        if (!_boards[i].online) continue;
        if (now - _boards[i].lastSeenMs > threshold) {
            _boards[i].online = false;
            if (_statusCallback) _statusCallback(i + 1, false);
        }
    }

    // The special peer (NaviCore) sits outside 1..quantity, so the loop above
    // skips it — check it here so its ETM offline transition fires too (and so
    // ensured retries to it stop once it's gone).
    if (_specialPeerID > _quantity && _specialPeerID <= WCB_MAX_BOARDS) {
        int sp = _specialPeerID - 1;
        if (_boards[sp].online && (now - _boards[sp].lastSeenMs > threshold)) {
            _boards[sp].online = false;
            if (_statusCallback) _statusCallback(_specialPeerID, false);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _findFreePending
//
// Returns the index of a usable slot in _pending[], or -1 when the table is
// saturated with in-flight ensured deliveries. Prefers the first inactive slot.
// If all WCB_PENDING_MAX slots are occupied, reclaims only a slot that is SAFE
// to drop — a best-effort (un-ensured) slot, or an ensured slot that has already
// completed (every expected board acked or gone). A still-outstanding ensured
// slot is NEVER evicted, because dropping it would silently lose a guaranteed
// command that hasn't been delivered yet. Among reclaimable slots, the oldest is
// dropped. If every slot is an outstanding ensured delivery, returns -1 and the
// caller decides how to degrade (see _sendPacket).
//
// MUST be called with _pendingMux held (it is only invoked from _sendPacket,
// which takes the lock across the find + fill so the claim is atomic vs the
// cross-core ACK handler).
// ─────────────────────────────────────────────────────────────────────────────
int WCB_Client::_findFreePending() {
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        if (!_pending[i].active) return i;
    }
    // All slots busy. Reclaim the oldest slot that is safe to drop. Age is
    // (now - sentMs) with unsigned arithmetic, correct across a millis() rollover
    // (entries live for ms, far under the ~49.7-day wrap period).
    unsigned long now    = millis();
    int           victim = -1;
    unsigned long maxAge = 0;
    for (int i = 0; i < WCB_PENDING_MAX; i++) {
        bool reclaimable = !_pending[i].ensured || _ensuredComplete(_pending[i]);
        if (!reclaimable) continue;                 // outstanding ensured — must not drop
        unsigned long age = now - _pending[i].sentMs;
        if (victim < 0 || age > maxAge) { maxAge = age; victim = i; }
    }
    // STATS: an evicted slot is a delivery we stop waiting on — including a
    // best-effort unicast that is reclaimable at ANY age (see `reclaimable`
    // above), so one milliseconds old can be dropped well inside the 1 s window
    // the update() timeout treats as legitimate, and its later ACK will find no
    // active slot. Counting it here is the only way that becomes visible.
    if (victim >= 0) { _settleSlot(_pending[victim]); _pending[victim].active = false; }
    return victim;   // -1 => every slot is an outstanding ensured delivery
}
// ─────────────────────────────────────────────────────────────────────────────
// _maybeHandleMgmtFrag
//
// Reassemble an inbound MGMT fragment session and deliver it as an ordinary
// command. This is the RECEIVE half of the fragmentation send() has always had:
// send() falls through to _sendFragmented for anything over
// _maxSingleCommandLen(), but only the WCB FIRMWARE could rebuild it, so an
// oversized command from one WCB_Client host to another was transmitted
// perfectly and then silently dropped. With this, send() is symmetric between
// any two peers on the mesh.
//
// Shape of what arrives (wcb_packet_mgmt_t, 226 B, see WCB_Client.h):
//   - sent to the BROADCAST MAC with targetWCB addressing, so every host on the
//     network sees every session and must filter on targetWCB itself;
//   - there is no sender field in the struct, so the sender is mac[5] — safe
//     because _handleReceive has already proved octets 0-4 belong to this
//     network's scheme before we are called;
//   - chunk i occupies bytes [i*WCB_MGMT_CHUNK_LEN, +len) of the command, and
//     payload[180] is zero-filled by the sender so a short final chunk is
//     NUL-terminated;
//   - every chunk is re-sent across 2-3 passes, so duplicates are NORMAL, not a
//     fault. The mask dedups them and _mgmtDoneSession stops the completed set
//     from being delivered again on the following pass.
//
// Returns true when the packet was ours to consume (so the raw hook does not
// also see it), false to let it fall through unchanged.
bool WCB_Client::_maybeHandleMgmtFrag(const uint8_t* mac, const uint8_t* data, int len) {
    if (len != (int)sizeof(wcb_packet_mgmt_t)) return false;
    const wcb_packet_mgmt_t* p = (const wcb_packet_mgmt_t*)data;
    if (strncmp(p->structPassword, _password, sizeof(p->structPassword) - 1) != 0) return false;
    if (p->packetType != WCB_MGMT_PACKET_TYPE_FRAG &&
        p->packetType != WCB_MGMT_PACKET_TYPE_FRAG_UNICAST) return false;
    if (!_commandCallback) return false;          // nothing to deliver to — leave it alone
    if (p->targetWCB != _deviceID) return true;   // a session for someone else: consumed, not ours

    // BOUNDS BEFORE INDEXING. totalChunks == 0 would make the completion mask 0
    // and the session could never finish; an out-of-range chunkIdx would write
    // past _mgmtBuf. Both are attacker/corruption reachable, so they are checked
    // before either value is used for arithmetic.
    if (p->totalChunks == 0 || p->totalChunks > WCB_MGMT_MAX_CHUNKS) return true;
    if (p->chunkIdx >= p->totalChunks)                                return true;

    const uint8_t sender = mac[5];

    // Already delivered this exact session — this is one of the sender's repeat
    // passes. Swallow it silently.
    if (sender == _mgmtDoneSender && p->sessionId == _mgmtDoneSession) return true;

    // A different session (or a different sender) supersedes whatever was in
    // progress. Last writer wins: holding the old one would only let a peer that
    // went away block the buffer until the timeout.
    if (p->sessionId != _mgmtSession || sender != _mgmtSender) {
        _mgmtSession = p->sessionId;
        _mgmtSender  = sender;
        _mgmtTotal   = p->totalChunks;
        _mgmtMask    = 0;
        _mgmtLastLen = 0;
    }
    // totalChunks disagreeing mid-session means two senders picked the same
    // random id, or corruption. Restart on the new value rather than mixing.
    if (p->totalChunks != _mgmtTotal) { _mgmtTotal = p->totalChunks; _mgmtMask = 0; _mgmtLastLen = 0; }

    size_t n = strnlen(p->payload, WCB_MGMT_CHUNK_LEN);
    memcpy(_mgmtBuf + (size_t)p->chunkIdx * WCB_MGMT_CHUNK_LEN, p->payload, n);
    if (p->chunkIdx == _mgmtTotal - 1) _mgmtLastLen = n;   // only the LAST chunk may be short
    _mgmtMask   |= (uint16_t)(1u << p->chunkIdx);
    _mgmtDeadline = millis() + WCB_MGMT_RX_TIMEOUT_MS;

    const uint16_t full = (uint16_t)((1u << _mgmtTotal) - 1);
    if (_mgmtMask != full) return true;            // still filling

    // Complete. Length is exact: every chunk but the last is a full
    // WCB_MGMT_CHUNK_LEN by construction in _sendFragmented.
    const size_t total = (size_t)(_mgmtTotal - 1) * WCB_MGMT_CHUNK_LEN + _mgmtLastLen;
    _mgmtBuf[total] = '\0';
    // Latch "done" BEFORE dispatching: the callback can run for a while and the
    // sender's next pass is only ~10 ms behind, so the guard has to be armed
    // first or a repeat could re-enter and deliver twice.
    _mgmtDoneSender  = sender;
    _mgmtDoneSession = _mgmtSession;
    _mgmtSession     = 0xFFFF;
    _mgmtMask        = 0;
    _mgmtTotal       = 0;
    _mgmtDeadline    = 0;
    _commandCallback(sender, _mgmtBuf);
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// _handleReceive
//
// Central receive handler, called by the static ESP-NOW callback for every
// incoming packet. Routes by packet type:
//
//   HEARTBEAT : update the sender's online status and timestamp
//   ACK       : find the matching pending slot and mark it acknowledged
//   COMMAND   : verify checksum, send ACK back, deliver to user callback
//
// Packets are silently dropped if:
//   - They are shorter than wcb_packet_etm_t (incomplete packet)
//   - The structPassword doesn't match our network password
//   - They are not addressed to this device or broadcast
//   - The CRC32 is wrong or missing (when _checksumEnabled is true)
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_handleReceive(const uint8_t* mac, const uint8_t* data, int len) {

    // Network-namespace check FIRST — the sender's MAC must belong to THIS
    // network's octet scheme (02:oct2:oct3:00:00:ID, see _buildMACs). The radio
    // delivers ESP-NOW broadcasts/action-frames from foreign networks sharing the
    // channel regardless of their scoped FF:oct2:oct3:FF:FF:FF group, so without
    // this the password was the ONLY gate — and a mis-set local oct2/oct3 would
    // still "see" other networks' boards (false-positive online status) even
    // though it could never address them. Drop anything whose source octets
    // aren't ours. Done before the size routing below so it also gates raw
    // (non-etm) packets handed to _rawPacketCallback. (mac == info->src_addr.)
    if (!mac ||
        mac[0] != 0x02 || mac[1] != _oct2 || mac[2] != _oct3 ||
        mac[3] != 0x00 || mac[4] != 0x00)
        return;

    // Anything that is NOT the standard 252-byte WCB packet (e.g. an
    // application's OTA control/data structs, 55/243 B) goes to the raw-packet
    // hook, if registered, and stops here. This lets a custom protocol share the
    // mesh without forking the receive path. Undersized junk with no hook
    // registered is simply dropped (same as the previous `< sizeof` guard).
    if (len != (int)sizeof(wcb_packet_etm_t)) {
        // Channel sync: off the home channel nothing raw is ours (a search visit or a pull).
        if (_csOffHome) return;
        _csNoteBusy();   // OTA, config pulls, sequence replies: a search waits until they stop...
        { const unsigned long t = millis(); _csRawRxMs = t ? t : 1; }   // ...and so does a live switch (_csMoveBusy)
        // A stored-sequence inventory reply we asked for? Consume it here rather
        // than making every consumer hand-roll the reassembly. Only intercepts
        // while a request is actually in flight and the packet passes every check,
        // so a sketch that never calls requestSequenceNames() sees the raw hook
        // behave exactly as it always has.
        if (_maybeHandleSeqFrag(data, len)) return;
        // An oversized command someone send()-ed to US, arriving as MGMT
        // fragments. Rebuilt here and delivered as a normal command, so a
        // consumer never learns it was split. Checked before the raw hook so a
        // sketch using both still sees fragments handled, not handed over raw.
        if (_maybeHandleMgmtFrag(mac, data, len)) return;
        if (_rawPacketCallback) _rawPacketCallback(mac, data, len);
        return;
    }

    const wcb_packet_etm_t* pkt = (const wcb_packet_etm_t*)data;

    // Password check — reject packets from other WCB networks on the same channel
    if (strncmp(pkt->structPassword, _password, sizeof(pkt->structPassword) - 1) != 0)
        return;

    int senderID = atoi(pkt->structSenderID);
    int targetID = atoi(pkt->structTargetID);

    // Bind the claimed sender id to its real MAC. The octet check above already
    // proved octets 0-4; every WCB and WCB_Client forces its STA MAC's LAST octet
    // to its board number (02:oct2:oct3:00:00:<id>, see _buildMACs), so a packet
    // whose structSenderID disagrees with mac[5] is spoofed or misconfigured.
    // Drop it before it can mark a board online, satisfy an ACK, or auto-join.
    if (senderID >= 1 && senderID <= WCB_MAX_BOARDS && (uint8_t)senderID != mac[5]) return;

    // ── Channel sync (1.19.0) ───────────────────────────────────────────────
    // Off the home channel (a search visit or a pull) this device is a visitor: what
    // it hears there is another group's traffic, and marking that group online, ACKing
    // it or learning it would mislead every decision made once it is home. Only the
    // REPLY to its PROBE is wanted.
    const uint8_t ptype   = pkt->structPacketType;
    const bool    csFrame = ptype == chansync::PT_MOVE || ptype == chansync::PT_PROBE ||
                            ptype == chansync::PT_PROBE_REPLY;
    if (_csOffHome && ptype != chansync::PT_PROBE_REPLY) return;
    // Presence for channel sync: any frame from a bound sender, except a channel-sync
    // frame - a prober visiting from another channel sends only PROBEs here, and must
    // not count as a member of this group (its REPLYs would name it as the best here).
    if (!csFrame && senderID >= 1 && senderID <= WCB_MAX_BOARDS && senderID != _deviceID) {
        const unsigned long t = millis();
        _csHeardMs[senderID - 1] = t ? t : 1;
    }

    // Only process packets addressed to us or to everyone
    if (targetID != WCB_TARGET_BROADCAST && targetID != _deviceID) return;

    switch (pkt->structPacketType) {

        // ── Heartbeat ────────────────────────────────────────────────────────
        // Update the sender's last-seen timestamp. If the board was previously
        // offline, mark it online and fire the status callback.
        case WCB_PACKET_HEARTBEAT:
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS) {
                int  idx        = senderID - 1;
                // Only a sender _checkOfflineBoards() also sweeps (floor, learned, special peer) may
                // be marked online - otherwise nothing ever marks it offline again. A temporary
                // client above the floor heartbeats too; it is tracked through its WDP neighbour row
                // (getNeighbor, which ages out), not here. (HIL navicore.probe_temp_peer_not_learned.)
                if (!(idx < _quantity || _learnedPeer[idx] || senderID == _specialPeerID)) break;
                bool wasOffline = !_boards[idx].online;
                _boards[idx].online     = true;
                _boards[idx].lastSeenMs = millis();
                if (wasOffline && _statusCallback)
                    _statusCallback((uint8_t)senderID, true);
            }
            break;

        // ── Boot announce ────────────────────────────────────────────────────
        // A WCB sends a few of these right after it (re)starts, and its ETM
        // sequence counter restarts at 0. Our anti-replay window still held the
        // seqs it used before the reboot, so until its counter climbed back past
        // the old high, a new command whose seq landed on a remembered one was
        // dropped as a duplicate: some ;W<id> commands ran and others silently
        // did not. Forget the window, as the WCB firmware clears its own ring on
        // the same packet, and count the announce as presence like a heartbeat.
        // Clear once per boot: the three announces arrive ~1.2 s apart, and a
        // command the WCB sends between them whose ACK is lost is retried — a
        // second clear would let that retry run the command twice.
        // Same RX task as _cmdSeqDup, so the window stays lock-free.
        case WCB_PACKET_ETM_BOOT:
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS) {
                int           idx        = senderID - 1;
                bool          wasOffline = !_boards[idx].online;
                unsigned long now        = millis();
                // Mark it online only if _checkOfflineBoards() will also mark it OFFLINE again: floor
                // peers, learned peers and the special peer. Clients send boot announces too (so a
                // WCB clears their duplicate ring), and a TEMPORARY client above the floor is none of
                // those - marked online here, the sweep skipped it forever, so it stayed 'known'
                // after it left (HIL navicore.probe_temp_peer_not_learned, 2026-09-22). Its duplicate
                // ring is still cleared below.
                const bool tracked = idx < _quantity || _learnedPeer[idx] || senderID == _specialPeerID;
                if (tracked) {
                    _boards[idx].online     = true;
                    _boards[idx].lastSeenMs = now;
                } else {
                    wasOffline = false;   // no online transition to report for an untracked sender
                }
                if (!_boards[idx].seqResetMs || now - _boards[idx].seqResetMs > WCB_BOOT_RESET_HOLD_MS) {
                    _boards[idx].haveCmdSeq = false;
                    _boards[idx].cmdSeqMask = 0;
                    _boards[idx].seqResetMs = now ? now : 1;
                }
                if (wasOffline && _statusCallback)
                    _statusCallback((uint8_t)senderID, true);
            }
            break;

        // ── ACK ──────────────────────────────────────────────────────────────
        // Find the pending slot whose sequence number matches this ACK and
        // record that the sender has acknowledged it. For unicast packets the
        // slot is freed immediately; for broadcasts the slot stays open because
        // ACKs may arrive from multiple boards.
        case WCB_PACKET_ACK:
            // Any ACK proves a board heard something we sent: the transmit watchdog's best evidence.
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS) { unsigned long t = millis(); _txwdProofMs = t ? t : 1; }
            // Runs on Core 0 (WiFi/RX task). Serialize against the loop task's
            // slot claim+fill in _sendPacket so we never match/mutate a slot that
            // is being half-written (see _pendingMux). Pure array work — no
            // esp_now/Serial inside the critical section.
            portENTER_CRITICAL(&_pendingMux);
            for (int i = 0; i < WCB_PENDING_MAX; i++) {
                if (!_pending[i].active) continue;
                if (_pending[i].seqNum != pkt->structSequenceNumber) continue;

                if (senderID >= 1 && senderID <= WCB_MAX_BOARDS) {
                    // STATS: count the TRANSITION, not the packet. A peer re-ACKs
                    // every retransmit it hears — _sendAck() is the first thing
                    // the COMMAND case does, ahead of the CRC reject and the
                    // de-dup check — and an ensured BROADCAST slot deliberately
                    // stays open below to collect multi-board ACKs. So one board
                    // can ACK the same slot up to 1 + ETM_MAX_RETRIES = 4 times;
                    // incrementing next to the assignment would over-count 4x.
                    // The _statExpects gate additionally stops a late ACK from
                    // counting on top of a `failed` we already credited (giving
                    // up clears expected[], so the predicate goes false).
                    if (!_pending[i].ackReceived[senderID - 1] &&
                        _statExpects(_pending[i], (uint8_t)(senderID - 1)))
                        _stats[senderID - 1].ackd++;
                    _pending[i].ackReceived[senderID - 1] = true;
                }

                if (_pending[i].targetID != WCB_TARGET_BROADCAST) {
                    // STATS: the unicast slot closes here regardless of WHICH
                    // board ACKd (this free is not gated on senderID ==
                    // targetID), so settle it — if its own target never ACKd,
                    // that is a delivery we just stopped waiting on.
                    _settleSlot(_pending[i]);
                    _pending[i].active = false;
                }

                break;
            }
            portEXIT_CRITICAL(&_pendingMux);
            break;

        // ── Command ──────────────────────────────────────────────────────────
        // 1. Send an ACK so the sender stops retrying.
        // 2. Verify the CRC32 checksum (if enabled); reject on mismatch or missing.
        // 3. Strip the "|CRC..." suffix so the application sees a clean string.
        // 4. Deliver the command string to the user callback.
        case WCB_PACKET_COMMAND:

            // Channel sync waits for quiet - but best-effort JSON telemetry broadcast to
            // everyone is not traffic it must wait for (it never stops; see broadcast()).
            if (!(targetID == WCB_TARGET_BROADCAST && pkt->structCommand[0] == '{')) _csNoteBusy();

            // Always ACK, even if we end up rejecting the packet for a bad CRC.
            // This prevents the sender from flooding retries at us.
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS)
                _sendAck((uint8_t)senderID, pkt->structSequenceNumber);

            // A command from an authenticated sender means we may need to unicast
            // back to it — the ACK above, and any reply the callback sends. If it's
            // an ABOVE-FLOOR sender that isn't a registered peer yet (e.g. a client
            // / relay before auto-join's >=2-advert threshold), esp_now_send to it
            // fails (NOT_FOUND) and the reply is silently lost. Flag it (cheap bool,
            // same discipline as _pendingJoin) so update() registers it as a
            // transient peer on the LOOP task — NEVER add_peer on this RX callback
            // (the documented boot-loop hazard). Floor/special/self and already-
            // learned ids are skipped: they're registered already.
            if (senderID > _quantity && senderID <= WCB_MAX_BOARDS &&
                senderID != _deviceID && senderID != _specialPeerID &&
                senderID != WCB_SPECIAL_ID && !_learnedPeer[senderID - 1])
                _pendingReplyPeer[senderID - 1] = true;

            if (pkt->structCommandIncluded && _commandCallback) {
                char cmd[201];
                strncpy(cmd, pkt->structCommand, sizeof(cmd) - 1);
                cmd[sizeof(cmd) - 1] = '\0';

                // Only strip/verify a CRC suffix when checksums are ENABLED —
                // otherwise a legitimate payload that merely contains the literal
                // "|CRC" (e.g. a command-library or freetext command) would be
                // wrongly truncated or rejected. When enabled, the sender appends
                // the CRC as the tail "|CRCxxxxxxxx" (exactly 8 hex digits), so
                // anchor to the END of the string rather than the first match.
                if (_checksumEnabled) {
                    char*  crcTag = nullptr;
                    size_t len    = strlen(cmd);
                    if (len >= 12) {                        // "|CRC" + 8 hex digits
                        char* cand = cmd + len - 12;
                        if (strncmp(cand, "|CRC", 4) == 0) {
                            bool hex8 = true;
                            for (int h = 0; h < 8 && hex8; h++) {
                                char c = cand[4 + h];
                                if (!((c >= '0' && c <= '9') ||
                                      (c >= 'a' && c <= 'f') ||
                                      (c >= 'A' && c <= 'F'))) hex8 = false;
                            }
                            if (hex8) crcTag = cand;
                        }
                    }
                    if (!crcTag) {
                        // Expected a checksum but the tail isn't a valid one —
                        // likely a ?ETM,CHKSM mismatch between us and the sender.
                        Serial.printf("[WCB_Client] Missing CRC from WCB%d — packet rejected\n",
                                      senderID);
                        break;
                    }
                    uint32_t rxCRC   = (uint32_t)strtoul(crcTag + 4, nullptr, 16);
                    uint32_t calcCRC = _crc32(cmd, (size_t)(crcTag - cmd));
                    if (rxCRC != calcCRC) {
                        Serial.printf("[WCB_Client] CRC mismatch from WCB%d — packet rejected\n",
                                      senderID);
                        break;
                    }
                    *crcTag = '\0';  // Strip the verified "|CRC..." — deliver clean command
                }
                // checksum disabled → deliver exactly as received (no stripping)

                // ── Inbound de-dup: drop an exact ETM retransmit (same seq) so a
                // lost-ACK resend can't double-fire this command. ACK already sent.
                // seq 0 = un-sequenced (heartbeat-class) → never deduped.
                if (senderID >= 1 && senderID <= WCB_MAX_BOARDS &&
                    pkt->structSequenceNumber != 0 &&
                    _cmdSeqDup((uint8_t)senderID, pkt->structSequenceNumber))
                    break;

                // ── Bulk-transfer envelope (bb/bc/bd)? Parse + enqueue to the loop
                // task and DON'T deliver to the app callback. Gated on a registered
                // bulk sink so consumers that don't adopt it are unaffected.
                if (_bulkBeginCb && _maybeRouteBulk((uint8_t)senderID, cmd))
                    break;

                _commandCallback((uint8_t)senderID, cmd);
            }

            break;

        // ── WDP advert ─────────────────────────────────────────────────────────
        // A neighbor announcing itself. Decode its TLVs into the neighbor table
        // and fire onNeighbor. No ACK — adverts are fire-and-forget.
        case WCB_PACKET_WDP:
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS && pkt->structCommandIncluded)
                _handleWdpAdvert((uint8_t)senderID, (const uint8_t*)pkt->structCommand);
            break;

        // ── Channel sync (1.19.0): MOVE 25 / PROBE 26 / PROBE_REPLY 27 ────────
        // The sender is the frame's structSenderID, which the check above has bound to
        // the source MAC (mac[5]); only a bound id (1..WCB_MAX_BOARDS) is taken, so no
        // frame can speak for another board. COPY ONLY: the frame is judged, obeyed and
        // answered in update() (_csTick), never on this task - a MOVE switches the
        // radio and a PROBE is answered with a send, neither of which belongs here.
        // Released firmware and older libraries reach no case for 25-27 and drop them.
        case chansync::PT_MOVE:
        case chansync::PT_PROBE:
        case chansync::PT_PROBE_REPLY:
            if (senderID >= 1 && senderID <= WCB_MAX_BOARDS && senderID != _deviceID)
                _csRxFrame(ptype, (uint8_t)senderID, pkt->structCommand);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// _crc32
//
// CRC32 implementation that exactly matches the WCB firmware's calculateCRC32().
// The algorithm parameters must be identical or checksums will never verify:
//
//   Initial value : 0xFFFFFFFF
//   Polynomial    : 0xEDB88320  (bit-reversed / reflected CRC-32/ISO-HDLC)
//   Final XOR     : ~crc  (complement of the accumulated value)
//
// This is the same algorithm used by zlib, PNG, and most Ethernet stacks.
// ─────────────────────────────────────────────────────────────────────────────
uint32_t WCB_Client::_crc32(const char* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int j = 0; j < 8; j++) {
            // If LSB is set, shift right and XOR with polynomial; else just shift
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// _espNowReceiveCB  (static)
//
// ESP-NOW's receive callback must be a plain C function with no implicit 'this'
// pointer, so it is declared static. It simply delegates to the non-static
// _handleReceive() on the stored _instance pointer.
// ─────────────────────────────────────────────────────────────────────────────
void WCB_Client::_espNowReceiveCB(const esp_now_recv_info_t* info,
                                  const uint8_t* data, int len) {
    if (_instance) _instance->_handleReceive(info->src_addr, data, len);
}

// ─────────────────────────────────────────────────────────────────────────────
// Transmit watchdog (setTxWatchdog, 1.18.0) — see the header for the why.
// ─────────────────────────────────────────────────────────────────────────────

// Every frame this library sends goes through here: the watchdog counts what the radio refuses, and debugTxWedge()
// drops frames here the way a wedged radio would - "sent", never on the air. Called on the loop task and, for ACKs,
// the ESP-NOW receive task; the counter is a plain increment, so a lost count under a race is the worst case.
// Off the home channel (a channel-sync search) every frame but a channel-sync one is
// refused here, uncounted: on the channel being visited it would reach a group this
// device is not part of. An ensured send keeps its pending slot and is retried once home.
esp_err_t WCB_Client::_espSend(const uint8_t* mac, const uint8_t* data, size_t len) {
    if (_csOffHome) return ESP_ERR_ESPNOW_IF;
    return _espSendRaw(mac, data, len);
}

esp_err_t WCB_Client::_espSendRaw(const uint8_t* mac, const uint8_t* data, size_t len) {
    if (_txWedge) return ESP_OK;
    const esp_err_t e = esp_now_send(mac, data, len);
    if (e != ESP_OK) _txSendErrors++;
    return e;
}

void WCB_Client::setTxWatchdog(bool enabled, uint32_t probeIntervalMs) {
    if (probeIntervalMs < 5000UL)   probeIntervalMs = 5000UL;
    if (probeIntervalMs > 600000UL) probeIntervalMs = 600000UL;
    _txwdEnabled = enabled;
    _txwdProbeMs = probeIntervalMs;
    _txwdSentMs  = 0;
    _txwdMisses  = 0;
    _txwdGood    = 0;
    _txwdRecoveredMs = 0;
    _txwdNextMs  = millis() + probeIntervalMs;   // no verdict before the boards have had an interval to be heard
    if (!enabled) { _txwdLevel = 0; _txwdStallMs = 0; }
    Serial.printf("[WCB_Client] transmit watchdog %s (a WDP solicit every %lu s while boards are heard)\n",
                  enabled ? "on" : "off", (unsigned long)(probeIntervalMs / 1000UL));
}

void WCB_Client::onTxStall(WCBTxStallCallback callback) { _txStallCb = callback; }

WCBTxWatchStatus WCB_Client::getTxWatchStatus() const {
    WCBTxWatchStatus st;
    st.enabled      = _txwdEnabled;
    st.level        = _txwdLevel;
    st.misses       = _txwdMisses;
    st.probes       = _txwdProbes;
    st.restarts     = _txwdRestarts;
    st.sendErrors   = _txSendErrors;
    const unsigned long proof = _txwdProofMs;
    st.sinceProofMs = proof ? (uint32_t)(millis() - proof) : 0xFFFFFFFFUL;
    return st;
}

void WCB_Client::debugTxWedge(uint8_t mode) {
    _txWedge = mode <= 2 ? mode : 0;
    Serial.printf("[WCB_Client] debugTxWedge(%u): %s\n", _txWedge,
                  _txWedge == 1 ? "every frame dropped before the radio until the next ESP-NOW restart" :
                  _txWedge == 2 ? "every frame dropped before the radio until reboot" : "off");
}

// The boards that can judge whether the mesh hears us: heard within WCB_TXWD_HEARD_MS (so they are there and on our
// channel) and known to advertise (so they answer a solicit). None -> no verdict, never a false stall.
uint8_t WCB_Client::_txJudges(unsigned long now) const {
    uint8_t n = 0;
    for (int b = 0; b < WCB_MAX_BOARDS; b++) {
        if ((b + 1) == _deviceID) continue;
        if (!_boards[b].online || !_neighbors[b].valid) continue;
        if (now - _boards[b].lastSeenMs < WCB_TXWD_HEARD_MS) n++;
    }
    return n;
}

// The same bare SOLICIT the firmware's ?WDP,POLL sends (WCB_WDP.cpp wdpSendSolicit): magic, proto, SOLICIT len 0, END.
void WCB_Client::_sendWdpSolicit() {
    wcb_packet_etm_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "0");   // 0 = broadcast
    pkt.structCommandIncluded = 1;
    uint8_t* p = (uint8_t*)pkt.structCommand;
    p[0] = WCB_WDP_MAGIC; p[1] = WCB_WDP_PROTO; p[2] = WCB_WDP_TLV_SOLICIT; p[3] = 0; p[4] = WCB_WDP_TLV_END;
    pkt.structPacketType     = WCB_PACKET_WDP;
    pkt.structSequenceNumber = 0;
    _espSend(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt));
}

void WCB_Client::_txWatchAnswered(unsigned long now) {
    if (_txwdLevel) _txwdRecoveredMs = now ? now : 1;
    if (_txwdLevel) {
        const uint32_t was = _txwdStallMs ? (uint32_t)(now - _txwdStallMs) : 0;
        Serial.printf("[WCB_Client] mesh transmit recovered: the boards answer again after %lu s unanswered\n",
                      (unsigned long)(was / 1000UL));
        _txwdLevel = 0;
        if (_txStallCb) _txStallCb(0, was);
    }
    _txwdMisses  = 0;
    _txwdStallMs = 0;
    _txwdProofMs = now ? now : 1;   // the last time the mesh was shown to hear us (getTxWatchStatus, next probe)
}

// Raise the level, say what the radio holds - the evidence a wedge leaves - and act on it.
void WCB_Client::_txWatchLevel(uint8_t level, unsigned long now) {
    _txwdLevel = level;
    const uint32_t stalled = (uint32_t)(now - _txwdStallMs);
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    const bool macOk = memcmp(mac, _wcbMACs[_deviceID - 1], 6) == 0;
    uint8_t ch = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&ch, &second);
    Serial.printf("[WCB_Client] mesh transmit %s: %d WDP solicit(s) in a row unanswered for %lu s while boards are "
                  "heard (STA MAC %02X:%02X:%02X:%02X:%02X:%02X %s, channel %u of mesh %u, send errors %lu, internal "
                  "heap free %u largest %u min %u)%s\n",
                  level == 1 ? "stalled" : "still stalled", WCB_TXWD_MISSES, (unsigned long)(stalled / 1000UL),
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], macOk ? "as set" : "NOT the mesh MAC", ch,
                  _meshChannel, (unsigned long)_txSendErrors,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                  " - restarting ESP-NOW");
    _restartEspNow();
    if (_txStallCb) _txStallCb(level, stalled);
}

// ESP-NOW from scratch, on the loop task: the mesh MAC put back if the radio lost it, the driver re-initialised and
// every peer registered again (floor, broadcast, special, learned - from RAM, no NVS write). The receive callback is
// re-registered; nothing else this library holds depends on the driver's state.
void WCB_Client::_restartEspNow() {
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (memcmp(mac, _wcbMACs[_deviceID - 1], 6) != 0) {
        const esp_err_t me = esp_wifi_set_mac(WIFI_IF_STA, _wcbMACs[_deviceID - 1]);
        Serial.printf("[WCB_Client] mesh MAC put back on the STA interface: %s\n", esp_err_to_name(me));
    }
    esp_now_deinit();
    const esp_err_t e = esp_now_init();
    if (e != ESP_OK) {
        Serial.printf("[WCB_Client] ESP-NOW restart failed: esp_now_init() %s - the next stall tries again\n",
                      esp_err_to_name(e));
        return;
    }
    esp_now_register_recv_cb(_espNowReceiveCB);
    _registerPeers();
    for (int i = 0; i < WCB_MAX_BOARDS; i++) {
        if (!_learnedPeer[i] || esp_now_is_peer_exist(_wcbMACs[i])) continue;
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _wcbMACs[i], 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
    if (_txWedge == 1) _txWedge = 0;   // debugTxWedge(1) is the wedge a restart clears
    _txwdRestarts++;
    _checkMeshChannel();
}

void WCB_Client::_txWatchTick(unsigned long now) {
    if (!_txwdEnabled) return;

    // A probe is out: judge it once its answer window has passed.
    if (_txwdSentMs) {
        if (now - _txwdSentMs < WCB_TXWD_ANSWER_MS) return;
        const unsigned long sent = _txwdSentMs;
        _txwdSentMs = 0;
        // Answered: an ACK to anything we sent (no coincidence makes one), or adverts after the solicit from enough
        // boards - two when two or more could answer, so one board's own periodic advert is not taken for an answer.
        const unsigned long proof = _txwdProofMs;
        int adverts = 0;
        for (int b = 0; b < WCB_MAX_BOARDS; b++) {
            if ((b + 1) == _deviceID || !_neighbors[b].valid) continue;
            if ((long)(_neighbors[b].lastSeenMs - sent) >= 0) adverts++;
        }
        const bool answered = (proof && (long)(proof - sent) >= 0) || adverts >= (_txwdJudges >= 2 ? 2 : 1);
        if (answered) {
            _txwdMisses = 0;
            if (_txwdLevel && ++_txwdGood < WCB_TXWD_RECOVER) {   // a stall ends on answers in a row, not one
                _txwdNextMs = now + WCB_TXWD_RETRY_MS;
                return;
            }
            _txwdGood = 0;
            _txWatchAnswered(now);
            _txwdNextMs = now + _txwdProbeMs;
            return;
        }
        _txwdGood = 0;
        if (!_txJudges(now)) {                // everyone left during the wait: no verdict
            _txwdNextMs = now + _txwdProbeMs;
            return;
        }
        if (!_txwdStallMs) _txwdStallMs = sent;
        if (++_txwdMisses >= WCB_TXWD_MISSES) {
            _txwdMisses = 0;
            // A stall soon after one ended: the ESP-NOW restart did not hold, so do not offer it again first.
            const bool relapse = _txwdLevel == 0 && _txwdRecoveredMs && now - _txwdRecoveredMs < WCB_TXWD_RELAPSE_MS;
            _txWatchLevel((_txwdLevel == 0 && !relapse) ? 1 : 2, now);
        }
        // At level 2 the library has done what it can: ask at the normal pace, not every few seconds.
        _txwdNextMs = now + (_txwdLevel >= 2 ? _txwdProbeMs : WCB_TXWD_RETRY_MS);
        return;
    }

    if ((long)(now - _txwdNextMs) < 0) return;
    // An ACK within the interval already proves the mesh hears us (only while nothing is wrong).
    const unsigned long proof = _txwdProofMs;
    if (_txwdLevel == 0 && !_txwdMisses && proof && now - proof < _txwdProbeMs) {
        _txwdNextMs = proof + _txwdProbeMs;
        return;
    }
    const uint8_t judges = _txJudges(now);
    if (!judges) {                            // nobody to ask
        _txwdMisses = 0;
        _txwdNextMs = now + _txwdProbeMs;
        return;
    }
    _txwdJudges = judges;
    _sendWdpSolicit();
    _txwdSentMs = now ? now : 1;
    _txwdProbes++;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Channel sync (1.19.0) — see the header, and the WCB repo's docs/CHANNEL_SYNC.md
//  ---------------------------------------------------------------------------
//  Every ordering and verdict comes from WCB_ChanSyncWire.h (chansync::better,
//  judgeMove, shouldSearch, judgeSearch), the header the WCB firmware compiles, so
//  a client and a board judge the same frame the same way. What lives here is only
//  the plumbing: who is heard, when to act, and the radio.
//
//  The RX task copies (_csRxFrame); everything else runs on the loop task from
//  _csTick(), which update() calls first.
// ═════════════════════════════════════════════════════════════════════════════

// The regions esp_wifi_set_country_code() accepts (esp_wifi.h, IDF 5.5).
static const char kCsRegions[][3] = {
    "01", "AT", "AU", "BE", "BG", "BR", "CA", "CH", "CN", "CY", "CZ", "DE", "DK", "EE", "ES", "FI",
    "FR", "GB", "GR", "HK", "HR", "HU", "IE", "IN", "IS", "IT", "JP", "KR", "LI", "LT", "LU", "LV",
    "MT", "MX", "NL", "NO", "NZ", "PL", "PT", "RO", "SE", "SI", "SK", "TW", "US"
};

static bool csRegionValid(const char* cc) {
    for (size_t i = 0; i < sizeof(kCsRegions) / sizeof(kCsRegions[0]); i++)
        if (cc[0] == kCsRegions[i][0] && cc[1] == kCsRegions[i][1] && cc[2] == '\0') return true;
    return false;
}

static const char* csVerdictText(chansync::MoveVerdict v) {
    switch (v) {
        case chansync::MV_SYNC_OFF:     return "channel sync is off here";
        case chansync::MV_BAD_CHANNEL:  return "that channel is outside this device's region";
        case chansync::MV_SAME_CHANNEL: return "already on that channel";
        case chansync::MV_PINNED_SELF:  return "this device is joined to a network";
        case chansync::MV_OUTRANKED:    return "a device this one hears outranks the sender";
        default:                        return "accepted";
    }
}

void WCB_Client::setChanSyncRank(uint8_t rank) {
    _csRank    = rank;
    _csRankSet = true;
    // The rank is in every advert (TLV 0x14): after begin(), push the new key out now.
    if (_started && _wdpType[0]) { _wdpBootLeft = 2; _wdpNextBootMs = millis() + 200; }
}

uint8_t WCB_Client::chanSyncRank() const {
    return _csRankSet ? _csRank : (uint8_t)(chansync::RANK_CLIENT_BASE + _deviceID);
}

// Only the flag here: this may be called from a command callback on the RX task, and
// ending a search moves the radio. _csTick() unwinds a move or a search on the loop task.
void WCB_Client::setChanSyncEnabled(bool on) {
    if (on == _csEnabled) return;
    _csEnabled = on;
    Serial.printf("[WCB_Client] channel sync %s\n",
                  on ? "on" : "off - this device will not move, search or answer a search");
    if (_started && _wdpType[0]) { _wdpBootLeft = 2; _wdpNextBootMs = millis() + 200; }   // SYNC flag in the TLV
}

void WCB_Client::setRegion(const char* cc) {
    char up[3] = { 0, 0, 0 };
    if (cc && cc[0] && cc[1] && !cc[2]) {
        up[0] = (char)toupper((unsigned char)cc[0]);
        up[1] = (char)toupper((unsigned char)cc[1]);
    }
    if (!csRegionValid(up)) {
        Serial.printf("[WCB_Client] setRegion: \"%s\" is not a supported region - keeping \"%s\"\n",
                      cc ? cc : "", _csRegion);
        return;
    }
    memcpy(_csRegion, up, sizeof(_csRegion));
    // WiFi already started (a SoftAP raised before begin()): apply it now, so the next
    // softAP() on channel 12-13 is legal. begin() applies it again (idempotent).
    if (WiFi.getMode() != WIFI_MODE_NULL) {
        _csApplyRegion();
        _csMaxCh = _csReadMaxCh();
    } else {
        _csMaxCh = strcmp(_csRegion, "01") == 0 ? 11 : chansync::MAX_CHANNEL_CAP;
    }
}

// "01" is the IDF default, so it makes no call and a sketch that never sets a region
// runs exactly as before. Except: the IDF keeps the country in flash (esp_wifi.h,
// esp_wifi_set_country_code attention 4) and the Arduino core starts WiFi with flash
// storage on, so a device once set to "DE" comes up as "DE" with no call at all - and a
// later "01" would silently keep 12-13 legal here while the rest of the mesh cannot use
// them. Only then is "01" written back, with 802.11d on, as the IDF ships it.
// Any other region: esp_wifi_set_country_code(cc, false) - 802.11d off, so a STA
// association cannot swap in its access point's region and strand the mesh.
void WCB_Client::_csApplyRegion() {
    char cur[4] = { 0, 0, 0, 0 };
    const bool have = esp_wifi_get_country_code(cur) == ESP_OK;
    if (strcmp(_csRegion, "01") == 0) {
        if (!have || (cur[0] == '0' && cur[1] == '1')) return;
        const esp_err_t e = esp_wifi_set_country_code("01", true);
        Serial.printf("[WCB_Client] region: the WiFi driver restored \"%c%c\" from flash - back to \"01\" "
                      "(channels 1-11): %s\n", cur[0] ? cur[0] : '?', cur[1] ? cur[1] : '?', esp_err_to_name(e));
        return;
    }
    const esp_err_t e = esp_wifi_set_country_code(_csRegion, false);
    if (e != ESP_OK)
        Serial.printf("[WCB_Client] WARNING: region \"%s\" not applied (%s) - the radio keeps its region\n",
                      _csRegion, esp_err_to_name(e));
}

uint8_t WCB_Client::_csReadMaxCh() const {
    wifi_country_t c;
    memset(&c, 0, sizeof(c));
    if (esp_wifi_get_country(&c) != ESP_OK || c.nchan == 0)
        return strcmp(_csRegion, "01") == 0 ? 11 : chansync::MAX_CHANNEL_CAP;
    int top = (int)c.schan + (int)c.nchan - 1;
    if (top > chansync::MAX_CHANNEL_CAP) top = chansync::MAX_CHANNEL_CAP;   // JP's 14 is never used
    if (top < 1) top = 11;
    return (uint8_t)top;
}

uint8_t WCB_Client::maxChannel() const { return _csMaxCh; }

uint8_t WCB_Client::currentChannel() const {
    if (_csSearch != CS_SEARCH_IDLE && _csHomeCh) return _csHomeCh;   // hopping: where it returns to
    if (_started) {
        uint8_t p = 0;
        wifi_second_chan_t s = WIFI_SECOND_CHAN_NONE;
        if (esp_wifi_get_channel(&p, &s) == ESP_OK && p) return p;
    }
    return _meshChannel;
}

void WCB_Client::chanSyncSearchNow() {
    if (!_started || !_csEnabled) {
        Serial.printf("[WCB_Client] channel sync: no search - %s\n",
                      !_started ? "not started (call begin() first)" : "channel sync is off");
        return;
    }
    // The boot grace holds a requested search too, as the firmware's does: until the boot
    // SOLICIT's adverts and the heartbeats are in, every key is derived and this device
    // takes itself for its group's leader.
    const unsigned long t0 = millis();
    if (t0 - _csBootMs < chansync::BOOT_GRACE_MS || _csHostMoveCh) {
        Serial.printf("[WCB_Client] channel sync: no search - %s\n",
                      _csHostMoveCh ? "the host is still applying a move" :
                                      "the boot grace (40 s after begin(): who leads is still settling)");
        return;
    }
    // Only a group's leader searches, by request too: judgeSearch weighs what it finds
    // against the searcher, so a follower would take itself off to a group its own
    // leader outranks - and its group, judging that MOVE, would rightly stay behind.
    const chansync::Key self = _csSelfKey(), lead = _csBestPresent(t0);
    if (lead.id != self.id) {
        Serial.printf("[WCB_Client] channel sync: no search - WCB%u (rank %u) leads this group; a search is its job\n",
                      lead.id, lead.rank);
        return;
    }
    _csSearchNowReq = true;
    const unsigned long t = millis();
    _csSearchNowMs = t ? t : 1;
}

bool WCB_Client::chanSyncMove(uint8_t channel, String* why) {
    char reason[160];
    reason[0] = '\0';
    const unsigned long now = millis();
    const char* w = nullptr;
    if (!_started)                                          snprintf(reason, sizeof(reason), "not started (call begin() first)");
    else if (!_csEnabled)                                   snprintf(reason, sizeof(reason), "channel sync is off");
    else if (!chansync::validChannel(channel, _csMaxCh))    snprintf(reason, sizeof(reason), "channel %u is outside 1-%u (region \"%s\")",
                                                                     channel, _csMaxCh, _csRegion);
    else if (channel == currentChannel())                   snprintf(reason, sizeof(reason), "the mesh is already on channel %u", channel);
    else if (_csSearch != CS_SEARCH_IDLE || _csMovePending || _csAnnLeft || _csHostMoveCh)
                                                            snprintf(reason, sizeof(reason), "a move or search is already running");
    else if (!_csViewSettled(now))                          snprintf(reason, sizeof(reason), "the neighbors' channel facts are still "
                                                                     "arriving (%lu s after begin()) - a pinned device may not be known yet",
                                                                     (unsigned long)(WCB_CHSYNC_VIEW_SETTLE_MS / 1000UL));
    else if (_csIfaceUp(&w) && !_csMoveCb)                  snprintf(reason, sizeof(reason), "%s owns this device's channel and no "
                                                                     "onChannelMove() handler applies a move, so it could not follow", w);
    else {
        // A pinned device stays: the mesh would go without it, and its leader's search
        // would bring everyone back.
        for (uint8_t id = 1; id <= WCB_MAX_BOARDS && !reason[0]; id++) {
            if (id == _deviceID || !_csHeard(id, now)) continue;
            portENTER_CRITICAL(&_csMux);
            const bool pinned = _csPeer[id - 1].has && (_csPeer[id - 1].f.flags & chansync::F_PINNED);
            const uint8_t pch = _csPeer[id - 1].f.channel;
            portEXIT_CRITICAL(&_csMux);
            if (pinned)
                snprintf(reason, sizeof(reason), "WCB%u is joined to a network on channel %u and cannot move", id, pch);
        }
    }
    if (reason[0]) {
        Serial.printf("[WCB_Client] channel sync: move to channel %u refused - %s\n", channel, reason);
        if (why) *why = reason;
        return false;
    }
    const chansync::Key self = _csSelfKey();
    _csStartFrames(channel, (uint8_t)(self.flags | chansync::MF_OPERATOR), chansync::MOVE_REPEATS,
                   chansync::MOVE_REPEAT_GAP_MS, chansync::MOVE_COUNTDOWN_MS, now);
    _csMovePending = true;
    _csMoveCh      = channel;
    _csMoveAtMs    = _csAnnSwitchMs;
    Serial.printf("[WCB_Client] channel sync: operator move - the mesh moves to channel %u in %u ms\n",
                  channel, (unsigned)chansync::MOVE_COUNTDOWN_MS);
    if (why) *why = "";
    return true;
}

void WCB_Client::printChanSyncStatus(Print& out) {
    const unsigned long now  = millis();
    const chansync::Key self = _csSelfKey();
    const chansync::Key lead = _csBestPresent(now);
    const char* state = _csSearch != CS_SEARCH_IDLE ? "search" :
                        (_csMovePending || _csAnnLeft || _csHostMoveCh) ? "move" : "idle";
    const uint8_t ch  = currentChannel();
    out.printf("Channel sync: %s - channel %u, region %s (channels 1-%u), rank %u%s\n",
               _csEnabled ? "on" : "off", ch, _csRegion, _csMaxCh, self.rank,
               self.rank == chansync::RANK_NEVER ? " (never leads)" : "");
    if (lead.id == self.id) out.println("  leader: this device");
    else out.printf("  leader: WCB%u (rank %u%s)\n", lead.id, lead.rank,
                    (lead.flags & chansync::F_PINNED) ? ", joined to a network" : "");
    if (_csMovePending) {
        const long left = (long)(_csMoveAtMs - now);
        out.printf("  moving to channel %u in %ld ms%s\n", _csMoveCh, left > 0 ? left : 0L,
                   _csDeferSinceMs ? " (waiting for a session the switch would cut)" : "");
    }
    if (_csHostMoveCh)
        out.printf("  the host is applying the move to channel %u (%lu s ago) - it has not restarted yet\n",
                   _csHostMoveCh, (unsigned long)((now - _csHostMoveMs) / 1000UL));
    uint8_t who = 0;
    if (_csMemberMissing(now, &who))
        out.printf("  missing: WCB%u%s\n", who,
                   _csSched ? "" : " (a search needs 10 s of this, 40 s after boot)");
    out.printf("  searches: %lu since boot; %u of %u attempts used on the current trigger",
               (unsigned long)_csSearches, _csAttempt, (unsigned)chansync::SEARCH_ATTEMPTS);
    if (_csSched && _csAttempt < chansync::SEARCH_ATTEMPTS) {
        const long due = (long)(_csNextDueMs - now);
        out.printf("; the next is due in %ld s", due > 0 ? due / 1000L : 0L);
    }
    out.println();
    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        if (id == _deviceID || !_csHeard(id, now)) continue;
        portENTER_CRITICAL(&_csMux);
        const bool has = _csPeer[id - 1].has;
        const chansync::ChanFacts f = _csPeer[id - 1].f;
        portEXIT_CRITICAL(&_csMux);
        if (has)
            out.printf("  WCB%u: rank %u, channel %u, region %s%s%s\n", id, f.rank, f.channel, f.cc,
                       (f.flags & chansync::F_PINNED) ? ", joined to a network" : "",
                       (f.flags & chansync::F_SYNC) ? "" : ", sync off");
        else
            out.printf("  WCB%u: rank %u (derived - no channel sync on that device)\n", id, _csKeyOf(id).rank);
    }
    out.printf("[CHSYNC:EN=%d,CH=%u,MAXCH=%u,CC=%s,RANK=%u,PIN=0,LEAD=%u,LRANK=%u,LPIN=%d,STATE=%s]\n",
               _csEnabled ? 1 : 0, ch, _csMaxCh, _csRegion, self.rank, lead.id, lead.rank,
               (lead.flags & chansync::F_PINNED) ? 1 : 0, state);
}

// begin(): a fresh start. The neighbor's CHAN facts (_csPeer) are kept, like the
// neighbor table they mirror.
void WCB_Client::_csReset() {
    const unsigned long t = millis();
    _csBootMs = t;
    for (int i = 0; i < WCB_MAX_BOARDS; i++) _csHeardMs[i] = 0;
    portENTER_CRITICAL(&_csMux);
    _csHead = _csTail = 0;
    portEXIT_CRITICAL(&_csMux);
    _csOffHome      = false;
    _csBusyMs       = 0;
    _csRawRxMs      = 0;
    _csBootSolicited = false;
    _csMovePending  = false;
    _csDeferSinceMs = 0;
    _csHostMoveCh   = 0;
    _csHeldValid    = false;
    _csAnnLeft      = 0;
    _csReplyPending = false;
    _csSeenSender   = 0;
    _csSeenMoveId   = 0;
    _csSearch       = CS_SEARCH_IDLE;
    _csHomeCh       = 0;
    _csVisitCh      = 0;
    _csSearchNowReq = false;
    _csResetTrigger();
    _csLastEvalMs   = 0;
    for (int c = 0; c <= chansync::MAX_CHANNEL_CAP; c++) _csFound[c] = chansync::noKey();
    memset(_csPullMask, 0, sizeof(_csPullMask));
}

// RX task. Copy the frame and its arrival time (a MOVE's countdown and a REPLY's jitter
// run from it), nothing else. A full ring drops the frame: a MOVE is sent 5 times, and a
// lost PROBE or REPLY costs one channel of one search.
void WCB_Client::_csRxFrame(uint8_t type, uint8_t sender, const char* payload) {
    CsFrame f;
    f.type   = type;
    f.sender = sender;
    memcpy(f.data, payload, sizeof(f.data));
    f.rxMs   = millis();
    portENTER_CRITICAL(&_csMux);
    const uint8_t next = (uint8_t)((_csHead + 1) % CS_RING);
    if (next != _csTail) { _csRing[_csHead] = f; _csHead = next; }
    portEXIT_CRITICAL(&_csMux);
}

bool WCB_Client::_csPop(CsFrame& out) {
    bool got = false;
    portENTER_CRITICAL(&_csMux);
    if (_csTail != _csHead) { out = _csRing[_csTail]; _csTail = (uint8_t)((_csTail + 1) % CS_RING); got = true; }
    portEXIT_CRITICAL(&_csMux);
    return got;
}

chansync::Key WCB_Client::_csSelfKey() const {
    chansync::Key k;
    k.id    = _deviceID;
    k.rank  = chanSyncRank();
    k.flags = _csEnabled ? chansync::F_SYNC : 0;   // never F_PINNED: a client does not join a network
    return k;
}

// A neighbor's key: from its latest CHAN TLV, or - for firmware older than channel sync -
// derived as docs/CHANNEL_SYNC.md §2 says: a WCB's rank is its number, a client's
// 100 + id, a temporary peer never leads; unpinned, sync off (the firmware's keyOf). With no
// advert at all there is no telling a client from a WCB, except at the controller slot.
chansync::Key WCB_Client::_csKeyOf(uint8_t id) const {
    chansync::Key k;
    k.id = id;
    portENTER_CRITICAL(&_csMux);
    const bool has = _csPeer[id - 1].has;
    const chansync::ChanFacts f = _csPeer[id - 1].f;
    portEXIT_CRITICAL(&_csMux);
    if (has) {
        k.rank  = f.rank;
        k.flags = (uint8_t)(f.flags & (chansync::F_PINNED | chansync::F_SYNC));
        return k;
    }
    const WCBNeighbor& nb = _neighbors[id - 1];
    const bool temporary = (nb.valid && nb.temporary) || (_csNeverMiss & (1UL << (id - 1)));
    const bool client    = (nb.valid && nb.isClient) || id == _specialPeerID || id == WCB_SPECIAL_ID;
    k.rank  = temporary ? chansync::RANK_NEVER : (client ? (uint8_t)(chansync::RANK_CLIENT_BASE + id) : id);
    k.flags = 0;
    return k;
}

// Heard within the offline window (heartbeat interval x missed beats, 50 s), by any frame
// but a channel-sync one. The same window the heartbeat sweep uses, so a device a search
// is looking for (offline by the sweep) is no longer counted as present either.
bool WCB_Client::_csHeard(uint8_t id, unsigned long now) const {
    const unsigned long t = _csHeardMs[id - 1];
    const unsigned long window = (unsigned long)_heartbeatIntervalSec * _missedBeforeOffline * 1000UL;
    return t != 0 && (now - t) < window;
}

// The best device this one hears, ITSELF INCLUDED (what judgeMove and shouldSearch take,
// what a REPLY names, and who the leader checks compare with) - among those that TAKE PART
// (F_SYNC), exactly as the WCB firmware's look() picks it (WCB_ChanSync.cpp). A device with
// sync off, or firmware older than channel sync (a derived key, SYNC off), cannot move or
// search: a group led by it would never look for a stray board and would refuse every MOVE
// but its own. Counting it here split mixed fleets - this device deferring to a leader the
// boards themselves do not have. It is still heard (the pinned check in chanSyncMove).
chansync::Key WCB_Client::_csBestPresent(unsigned long now, uint8_t exclude) const {
    chansync::Key best = _csSelfKey();
    for (uint8_t id = 1; id <= WCB_MAX_BOARDS; id++) {
        if (id == _deviceID || id == exclude || !_csHeard(id, now)) continue;
        const chansync::Key k = _csKeyOf(id);
        if ((k.flags & chansync::F_SYNC) && chansync::better(k, best)) best = k;
    }
    return best;
}

// A board this device knows of is missing: a member of its peer set - the floor
// (1..wcb_quantity), a learned peer, the special peer - that the heartbeat sweep has
// marked offline (or that has not been heard since boot). Temporary peers and rank-255
// devices are never missed: nobody searches for a relay. That is _csNeverMiss, not the
// neighbor record: the record that says "temporary" ages out (50 s / 180 s) exactly when
// the device has gone, and after a reboot it is not there at all, so a relay switched off
// at a floor id would cost seven searches after every boot.
bool WCB_Client::_csMemberMissing(unsigned long now, uint8_t* who) const {
    (void)now;
    const uint32_t never = _csNeverMiss;
    for (uint8_t i = 0; i < WCB_MAX_BOARDS; i++) {
        const uint8_t id = (uint8_t)(i + 1);
        if (id == _deviceID) continue;
        const bool member = i < _quantity || _learnedPeer[i] || (_specialPeerID && id == _specialPeerID);
        if (!member) continue;
        if (never & (1UL << i)) continue;
        if (_neighbors[i].valid && _neighbors[i].temporary) continue;   // (the RX task sets the bit with it)
        if (!_boards[i].online) { if (who) *who = id; return true; }
    }
    return false;
}

// The never-missed ids live beside the learned peers in "wcb_peers", under the same octet
// fingerprint _loadLearnedPeers checks (a different network wipes the namespace). Loop
// task only (NVS); the RX task only flips bits and sets _csNeverMissDirty.
void WCB_Client::_csSaveNeverMiss() {
    Preferences prefs;
    if (!prefs.begin("wcb_peers", false)) return;
    prefs.putUChar("ver", 1);                 // the fingerprint _loadLearnedPeers keys on, as _saveLearnedPeers writes it
    prefs.putUChar("o2", _oct2);
    prefs.putUChar("o3", _oct3);
    prefs.putUInt("csnever", _csNeverMiss);
    prefs.end();
}

void WCB_Client::_csLoadNeverMiss() {
    Preferences prefs;
    if (!prefs.begin("wcb_peers", true)) return;
    const bool ours = prefs.getUChar("o2", 0) == _oct2 && prefs.getUChar("o3", 0) == _oct3;
    const uint32_t m = prefs.getUInt("csnever", 0);
    prefs.end();
    if (ours) _csNeverMiss = _csNeverMiss | m;   // OR: the receive callback is live already and may have set a bit
}

bool WCB_Client::_csSoftApUp() const {
    const wifi_mode_t m = WiFi.getMode();
    return m == WIFI_MODE_AP || m == WIFI_MODE_APSTA;
}

// An interface that owns the radio's channel: a SoftAP up, or the STA associated with an
// access point. Either way esp_wifi_set_channel is not ours to call (IDF: not with the STA
// connected; an AP moves only by restarting), so a move needs the host (onChannelMove).
bool WCB_Client::_csIfaceUp(const char** why) const {
    if (_csSoftApUp()) { if (why) *why = "a SoftAP"; return true; }
    const wifi_mode_t m = WiFi.getMode();
    if (m == WIFI_MODE_STA) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) { if (why) *why = "a WiFi association"; return true; }
    }
    return false;
}

// What a live switch would cut, so it waits (at most chansync::MOVE_DEFER_MAX_MS): the
// sessions _csQuiet() also waits for - a fragmented send, a bulk transfer, a sequence
// request, a sent command awaiting its ACK - and raw packets still arriving (an OTA, a
// config pull through WcbMgmt). Not this device's own command or servo traffic: the rest
// of the mesh has switched at the same instant, so that traffic reaches nobody here, and
// holding the radio for it would only strand the device (the firmware's applyMove waits for
// sessions, not traffic, WCB.ino chanSyncHostBusy).
bool WCB_Client::_csMoveBusy(unsigned long now, const char** why) {
    const char* w = nullptr;
    if (_fragJob.active)      w = "a fragmented send";
    else if (_bulkRx.active)  w = "a bulk transfer";
    else if (_seqPending)     w = "a sequence request";
    else {
        const unsigned long raw = _csRawRxMs;
        if (raw && now - raw < WCB_CHSYNC_QUIET_MS) w = "raw packets arriving (an OTA or a config pull)";
    }
    if (!w) {
        portENTER_CRITICAL(&_pendingMux);
        for (int i = 0; i < WCB_PENDING_MAX && !w; i++)
            if (_pending[i].active) w = "a sent command awaiting its ACK";
        portEXIT_CRITICAL(&_pendingMux);
    }
    if (why) *why = w;
    return w != nullptr;
}

// The neighbors' CHAN facts have had time to arrive: the boot SOLICIT goes out with the
// first boot advert, and every WCB answers it within 0.6 s.
bool WCB_Client::_csViewSettled(unsigned long now) const {
    return now - _csBootMs >= WCB_CHSYNC_VIEW_SETTLE_MS;
}

// Forget the schedule outright: begin(), a move, sync turned off, or a condition that has
// stayed clear WCB_CHSYNC_TRIGGER_RESET_MS (the firmware's resetTrigger).
void WCB_Client::_csResetTrigger() {
    _csTrigSinceMs = 0;
    _csSched       = false;
    _csAttempt     = 0;
    _csNextDueMs   = 0;
    _csClearedMs   = 0;
    _csGaveUpSaid  = false;
}

// A search is over and the radio is home. A MOVE heard at home meanwhile (an operator move
// made during the search, say) is judged now, with the time it has waited off its countdown
// (_csHandleMove schedules from its arrival): left unjudged, this device's next search would
// find its group gone and pull it back.
void WCB_Client::_csEndSearch(unsigned long now) {
    _csSearch = CS_SEARCH_IDLE;
    if (!_csHeldValid) return;
    _csHeldValid = false;
    _csHandleMove(_csHeld, now);
}

// Can the radio leave its channel for a search without breaking a WiFi link? IDF:
// esp_wifi_set_channel must not be called with stations on the SoftAP, nor while the STA
// is connected or connecting. A connect in progress is invisible here; a host that joins
// networks should not search while it connects (a rank-255 relay never searches).
bool WCB_Client::_csCanHop(const char** why) const {
    const wifi_mode_t m = WiFi.getMode();
    if (m == WIFI_MODE_AP || m == WIFI_MODE_APSTA) {
        wifi_sta_list_t list;
        memset(&list, 0, sizeof(list));
        if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) { if (why) *why = "the SoftAP's stations cannot be read"; return false; }
        if (list.num > 0) { if (why) *why = "a station is connected to this device's SoftAP"; return false; }
    }
    if (m == WIFI_MODE_STA || m == WIFI_MODE_APSTA) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) { if (why) *why = "this device is associated with an access point"; return false; }
    }
    return true;
}

// A search leaves the mesh for ~1.5-2 s. Not while anything it would cut is under way.
// The transmit watchdog is part of this: a probe outstanding when the radio leaves would
// see no answer and count a miss against a radio that is fine.
bool WCB_Client::_csQuiet(unsigned long now, const char** why) {
    const char* w = nullptr;
    if (_csMovePending || _csAnnLeft)                  w = "a move is running";
    else if (_fragJob.active)                          w = "a fragmented send is in flight";
    else if (_bulkRx.active)                           w = "a bulk transfer is running";
    else if (_seqPending)                              w = "a sequence request is in flight";
    else if (_txwdSentMs)                              w = "a transmit-watchdog probe is out";
    else if (_bootAnnouncesLeft || _wdpBootLeft)       w = "boot announces or adverts are still going out";
    else {
        const unsigned long busy = _csBusyMs;
        if (busy && now - busy < WCB_CHSYNC_QUIET_MS)  w = "commands or raw packets in the last 3 s";
    }
    if (!w) {
        portENTER_CRITICAL(&_pendingMux);
        for (int i = 0; i < WCB_PENDING_MAX && !w; i++)
            if (_pending[i].active) w = "a sent command is still waiting for its ACK";
        portEXIT_CRITICAL(&_pendingMux);
    }
    if (why) *why = w;
    return w == nullptr;
}

bool WCB_Client::_csSetRadio(uint8_t ch) {
    const esp_err_t e = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (e != ESP_OK) {
        Serial.printf("[WCB_Client] channel sync: esp_wifi_set_channel(%u) failed: %s\n", ch, esp_err_to_name(e));
        return false;
    }
    return true;
}

// One ETM-struct broadcast, the envelope WDP uses (target 0, sequence 0, raw payload in
// structCommand, no CRC). Through _espSendRaw: these are the frames allowed off home.
void WCB_Client::_csSendFrame(uint8_t type, const uint8_t* payload, int len) {
    wcb_packet_etm_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, _password, sizeof(pkt.structPassword) - 1);
    snprintf(pkt.structSenderID, sizeof(pkt.structSenderID), "%d", _deviceID);
    snprintf(pkt.structTargetID, sizeof(pkt.structTargetID), "0");
    pkt.structCommandIncluded = 1;
    if (len > (int)sizeof(pkt.structCommand)) len = sizeof(pkt.structCommand);
    if (len > 0) memcpy(pkt.structCommand, payload, len);
    pkt.structPacketType     = type;
    pkt.structSequenceNumber = 0;
    _espSendRaw(_broadcastMAC, (uint8_t*)&pkt, sizeof(pkt));
}

// Arm a run of MOVE frames counting down to one switch time. The caller decides whether
// this device switches too (an announce at home: _csMovePending) or not (a pull abroad).
void WCB_Client::_csStartFrames(uint8_t toCh, uint8_t flags, uint8_t repeats, uint16_t gapMs,
                                uint16_t countdownMs, unsigned long now) {
    do { _csAnnId = (uint16_t)esp_random(); } while (_csAnnId == 0);
    _csAnnCh       = toCh;
    _csAnnFlags    = flags;
    _csAnnLeft     = repeats;
    _csAnnGapMs    = gapMs;
    _csAnnNextMs   = now;
    _csAnnSwitchMs = now + countdownMs;
}

// Each frame carries the time still left, so a device that hears only the last one
// switches with the rest.
void WCB_Client::_csSendMoveFrame(unsigned long now) {
    chansync::Move m;
    m.newChannel = _csAnnCh;
    m.rank       = chanSyncRank();
    m.flags      = _csAnnFlags;
    m.moveId     = _csAnnId;
    const long left = (long)(_csAnnSwitchMs - now);
    m.delayMs    = (uint16_t)(left > 0 ? left : 0);
    m.cc[0] = _csRegion[0]; m.cc[1] = _csRegion[1]; m.cc[2] = '\0';
    uint8_t b[chansync::MOVE_LEN];
    const int n = chansync::encodeMove(b, sizeof(b), m);
    if (n) _csSendFrame(chansync::PT_MOVE, b, n);
    if (_csAnnLeft) _csAnnLeft--;
    _csAnnNextMs  = now + _csAnnGapMs;
    _csDwellEndMs = now + WCB_CHSYNC_SETTLE_MS;   // a pull: let the last frame leave before the radio does
}

// The loop-task half of every channel-sync frame.
void WCB_Client::_csHandleFrame(const CsFrame& f, unsigned long now) {
    if (!_csEnabled) return;                       // off: never moves, never answers
    switch (f.type) {
        case chansync::PT_MOVE:
            _csHandleMove(f, now);
            break;
        case chansync::PT_PROBE: {
            if (_csSearch != CS_SEARCH_IDLE) break;   // visiting: not a member of this channel's group
            chansync::Probe p;
            if (!chansync::decodeProbe(f.data, sizeof(f.data), p)) break;
            // Answer after (id % 16) x 3 ms so a group does not answer in lockstep. A later
            // PROBE replaces an unanswered one: the latest prober is the one listening.
            _csReplyPending = true;
            _csReplyNonce   = p.nonce;
            _csReplyProber  = f.sender;
            _csReplyDueMs   = f.rxMs + (unsigned long)(_deviceID % 16) * chansync::REPLY_JITTER_STEP_MS;
            break;
        }
        case chansync::PT_PROBE_REPLY: {
            if (_csSearch != CS_SEARCH_VISIT) break;
            chansync::Reply r;
            if (!chansync::decodeReply(f.data, sizeof(f.data), r)) break;
            if (r.nonce != _csNonce) break;                                    // not our search
            // The responder says which channel it is on, so a reply drained after the radio
            // moved on is still credited to the right channel.
            if (!chansync::validChannel(r.channel, chansync::MAX_CHANNEL_CAP) || r.channel == _csHomeCh) break;
            // Credit the best the reply names, and the responder itself ("itself included"),
            // as the firmware's onReply does. A responder that still lists THIS device as
            // heard (its offline sweep has not run yet) can name it as the best: that is not
            // who leads over there, and judgeSearch would compare this device with itself
            // and neither go nor pull - so the responder's own key stands in for it.
            chansync::Key responder;
            responder.id    = f.sender;
            responder.rank  = r.rank;
            responder.flags = (uint8_t)(r.flags & (chansync::F_PINNED | chansync::F_SYNC));
            chansync::Key k = r.best;
            k.flags = (uint8_t)(k.flags & (chansync::F_PINNED | chansync::F_SYNC));
            if (k.id == 0 || k.id > WCB_MAX_BOARDS || k.id == _deviceID) k = chansync::noKey();
            if (chansync::better(responder, k)) k = responder;
            if (chansync::better(k, _csFound[r.channel])) _csFound[r.channel] = k;
            break;
        }
        default: break;
    }
}

void WCB_Client::_csHandleMove(const CsFrame& f, unsigned long now) {
    chansync::Move m;
    if (!chansync::decodeMove(f.data, sizeof(f.data), m)) return;
    // The first frame of a move decides; its repeats are ignored, accepted or not.
    if (f.sender == _csSeenSender && m.moveId == _csSeenMoveId && now - _csSeenMs < WCB_CHSYNC_DEDUP_MS) return;
    // Heard at home while a search is under way (one heard away never reaches here - the RX
    // task drops it): the search decides for the group first, and this one is judged when it
    // is over (_csEndSearch). Not recorded as seen until then, or that judgement would never
    // come. The latest one is kept.
    if (_csSearch != CS_SEARCH_IDLE) { _csHeld = f; _csHeldValid = true; return; }
    _csSeenSender = f.sender;
    _csSeenMoveId = m.moveId;
    _csSeenMs     = now;
    if (_csHostMoveCh) return;                     // the host is restarting onto a move already taken

    chansync::Key sender;
    sender.id    = f.sender;
    sender.rank  = m.rank;
    sender.flags = (uint8_t)(m.flags & (chansync::F_PINNED | chansync::F_SYNC));
    const chansync::MoveVerdict v = chansync::judgeMove(m, sender, _csSelfKey(), _csBestPresent(now),
                                                        currentChannel(), _csMaxCh, _csEnabled);
    if (v != chansync::MV_ACCEPT) {
        if (v != chansync::MV_SAME_CHANNEL)
            Serial.printf("[WCB_Client] channel sync: not following WCB%u to channel %u - %s\n",
                          f.sender, m.newChannel, csVerdictText(v));
        return;
    }
    // Never longer than a first frame's countdown, whatever a frame claims.
    const unsigned long waitMs = m.delayMs > chansync::MOVE_COUNTDOWN_MS ? chansync::MOVE_COUNTDOWN_MS : m.delayMs;
    _csAnnLeft      = 0;                           // an own announce is superseded by a better sender's
    _csMovePending  = true;
    _csDeferSinceMs = 0;
    _csMoveCh       = m.newChannel;
    _csMoveAtMs     = f.rxMs + waitMs;
    Serial.printf("[WCB_Client] channel sync: WCB%u (rank %u%s) moves the mesh to channel %u - switching in %lu ms\n",
                  f.sender, m.rank, (m.flags & chansync::MF_OPERATOR) ? ", operator" : "", m.newChannel, waitMs);
}

// The switch. The host decides first; the library switches live only with no SoftAP up and
// no STA association (either owns the channel; esp_wifi_set_channel is not ours to call).
void WCB_Client::_csApplyMove(uint8_t ch, unsigned long now) {
    if (_csMoveCb && _csMoveCb(ch)) {
        // Applied live inside the callback (setMeshChannel(ch), no SoftAP): it has arrived.
        uint8_t primary = 0;
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
        if (_meshChannel == ch && esp_wifi_get_channel(&primary, &second) == ESP_OK && primary == ch) {
            _csArrived(ch, now);
            Serial.printf("[WCB_Client] channel sync: now on channel %u (the host applied it)\n", ch);
            return;
        }
        // The host restarts onto ch. Until it does this device is LEAVING, not home: it
        // starts no search (which would find its group gone and pull it back), announces
        // and follows no other move, and says so loudly if the restart never comes (_csTick).
        _csHostMoveCh = ch;
        _csHostMoveMs = now ? now : 1;
        _csHostWarnMs = _csHostMoveMs;
        _csAnnLeft    = 0;
        _csResetTrigger();
        Serial.printf("[WCB_Client] channel sync: the mesh is moving to channel %u - the host applies it\n", ch);
        return;
    }
    const char* w = nullptr;
    if (_csIfaceUp(&w)) {
        Serial.printf("[WCB_Client] channel sync: cannot move to channel %u - %s owns this radio's channel and "
                      "the host did not apply the move (onChannelMove); staying on channel %u\n", ch, w, currentChannel());
        return;
    }
    if (!_csSetRadio(ch)) {
        Serial.printf("[WCB_Client] channel sync: staying on channel %u\n", currentChannel());
        return;
    }
    _csArrived(ch, now);
    Serial.printf("[WCB_Client] channel sync: now on channel %u\n", ch);
}

// This device is on `ch` for good (a live move). ESP-NOW peers are registered on
// channel 0, "the current channel", so none needs registering again.
void WCB_Client::_csArrived(uint8_t ch, unsigned long now) {
    _meshChannel       = ch;
    _lastWarnedChannel = 0;
    for (int i = 0; i < WCB_MAX_BOARDS; i++) _csHeardMs[i] = 0;   // heard on the old channel
    _csReplyPending = false;
    _csResetTrigger();                             // a fresh trigger on the new channel
    // A channel change is not a dead radio: no verdict until the boards have had time to be
    // heard here, and the ones that stayed behind have dropped out of the judges.
    _txwdSentMs = 0;
    _txwdMisses = 0;
    _txwdGood   = 0;
    if (_txwdEnabled) _txwdNextMs = now + WCB_TXWD_HEARD_MS;
    // Be heard on the new channel at once: a heartbeat now, the advert (its CHAN TLV
    // names the new channel) just after, staggered by id like a solicited one.
    _nextHeartbeatMs = now;
    if (_wdpType[0]) { _wdpBootLeft = 2; _wdpNextBootMs = now + 200 + (unsigned long)((_deviceID % 16) * 20); }
}

void WCB_Client::_csStartSearch(unsigned long now, bool manual, uint8_t missingId) {
    _csHomeCh = currentChannel();
    do { _csNonce = (uint16_t)esp_random(); } while (_csNonce == 0);
    for (int c = 0; c <= chansync::MAX_CHANNEL_CAP; c++) _csFound[c] = chansync::noKey();
    memset(_csPullMask, 0, sizeof(_csPullMask));
    _csVisitCh    = 0;
    _csDwellEndMs = now;
    _csSearches++;
    if (manual)
        Serial.printf("[WCB_Client] channel sync: searching channels 1-%u from channel %u (requested)\n",
                      _csMaxCh, _csHomeCh);
    else
        Serial.printf("[WCB_Client] channel sync: WCB%u is missing - searching channels 1-%u from channel %u "
                      "(attempt %u of %u)\n", missingId, _csMaxCh, _csHomeCh, _csAttempt, (unsigned)chansync::SEARCH_ATTEMPTS);
    // _csOffHome is set by _csSearchStep just before the radio leaves: until then this device
    // is home, and a MOVE heard meanwhile is held for after the search, not dropped.
    _csSearch = CS_SEARCH_VISIT;
}

void WCB_Client::_csGoHome() {
    if (_csHomeCh && !_csSetRadio(_csHomeCh))
        Serial.printf("[WCB_Client] WARNING: channel sync could not return to channel %u\n", _csHomeCh);
    _csOffHome = false;
    _csAnnLeft = 0;
}

// One step of a search, per update(). Never blocks: each channel's dwell and each pull
// frame's gap is a deadline checked here.
void WCB_Client::_csSearchStep(unsigned long now) {
    if (_csSearch == CS_SEARCH_VISIT) {
        if (_csVisitCh && (long)(now - _csDwellEndMs) < 0) return;   // listening for REPLYs
        uint8_t ch = _csVisitCh;
        do { ch++; } while (ch == _csHomeCh);
        if (ch > _csMaxCh) {                       // every channel visited
            _csGoHome();
            _csSearchDone(now);
            return;
        }
        _csVisitCh = ch;
        _csOffHome = true;                         // before the switch: nothing heard there is taken for home
        if (!_csSetRadio(ch)) {
            _csGoHome();
            Serial.println("[WCB_Client] channel sync: search abandoned");
            _csEndSearch(now);
            return;
        }
        const chansync::Key self = _csSelfKey();
        chansync::Probe p;
        p.homeChannel = _csHomeCh;
        p.rank        = self.rank;
        p.flags       = self.flags;
        p.nonce       = _csNonce;
        uint8_t b[chansync::PROBE_LEN];
        const int n = chansync::encodeProbe(b, sizeof(b), p);
        if (n) _csSendFrame(chansync::PT_PROBE, b, n);
        _csDwellEndMs = now + chansync::PROBE_DWELL_MS;
        return;
    }
    if (_csSearch == CS_SEARCH_PULL) {
        if (_csAnnLeft) {                          // frames still to send on this channel
            if ((long)(now - _csAnnNextMs) >= 0) _csSendMoveFrame(now);
            return;
        }
        if (_csVisitCh && (long)(now - _csDwellEndMs) < 0) return;   // let the last frame leave
        uint8_t ch = _csVisitCh;
        do { ch++; } while (ch <= chansync::MAX_CHANNEL_CAP && !_csPullMask[ch]);
        if (ch > chansync::MAX_CHANNEL_CAP) {      // every worse group told
            _csGoHome();
            Serial.printf("[WCB_Client] channel sync: back on channel %u\n", _csHomeCh);
            _csEndSearch(now);
            return;
        }
        _csVisitCh = ch;
        _csOffHome = true;
        if (!_csSetRadio(ch)) {
            _csGoHome();
            Serial.println("[WCB_Client] channel sync: pull abandoned");
            _csEndSearch(now);
            return;
        }
        _csStartFrames(_csHomeCh, _csSelfKey().flags, chansync::PULL_REPEATS, chansync::PULL_REPEAT_GAP_MS,
                       chansync::PULL_COUNTDOWN_MS, now);
        _csSendMoveFrame(now);
        return;
    }
}

// Home again: what the replies said (chansync::judgeSearch).
void WCB_Client::_csSearchDone(unsigned long now) {
    const chansync::Key self = _csSelfKey();
    const chansync::SearchResult r = chansync::judgeSearch(self, _csHomeCh, _csFound);
    if (r.moveTo) {
        const chansync::Key k = _csFound[r.moveTo];
        const char* w = nullptr;
        if (_csIfaceUp(&w) && !_csMoveCb) {
            Serial.printf("[WCB_Client] channel sync: WCB%u (rank %u) leads on channel %u, but this device cannot "
                          "follow - %s owns its channel and no onChannelMove() applies a move\n",
                          k.id, k.rank, r.moveTo, w);
            _csEndSearch(now);
            return;
        }
        Serial.printf("[WCB_Client] channel sync: WCB%u (rank %u) leads on channel %u - taking this group there\n",
                      k.id, k.rank, r.moveTo);
        _csStartFrames(r.moveTo, self.flags, chansync::MOVE_REPEATS, chansync::MOVE_REPEAT_GAP_MS,
                       chansync::MOVE_COUNTDOWN_MS, now);
        _csMovePending  = true;
        _csDeferSinceMs = 0;
        _csMoveCh       = r.moveTo;
        _csMoveAtMs     = _csAnnSwitchMs;
        _csEndSearch(now);                         // a MOVE held meanwhile from a better sender supersedes it
        return;
    }
    if (r.pullCount) {
        memcpy(_csPullMask, r.pull, sizeof(_csPullMask));
        Serial.printf("[WCB_Client] channel sync: %u group(s) with a worse leader found - pulling them to channel %u:",
                      r.pullCount, _csHomeCh);
        for (uint8_t c = 1; c <= chansync::MAX_CHANNEL_CAP; c++)
            if (_csPullMask[c]) Serial.printf(" ch %u (WCB%u, rank %u)", c, _csFound[c].id, _csFound[c].rank);
        Serial.println();
        _csSearch     = CS_SEARCH_PULL;
        _csVisitCh    = 0;
        _csAnnLeft    = 0;
        _csDwellEndMs = now;
        return;                                    // _csSearchStep takes the radio away for each pull
    }
    Serial.printf("[WCB_Client] channel sync: nobody answered on the other channels - staying on channel %u\n",
                  _csHomeCh);
    _csEndSearch(now);
}

// The whole state machine, first thing in update() (loop task).
void WCB_Client::_csTick(unsigned long now) {
    // 1. What arrived.
    CsFrame f;
    while (_csPop(f)) _csHandleFrame(f, now);

    // The never-missed set changed on the RX task: save it here (NVS), while the radio is home.
    if (_csNeverMissDirty && _csSearch == CS_SEARCH_IDLE) { _csNeverMissDirty = false; _csSaveNeverMiss(); }

    // 2. Turned off (live): unwind whatever is in flight, here on the loop task.
    if (!_csEnabled) {
        if (_csSearch != CS_SEARCH_IDLE) {
            _csGoHome();
            _csSearch = CS_SEARCH_IDLE;
            Serial.println("[WCB_Client] channel sync: off - search abandoned");
        }
        _csHeldValid    = false;
        _csMovePending  = false;
        _csDeferSinceMs = 0;
        _csAnnLeft      = 0;
        _csReplyPending = false;
        _csSearchNowReq = false;
        _csResetTrigger();
        return;
    }

    // 3. A search owns the radio until it is home.
    if (_csSearch != CS_SEARCH_IDLE) { _csSearchStep(now); return; }

    // 3b. The host took a move and has not restarted: this device is leaving and does
    //     nothing that assumes it stays. If the restart never comes, say so - loudly.
    if (_csHostMoveCh && now - _csHostWarnMs >= WCB_CHSYNC_HOST_MOVE_MS) {
        _csHostWarnMs = now;
        Serial.printf("[WCB_Client] *** channel sync: the host took the move to channel %u %lu s ago and has not "
                      "restarted - this device is still on channel %u, OFF THE MESH ***\n", _csHostMoveCh,
                      (unsigned long)((now - _csHostMoveMs) / 1000UL), currentChannel());
    }

    // 4. MOVE frames this device is announcing at home.
    if (_csAnnLeft && (long)(now - _csAnnNextMs) >= 0) _csSendMoveFrame(now);

    // 5. The switch. A live channel change cuts whatever session is in flight, so it waits
    //    for one (_csMoveBusy) - never past MOVE_DEFER_MAX_MS, as on a WCB (applyMove).
    if (_csMovePending && (long)(now - _csMoveAtMs) >= 0) {
        const char* w = nullptr;
        bool hold = false;
        if (_csMoveBusy(now, &w)) {
            if (!_csDeferSinceMs) {
                _csDeferSinceMs = now ? now : 1;
                Serial.printf("[WCB_Client] channel sync: the move to channel %u waits for %s (at most %lu s)\n",
                              _csMoveCh, w, (unsigned long)(chansync::MOVE_DEFER_MAX_MS / 1000UL));
            }
            if (now - _csDeferSinceMs < chansync::MOVE_DEFER_MAX_MS) hold = true;
            else Serial.printf("[WCB_Client] channel sync: %s still after %lu s - moving anyway\n", w,
                               (unsigned long)(chansync::MOVE_DEFER_MAX_MS / 1000UL));
        }
        if (!hold) {
            _csMovePending  = false;
            _csDeferSinceMs = 0;
            _csAnnLeft      = 0;
            _csApplyMove(_csMoveCh, now);
        }
    }

    // 6. A REPLY owed to a PROBE: the best device heard here, itself included, the
    //    prober left out.
    if (_csReplyPending && (long)(now - _csReplyDueMs) >= 0) {
        _csReplyPending = false;
        const chansync::Key self = _csSelfKey();
        chansync::Reply r;
        r.channel = currentChannel();
        r.nonce   = _csReplyNonce;
        r.rank    = self.rank;
        r.flags   = self.flags;
        r.best    = _csBestPresent(now, _csReplyProber);
        r.cc[0] = _csRegion[0]; r.cc[1] = _csRegion[1]; r.cc[2] = '\0';
        uint8_t b[chansync::REPLY_LEN];
        const int n = chansync::encodeReply(b, sizeof(b), r);
        if (n) _csSendFrame(chansync::PT_PROBE_REPLY, b, n);
    }

    // 6b. The boot SOLICIT for a device with no identity (with one, _wdpTick sends it after
    //     the first boot advert): the neighbors' CHAN facts now, not at their next 60 s advert.
    if (!_csBootSolicited && !_wdpType[0] && now - _csBootMs >= 1500) {
        _csBootSolicited = true;
        _sendWdpSolicit();
    }

    // 7. Whether to search: four times a second is plenty.
    if (now - _csLastEvalMs < 250) return;
    _csLastEvalMs = now;
    if (_csMovePending || _csAnnLeft || _csHostMoveCh) return;

    const char* why = nullptr;
    if (_csSearchNowReq) {
        if (_csBestPresent(now).id != _deviceID) {     // a leader appeared since the request
            _csSearchNowReq = false;
            Serial.println("[WCB_Client] channel sync: search not run - another device leads this group now");
        } else if (!_csCanHop(&why)) {
            _csSearchNowReq = false;
            Serial.printf("[WCB_Client] channel sync: cannot search now - %s\n", why);
        } else if (_csQuiet(now, &why)) {
            _csSearchNowReq = false;
            _csStartSearch(now, true, 0);
            // A requested search restarts the schedule: if a board is still missing, the next
            // attempts follow from here (the firmware's ?CHSYNC,SEARCH).
            _csSched       = true;
            _csAttempt     = 1;
            _csNextDueMs   = now + (chansync::searchAttemptOffsetMs(1) - chansync::searchAttemptOffsetMs(0));
            _csTrigSinceMs = now ? now : 1;
            _csClearedMs   = 0;
            _csGaveUpSaid  = false;
            return;
        } else if (now - _csSearchNowMs >= chansync::MOVE_DEFER_MAX_MS) {
            _csSearchNowReq = false;
            Serial.printf("[WCB_Client] channel sync: search not run - not quiet for %lu s (%s)\n",
                          (unsigned long)(chansync::MOVE_DEFER_MAX_MS / 1000UL), why);
        }
        if (_csSearchNowReq) return;               // still waiting for quiet
    }

    // The trigger, as the WCB firmware's evaluateTrigger: only a group's leader that misses
    // a known board, not in the boot grace, once the condition has held TRIGGER_STABLE_MS.
    // canHop is passed as true and checked when an attempt is due instead: a SoftAP whose
    // phone comes and goes would otherwise restart the schedule every time.
    uint8_t who = 0;
    const bool missing = _csMemberMissing(now, &who);
    const bool cond = now - _csBootMs >= chansync::BOOT_GRACE_MS &&
                      chansync::shouldSearch(_csSelfKey(), _csBestPresent(now), missing, _csEnabled, true);
    if (!cond) {
        _csTrigSinceMs = 0;
        // A condition back within WCB_CHSYNC_TRIGGER_RESET_MS carries on the schedule it had: a
        // member that flaps offline and online would otherwise restart seven attempts each time.
        if (_csSched) {
            if (!_csClearedMs) _csClearedMs = now ? now : 1;
            else if (now - _csClearedMs >= WCB_CHSYNC_TRIGGER_RESET_MS) _csResetTrigger();
        }
        return;
    }
    _csClearedMs = 0;
    if (!_csTrigSinceMs) _csTrigSinceMs = now ? now : 1;
    if (!_csSched) {                               // a new episode
        _csSched      = true;
        _csAttempt    = 0;
        _csNextDueMs  = _csTrigSinceMs + chansync::TRIGGER_STABLE_MS;
        _csGaveUpSaid = false;
    } else if ((long)(_csNextDueMs - (_csTrigSinceMs + chansync::TRIGGER_STABLE_MS)) < 0) {
        _csNextDueMs = _csTrigSinceMs + chansync::TRIGGER_STABLE_MS;   // back after a short clear: it must hold again
    }
    if (_csAttempt >= chansync::SEARCH_ATTEMPTS) {
        if (!_csGaveUpSaid) {
            _csGaveUpSaid = true;
            Serial.printf("[WCB_Client] channel sync: WCB%u is still missing after %u searches - not searching again "
                          "until the next boot, chanSyncSearchNow(), or it has been back for %lu minutes\n", who,
                          (unsigned)chansync::SEARCH_ATTEMPTS, (unsigned long)(WCB_CHSYNC_TRIGGER_RESET_MS / 60000UL));
        }
        return;
    }
    if ((long)(now - _csNextDueMs) < 0) return;
    if (!_csCanHop(nullptr) || !_csQuiet(now, nullptr)) return;   // due: go as soon as it can
    // Re-based on when this attempt really starts: one held up for a while (a station on the
    // AP, a busy device) is not followed at once by the next, whose offset has also passed.
    _csAttempt++;
    if (_csAttempt < chansync::SEARCH_ATTEMPTS)
        _csNextDueMs = now + (chansync::searchAttemptOffsetMs(_csAttempt) - chansync::searchAttemptOffsetMs(_csAttempt - 1));
    _csStartSearch(now, false, who);
}
