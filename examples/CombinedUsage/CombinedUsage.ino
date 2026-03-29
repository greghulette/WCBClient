/*
  CombinedUsage.ino — WCBClient + WCBStream Combined Example

  Demonstrates text command sending/receiving AND Pololu Maestro servo
  forwarding running simultaneously on the same ESP32.

  ── What this sketch does ────────────────────────────────────────────────────
  Every 2 seconds : moves servos on two Maestros back and forth using
                    WCBStream — no serial wire required.
  Every 5 seconds : broadcasts a text command to all WCBs, and sends a
                    unicast text command to WCB1 (if it's online).
  Any time        : prints commands received from the WCB network.

  ── Network credentials ──────────────────────────────────────────────────────
  Fill in the values below to match your WCB system. Find them via the WCB
  Config Tool or by querying a WCB over serial (?WCBM, ?WCBP, ?WCBQ).
*/

#include <WCBClient.h>
#include <WCBStream.h>
#include <PololuMaestro.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 4;
const uint8_t DEVICE_ID    = 20;  // 20 = special out-of-band slot

// ─────────────────────────────────────────────────────────────────────────────
// Maestro routing — set to match your physical wiring
//
// MAESTRO_WCB_x  : WCB number each Maestro is physically wired to
// MAESTRO_PORT_x : serial port on that WCB connected to the Maestro RX (1–5)
// MAESTRO_ID_x   : device number from Maestro Control Center (Serial tab)
//
// The two Maestros can be on the same WCB or on different WCBs — just set
// MAESTRO_WCB_1 and MAESTRO_WCB_2 independently.
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAESTRO_WCB_1  = 2;   // WCB2 has the first Maestro
const uint8_t MAESTRO_PORT_1 = 1;   // wired to Serial1 on WCB2
const uint8_t MAESTRO_ID_1   = 1;

const uint8_t MAESTRO_WCB_2  = 2;   // change to 3 (or any other WCB) if needed
const uint8_t MAESTRO_PORT_2 = 2;   // wired to Serial2 on WCB2
const uint8_t MAESTRO_ID_2   = 2;

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void onCommandReceived(uint8_t senderID, const char* command) {
    Serial.printf("[RX] From WCB%d: %s\n", senderID, command);
}

void onStatusChanged(uint8_t wcbID, bool online) {
    Serial.printf("[STATUS] WCB%d is now %s\n", wcbID, online ? "ONLINE" : "OFFLINE");
}

// ─────────────────────────────────────────────────────────────────────────────
// WCBClient must be declared before any WCBStream objects.
// WCBStream self-registers with WCBClient on construction so wcb.update()
// flushes all streams automatically — no extra calls needed in loop().
// ─────────────────────────────────────────────────────────────────────────────
WCBClient wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
              onCommandReceived, onStatusChanged);

// Unicast each stream to its own WCB:port pair
WCBStream maestroStream1(MAESTRO_WCB_1, MAESTRO_PORT_1);
WCBStream maestroStream2(MAESTRO_WCB_2, MAESTRO_PORT_2);

MiniMaestro maestro1(maestroStream1, Maestro::noResetPin, MAESTRO_ID_1);
MiniMaestro maestro2(maestroStream2, Maestro::noResetPin, MAESTRO_ID_2);

unsigned long lastMaestroMs = 0;
unsigned long lastCommandMs = 0;
bool          servoToggle   = false;

void setup() {
    Serial.begin(115200);
    delay(2000);
    wcb.begin();
}

void loop() {
    // update() drives heartbeats, offline detection, and all WCBStream flushes
    wcb.update();

    // ── Maestro servo movement (every 2 seconds) ──────────────────────────
    if (millis() - lastMaestroMs >= 2000) {
        lastMaestroMs = millis();
        servoToggle = !servoToggle;
        uint16_t pos = servoToggle ? 4000 : 8000;

        maestro1.setTarget(0, pos);
        maestro2.setTarget(0, pos);
        Serial.printf("[TX] Maestros ch0 → %d (%s)\n", pos,
                      servoToggle ? "left" : "right");
    }

    // ── Text command sending (every 5 seconds) ────────────────────────────
    if (millis() - lastCommandMs >= 5000) {
        lastCommandMs = millis();

        wcb.broadcast(":PP100");
        Serial.println("[TX] Broadcast :PP100");

        if (wcb.isOnline(1)) {
            wcb.send(1, ":LEDON");
            Serial.println("[TX] Unicast to WCB1: :LEDON");
        } else {
            Serial.println("[TX] WCB1 offline — skipping");
        }
    }
}
