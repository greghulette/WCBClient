/*
  SequenceInventory.ino — WCB_Client Library Example

  Asks the WCBs on the mesh which STORED SEQUENCES they have, by NAME, and keeps
  that list fresh without polling.

  Why this exists: a controller that offers "run a sequence" needs to know what
  sequences exist. Hard-coding the list goes stale silently the first time someone
  saves one from the Wizard. There are three ways to ask a WCB and only one of them
  scales:

    ?SEQ,LIST   — human console text, local USB only, values included. Not a wire
                  format; there is no mesh reply at all.
    config pull — machine-readable but carries every sequence VALUE plus the whole
                  board config, and caps at ~2912 chars. A board with a real set of
                  sequences silently exceeds it and answers with nothing.
    this        — names only, ~16 bytes each, and a 4-byte fingerprint in the WDP
                  advert tells you when the list has actually changed.

  How the two halves fit together:
    • WCBNeighbor::seqHash arrives free with every WDP advert (~60 s, plus an
      immediate re-advert whenever the board's config changes). It is a fingerprint
      of that board's sequence inventory.
    • When the hash you saw differs from the hash you cached, call
      requestSequenceNames() and refresh. Otherwise do nothing.

  So a saved sequence on any WCB shows up here within a couple of seconds, and a
  mesh that is not changing costs zero request traffic.

  Flash to any ESP32 board — no wiring required, everything is wireless.
*/

#include <WCB_Client.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network credentials — must match your WCB system exactly (see BasicUsage).
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 4;

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// ─────────────────────────────────────────────────────────────────────────────
// Our cached view of the mesh's sequences.
//
// `knownHash` is what we had when we last pulled names from that board. 0 means
// "never pulled". Note that 0 is also what seqHash reads for a board running
// firmware older than the SEQHASH TLV — such a board simply never triggers a
// refresh here, which is the right outcome: it has nothing to tell us.
//
// An EMPTY inventory is NOT hash 0 — it is 0x811C9DC5 (the FNV-1a basis) — so
// "board with zero sequences" and "board that can't report" stay distinguishable.
// ─────────────────────────────────────────────────────────────────────────────
struct BoardSeqs {
    uint32_t knownHash;
    uint16_t count;
    String   names;      // comma-separated, exactly as the board sent it
};
BoardSeqs gSeqs[WCB_MAX_BOARDS];

// Only one request may be in flight at a time, so we walk the boards round-robin
// and let each update() tick issue at most one.
uint8_t gScanCursor = 1;

// ─────────────────────────────────────────────────────────────────────────────
// The inventory arrived. Fires on the LOOP task, so doing real work here is safe.
// ─────────────────────────────────────────────────────────────────────────────
void onSequenceNames(uint8_t wcbNumber, uint32_t hash, uint16_t count, const char* names) {
    if (wcbNumber < 1 || wcbNumber > WCB_MAX_BOARDS) return;

    BoardSeqs& b = gSeqs[wcbNumber - 1];
    b.knownHash = hash;
    b.count     = count;
    b.names     = names;    // copy — `names` is freed when this returns

    Serial.printf("\n[SEQ] WCB%d has %u sequence%s (hash %08X)\n",
                  wcbNumber, count, count == 1 ? "" : "s", hash);

    // Split the comma-separated list. A stored key can never contain a comma —
    // the firmware takes the key as everything before the first one — so this
    // needs no escaping logic.
    int start = 0;
    while (start < (int)b.names.length()) {
        int comma = b.names.indexOf(',', start);
        if (comma < 0) comma = b.names.length();
        String key = b.names.substring(start, comma);
        key.trim();
        if (key.length() > 0) Serial.printf("        ;SEQ%s\n", key.c_str());
        start = comma + 1;
    }
    if (count == 0) Serial.println("        (none stored)");
}

// ─────────────────────────────────────────────────────────────────────────────
// A neighbor advert was decoded. Runs on the WiFi task — so DON'T request from
// here; just note it and let loop() act. (We don't even need this callback for
// the refresh logic below, which polls getNeighbor(); it's here to show that the
// hash is available the moment an advert lands.)
// ─────────────────────────────────────────────────────────────────────────────
void onNeighbor(const WCBNeighbor& nb) {
    if (!nb.valid || nb.isClient) return;
    if (nb.seqHash != 0 && nb.wcbNumber >= 1 && nb.wcbNumber <= WCB_MAX_BOARDS &&
        nb.seqHash != gSeqs[nb.wcbNumber - 1].knownHash) {
        Serial.printf("[SEQ] WCB%d inventory changed (%08X -> %08X) — will refresh\n",
                      nb.wcbNumber, gSeqs[nb.wcbNumber - 1].knownHash, nb.seqHash);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== WCB_Client — Sequence Inventory ===\n");

    wcb.onSequenceNames(onSequenceNames);
    wcb.onNeighbor(onNeighbor);

    // Advertise ourselves so the WCBs know we exist (and so this device shows up
    // in their ?WDP,LIST). Not required to READ the mesh, but good manners.
    wcb.setIdentity("SeqBrowser", "1.0.0", "", "sequence-browser");

    if (!wcb.begin()) {
        Serial.println("ERROR: wcb.begin() failed — check credentials/ESP-NOW init");
        return;
    }
    Serial.println("Listening for WDP adverts. Sequence lists refresh on change.\n");
}

void loop() {
    wcb.update();

    // One request at a time. Walk the boards; for each, compare the hash it is
    // advertising against the one we last pulled with, and refresh on a mismatch.
    // A steady mesh issues nothing at all.
    if (!wcb.sequenceNamesPending()) {
        for (uint8_t i = 0; i < WCB_MAX_BOARDS; i++) {
            uint8_t n = gScanCursor;
            gScanCursor = (gScanCursor % WCB_MAX_BOARDS) + 1;

            const WCBNeighbor* nb = wcb.getNeighbor(n);
            if (!nb || nb->isClient) continue;      // gone, or not a WCB
            if (nb->seqHash == 0) continue;         // firmware predates the TLV
            if (nb->seqHash == gSeqs[n - 1].knownHash) continue;   // already current

            if (wcb.requestSequenceNames(n)) break; // one per tick
        }
    }

    delay(10);
}
