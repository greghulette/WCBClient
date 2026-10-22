/*
  ChannelSync.ino — WCB_Client Library Example (1.19.0)

  The ESP32 has one radio, so the whole mesh shares ONE channel. When the mesh
  moves (a WCB joined a WiFi network on another channel, or an operator moved it
  with ?CHSYNC,MOVE), channel sync carries this device along; and a group leader
  that misses a board it knows goes looking for it on the other channels. All of
  that is the library's job, on by default. The host has three:

    1. START where the mesh last was. A move is not remembered by the library:
       read the channel you saved and pass it to setMeshChannel() before begin().
    2. SAVE a move when it happens (onChannelMove). Return false and the library
       switches the radio live; a sketch hosting a SoftAP saves, asks its own
       loop() for a restart and returns true (the AP comes up on the new channel).
    3. SET THE REGION if the mesh uses channel 12 or 13 (setRegion, before begin()),
       the same one every WCB has (?REGION). Default "01": channels 1-11.

  Give the device an identity (setIdentity): its channel-sync key travels in the
  WDP advert (TLV 0x14), and without one the WCBs rank it as if it were a WCB.

  Serial Monitor, 115200, line ending "Newline":
    s        status, ending in the [CHSYNC:...] machine line
    f        search the other channels now (only if this device leads its group)
    m<ch>    move the whole mesh to channel <ch> (an operator move), e.g. m6

  Quick start:
    1. Fill in the network credentials below to match your WCB system.
    2. Choose a DEVICE_ID no real WCB uses.
    3. ETM must be on on the WCBs (the default): channel sync rides it, as WDP does.
*/

#include <WCB_Client.h>
#include <Preferences.h>

const uint8_t MAC_OCT2     = 0x00;
const uint8_t MAC_OCT3     = 0x00;
const char*   PASSWORD     = "change_me_or_risk_takeover";
const uint8_t WCB_QUANTITY = 12;
const uint8_t DEVICE_ID    = 5;

// "01" (world safe, channels 1-11) or the mesh's country code, e.g. "DE" for 12-13.
const char*   REGION       = "01";

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

static const char* NVS_NS = "chsync_demo";

// Called from wcb.update() - the LOOP task - once, when a move reaches its switch
// time. NVS is safe here. Save either way; false = "library, switch the radio live".
bool onChannelMove(uint8_t channel) {
    Preferences p;
    if (p.begin(NVS_NS, false)) { p.putUChar("ch", channel); p.end(); }
    Serial.printf("[APP] the mesh moved to channel %u - saved for the next boot\n", channel);
    return false;   // no SoftAP here, so the library can move the radio itself
}

void setup() {
    Serial.begin(115200);

    // 1. Where the mesh was when this device last ran (or the default).
    Preferences p;
    uint8_t channel = WCB_MESH_CHANNEL;
    if (p.begin(NVS_NS, true)) { channel = p.getUChar("ch", WCB_MESH_CHANNEL); p.end(); }

    wcb.setRegion(REGION);               // 3. before begin(), and before setMeshChannel(12/13)
    wcb.setMeshChannel(channel);
    // wcb.setChanSyncRank(...);         // default 100 + DEVICE_ID: any WCB (rank = its
                                         // number) and NaviCore (0) lead before this device
    wcb.onChannelMove(onChannelMove);    // 2.

    if (!wcb.begin()) {
        Serial.println("[WCB] begin() FAILED (see error above) — halting.");
        while (true) delay(1000);
    }
    wcb.setIdentity("ChannelSyncDemo", "1.0");
    Serial.printf("[APP] on channel %u of 1-%u (region %s). Keys: s, f, m<ch>\n",
                  wcb.currentChannel(), wcb.maxChannel(), wcb.region());
}

static char   line[16];
static size_t lineLen = 0;

static void handleLine(const char* l) {
    if (l[0] == 's') {
        wcb.printChanSyncStatus(Serial);
    } else if (l[0] == 'f') {
        wcb.chanSyncSearchNow();         // logs why not, when it cannot
    } else if (l[0] == 'm') {
        String why;
        const int ch = atoi(l + 1);
        if (wcb.chanSyncMove((uint8_t)(ch > 0 && ch < 256 ? ch : 0), &why)) Serial.printf("[APP] moving the mesh to channel %d\n", ch);
        else                                     Serial.printf("[APP] move refused: %s\n", why.c_str());
    }
}

void loop() {
    wcb.update();
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen) { line[lineLen] = '\0'; handleLine(line); lineLen = 0; }
        } else if (lineLen < sizeof(line) - 1) {
            line[lineLen++] = c;
        }
    }
}
