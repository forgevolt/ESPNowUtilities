// ---- PairAndPing ------------------------------------------------------------------------
// Two boards find each other, pair, and exchange a counter.
//
// Flash the same sketch to both, with cIsInitiator true on one and false on the other. The
// initiator broadcasts until somebody answers; the responder waits and replies.

#include <ESPNowUtilities.h>

constexpr bool cIsInitiator = true;   // set to false on the second board

// The message doubles as the heartbeat: a peer is dropped after cPeerTimeoutMs of silence, so
// it has to arrive well inside that, with room for a lost frame or two.
constexpr unsigned long cHeartbeatIntervalMs = 250;
constexpr unsigned long cPairingIntervalMs   = 1000;

static_assert(cHeartbeatIntervalMs * 2 <= ESPNowConnection::cPeerTimeoutMs,
              "heartbeat must beat cPeerTimeoutMs with margin, or the peer will be dropped");

enum EMyDevice : uint8_t { eUnknown = 0, eInitiator, eResponder };


// ---- This sketch's own message -----------------------------------------------------------
// First byte is the message type, numbered from cAppMsgTypeFirst. The version byte turns "the
// two boards were built from different revisions" into a rejected frame rather than bad data.

constexpr uint8_t cMsgPing     = ESPNowConnection::cAppMsgTypeFirst;
constexpr uint8_t cPingVersion = 1;

struct __attribute__((packed)) PingMsg
{
  uint8_t  msgType         = cMsgPing;
  uint8_t  protocolVersion = cPingVersion;
  uint32_t counter         = 0;
};


// ---- The link -----------------------------------------------------------------------------

class PingLink : public ESPNowConnection
{
  public:
    void requestPairing()
    {
      PairingRequestData pd;
      pd.device = eInitiator;
      copyStringToBuffer(pd.name, sizeof(pd.name), "Initiator");

      // The broadcast address is registered for the duration of the send only.
      sendToUnpairedAddress(cBroadcastAddress, &pd, sizeof(pd));
    }

    uint32_t received() const { return myReceived; }

  protected:
    void onPairingRequestMsg(const PairingRequestData& pd, const uint8_t src[6]) override
    {
      // Refuse a second device, but always answer our own peer asking again: it has lost the
      // pairing, and answering now saves both ends waiting out a timeout in disagreement.
      PeerInfo current;
      const bool fromOurPeer = getPeer(src, current);

      if (isPaired() == true && fromOurPeer == false)
        return;

      if (setPeer(src, pd.device, pd.name) == false)
        return;

      PairingResponseData response;
      response.device = eResponder;
      copyStringToBuffer(response.name, sizeof(response.name), "Responder");

      send(response, src);

      if (fromOurPeer == false)
      {
        Serial.print("paired with ");
        Serial.println(mac2string(src));
      }
    }

    void onPairingResponseMsg(const PairingResponseData& pd, const uint8_t src[6]) override
    {
      if (setPeer(src, pd.device, pd.name) == true)
      {
        Serial.print("paired with ");
        Serial.println(mac2string(src));
      }
    }

    // Runs in the WiFi task: keep it short and do not print from here.
    void onAppMsg(uint8_t msgType, const void* data, size_t len, const PeerInfo&) override
    {
      if (msgType != cMsgPing)
        return;

      PingMsg msg;

      // Rejects a frame of the wrong size - the length is whatever arrived over the air.
      if (copyFrameTo(data, len, msg) == false)
        return;

      if (msg.protocolVersion != cPingVersion)
        return;

      myReceived = msg.counter;
    }

  private:
    volatile uint32_t myReceived = 0;
};


PingLink      myLink;
uint32_t      myCounter       = 0;
unsigned long myLastPingMs    = 0;
unsigned long myLastPairingMs = 0;

// ----------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(500);

  if (myLink.begin(ESPNowConnection::cDefaultWifiChannel) == false)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  Serial.print(cIsInitiator ? "initiator, MAC " : "responder, MAC ");
  Serial.println(ESPNowConnection::mac2string(myLink.getMAC()));
}

// ----------------------------------------------------------------------------------------
void loop()
{
  myLink.removeLostPeer();

  const unsigned long now = millis();

  if (myLink.isPaired() == false)
  {
    if (now - myLastPairingMs < cPairingIntervalMs)
      return;

    myLastPairingMs = now;

    if (cIsInitiator == true)
      myLink.requestPairing();

    Serial.println("waiting for a peer...");
    return;
  }

  if (now - myLastPingMs < cHeartbeatIntervalMs)
    return;

  myLastPingMs = now;

  // Both boards send, so both keep the other's liveness timer fed. A link where only one end
  // transmits still needs the quiet end to send something of its own.
  PingMsg msg;
  msg.counter = ++myCounter;
  myLink.send(msg);

  Serial.print("sent ");
  Serial.print(myCounter);
  Serial.print(", last received ");
  Serial.println(myLink.received());
}
