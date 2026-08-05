// ---- JoystickReceiver -------------------------------------------------------------------
// Takes a joystick position and a fire button from a paired transmitter and prints them.
//
// Flash this to one ESP32 and JoystickTransmitter to another. This board waits: it answers the
// transmitter's pairing request, then accepts frames from that board alone.
//
// It also sends a status frame back. That is not decoration: the liveness timeout at each end
// measures silence *from* the other end, so a one-way link drops its peer no matter how often
// the talking end sends.

#include <ESPNowUtilities.h>

// ---- Wire protocol ----------------------------------------------------------------------
// Both sketches must agree on these byte for byte. Repeated in each sketch because an Arduino
// example is a self-contained folder; in your own project put them in one shared header.

constexpr uint8_t cMsgJoystick     = ESPNowConnection::cAppMsgTypeFirst;
constexpr uint8_t cMsgStatus       = ESPNowConnection::cAppMsgTypeFirst + 1;
constexpr uint8_t cJoystickVersion = 1;

struct __attribute__((packed)) JoystickMsg
{
  uint8_t msgType         = cMsgJoystick;
  uint8_t protocolVersion = cJoystickVersion;
  int16_t x               = 0; // -512 .. 511, 0 at centre
  int16_t y               = 0;
  uint8_t fire            = 0; // 0 or 1
};

struct __attribute__((packed)) StatusMsg
{
  uint8_t  msgType         = cMsgStatus;
  uint8_t  protocolVersion = cJoystickVersion;
  uint32_t received        = 0;
};

static_assert(sizeof(JoystickMsg) == 7, "JoystickMsg wire format changed - bump the version");
static_assert(sizeof(StatusMsg)   == 6, "StatusMsg wire format changed - bump the version");

// The status frame doubles as this board's heartbeat - see cPeerTimeoutMs.
constexpr unsigned long cStatusIntervalMs = 100;

static_assert(cStatusIntervalMs * 2 <= ESPNowConnection::cPeerTimeoutMs,
              "status interval must beat cPeerTimeoutMs with margin, or the peer is dropped");

// How long the stick may go unheard before this board stops acting on the last value. Shorter
// than cPeerTimeoutMs, so a brief dropout stops the machine without tearing down the pairing.
constexpr unsigned long cControlStaleMs = 300;

static_assert(cControlStaleMs < ESPNowConnection::cPeerTimeoutMs,
              "control should go stale before the pairing is given up, not after");

enum EMyDevice : uint8_t { eUnknown = 0, eTransmitter, eReceiver };


// ---- The link -----------------------------------------------------------------------------

class JoystickLink : public ESPNowConnection
{
  public:
    // Taken under a critical section: onAppMsg() writes these from the WiFi task while loop()
    // reads them, and they belong together. noInterrupts() would not do - it silences the
    // calling core only, and the WiFi task may be on the other one.
    bool latest(int16_t& x, int16_t& y, bool& fire, unsigned long& atMs) const
    {
      taskENTER_CRITICAL(&myLock);
      x    = myX;
      y    = myY;
      fire = myFire;
      atMs = myLastMsgMs;
      const bool everReceived = myEverReceived;
      taskEXIT_CRITICAL(&myLock);

      return everReceived;
    }

    uint32_t received() const
    {
      taskENTER_CRITICAL(&myLock);
      const uint32_t n = myReceived;
      taskEXIT_CRITICAL(&myLock);

      return n;
    }

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
      response.device = eReceiver;
      copyStringToBuffer(response.name, sizeof(response.name), "Receiver");

      send(response, src);

      if (fromOurPeer == false)
      {
        Serial.print("paired with ");
        Serial.println(mac2string(src));
      }
    }

    // Runs in the WiFi task: keep it short and do not print from here.
    void onAppMsg(uint8_t msgType, const void* data, size_t len, const PeerInfo&) override
    {
      if (msgType != cMsgJoystick)
        return;

      JoystickMsg msg;

      // Rejects a frame of the wrong size - the length is whatever arrived over the air.
      if (copyFrameTo(data, len, msg) == false)
        return;

      if (msg.protocolVersion != cJoystickVersion)
        return;

      taskENTER_CRITICAL(&myLock);
      myX            = msg.x;
      myY            = msg.y;
      myFire         = (msg.fire != 0);
      myLastMsgMs    = millis();
      myEverReceived = true;
      myReceived++;
      taskEXIT_CRITICAL(&myLock);
    }

  private:
    mutable portMUX_TYPE myLock = portMUX_INITIALIZER_UNLOCKED;

    int16_t       myX            = 0;
    int16_t       myY            = 0;
    bool          myFire         = false;
    unsigned long myLastMsgMs    = 0;
    bool          myEverReceived = false;
    uint32_t      myReceived     = 0;
};


JoystickLink  myLink;
unsigned long myLastStatusMs = 0;

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

  Serial.print("receiver, MAC ");
  Serial.println(ESPNowConnection::mac2string(myLink.getMAC()));
}

// ----------------------------------------------------------------------------------------
void loop()
{
  myLink.removeLostPeer();

  const unsigned long now = millis();

  if (myLink.isPaired() == false)
  {
    static unsigned long lastWaitMs = 0;
    if (now - lastWaitMs >= 1000)
    {
      lastWaitMs = now;
      Serial.println("waiting for a transmitter...");
    }
    return;
  }

  // Sending this back is what keeps the transmitter from dropping us.
  if (now - myLastStatusMs >= cStatusIntervalMs)
  {
    myLastStatusMs = now;

    StatusMsg status;
    status.received = myLink.received();
    myLink.send(status);
  }

  int16_t x = 0, y = 0;
  bool fire = false;
  unsigned long atMs = 0;

  if (myLink.latest(x, y, fire, atMs) == false)
    return; // paired, but nothing has arrived yet

  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs < 1000)   // once a second, so the monitor stays readable
    return;

  lastPrintMs = now;

  // Whatever the stick drives should be released here rather than left at its last commanded
  // value, so a dropout stops the machine instead of running it on blind.
  if (now - atMs > cControlStaleMs)
  {
    Serial.println("stick stale - holding off");
    return;
  }

  Serial.print("x ");
  Serial.print(x);
  Serial.print("  y ");
  Serial.print(y);
  Serial.print("  fire ");
  Serial.print(fire ? "on " : "off");
  Serial.print("  taken ");
  Serial.println(myLink.received());
}
