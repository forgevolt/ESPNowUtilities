// ---- JoystickTransmitter ----------------------------------------------------------------
// Sends a joystick position and a fire button to a paired receiver.
//
// Flash this to one ESP32 and JoystickReceiver to another. This board is the initiator: it
// broadcasts pairing requests until the receiver answers, then sends to it alone.
//
// No hardware needed - the stick values are generated. Replace readJoystick() and
// readFireButton() to drive it from a real stick.

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
  int16_t x               = 0; // -512 .. 511, 0 at center
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

// The joystick frame doubles as this board's heartbeat: a peer is dropped after
// cPeerTimeoutMs of silence, so it has to arrive well inside that.
constexpr unsigned long cSendIntervalMs    = 50;    // 20 Hz
constexpr unsigned long cPairingIntervalMs = 250;   // also sets how fast a drop recovers

static_assert(cSendIntervalMs * 2 <= ESPNowConnection::cPeerTimeoutMs,
              "send interval must beat cPeerTimeoutMs with margin, or the peer is dropped");

enum EMyDevice : uint8_t { eUnknown = 0, eTransmitter, eReceiver };


// ---- The link -----------------------------------------------------------------------------

class JoystickLink : public ESPNowConnection
{
  public:
    void requestPairing()
    {
      PairingRequestData pd;
      pd.device = eTransmitter;
      copyStringToBuffer(pd.name, sizeof(pd.name), "Joystick");

      // The broadcast address is registered for the duration of the send only.
      sendToUnpairedAddress(cBroadcastAddress, &pd, sizeof(pd));
    }

    uint32_t acknowledged() const { return myAcknowledged; }

  protected:
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
      if (msgType != cMsgStatus)
        return;

      StatusMsg msg;

      // Rejects a frame of the wrong size - the length is whatever arrived over the air.
      if (copyFrameTo(data, len, msg) == false)
        return;

      if (msg.protocolVersion != cJoystickVersion)
        return;

      myAcknowledged = msg.received;
    }

  private:
    volatile uint32_t myAcknowledged = 0;
};


JoystickLink  myLink;
unsigned long myLastSendMs    = 0;
unsigned long myLastPairingMs = 0;


// ---- Stand-in for real hardware ---------------------------------------------------------
//   x = int16_t(analogRead(cPinX) / 4) - 512;   // 12-bit ADC -> -512..511
//   return digitalRead(cPinFire) == LOW;

void readJoystick(int16_t& x, int16_t& y)
{
  const float angle = float(millis()) * (2.0f * PI / 4000.0f); // a circle every four seconds

  x = int16_t(cosf(angle) * 340.0f);
  y = int16_t(sinf(angle) * 340.0f);
}

bool readFireButton()
{
  // Held until its own expiry rather than drawn afresh each call, which at the send rate would
  // rattle the button on and off twenty times a second.
  static bool          pressed   = false;
  static unsigned long changedAt = 0;
  static unsigned long holdMs    = 0;

  const unsigned long now = millis();

  // Elapsed time rather than a deadline, so the millis() wrap after 49.7 days does no harm.
  if (now - changedAt >= holdMs)
  {
    pressed   = !pressed;
    changedAt = now;
    holdMs    = pressed ? random(100, 600) : random(700, 3000);
  }

  return pressed;
}


// ----------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(500);

  randomSeed(esp_random()); // otherwise the button pattern repeats after every reset

  if (myLink.begin(ESPNowConnection::cDefaultWifiChannel) == false)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  Serial.print("transmitter, MAC ");
  Serial.println(ESPNowConnection::mac2string(myLink.getMAC()));
}

// ----------------------------------------------------------------------------------------
void loop()
{
  // The receiver's status frame is what keeps this from firing: the timeout measures silence
  // *from* the peer, so a one-way link times out however often this end transmits.
  myLink.removeLostPeer();

  const unsigned long now = millis();

  if (myLink.isPaired() == false)
  {
    if (now - myLastPairingMs < cPairingIntervalMs)
      return;

    myLastPairingMs = now;
    myLink.requestPairing();

    Serial.println("waiting for a receiver...");
    return;
  }

  if (now - myLastSendMs < cSendIntervalMs)
    return;

  myLastSendMs = now;

  // Read into locals: a packed field cannot be bound to a reference.
  int16_t x = 0;
  int16_t y = 0;
  readJoystick(x, y);

  JoystickMsg msg;
  msg.x    = x;
  msg.y    = y;
  msg.fire = readFireButton() ? 1 : 0;

  myLink.send(msg);

  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs >= 1000)   // once a second, so the monitor stays readable at 20 Hz
  {
    lastPrintMs = now;

    Serial.print("x ");
    Serial.print(msg.x);
    Serial.print("  y ");
    Serial.print(msg.y);
    Serial.print("  fire ");
    Serial.print(msg.fire ? "on " : "off");
    Serial.print("  receiver confirmed ");
    Serial.println(myLink.acknowledged());
  }
}
