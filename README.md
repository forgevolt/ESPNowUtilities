# ESPNowUtilities

Paired, single-peer ESP-NOW links between two ESP32 boards.

ESP-NOW gives you connectionless frames between MAC addresses and nothing else — no discovery,
no notion of a session, no idea whether the other end is still there. This library adds the
part you would otherwise write again for every project:

- **Discovery and pairing.** One board broadcasts a request, the other answers, and both record
  each other. Each side carries a device kind and a name so you can tell what you paired with.
- **A single tracked peer.** Deliberately one, not many: a second device asking to pair while
  you are already paired is refused rather than silently accepted. The peer is dropped when
  nothing has been heard from it for `cPeerTimeoutMs`, so a link that goes away is noticed.
- **Sender authentication.** Application frames are delivered only if they came from the paired
  peer, keyed on the frame's source address rather than anything inside the payload.
- **Thread safety.** ESP-NOW callbacks run in WiFi task context. Peer state is behind a mutex,
  and the radio is never called while that mutex is held.

What it deliberately does **not** do is define your messages. The library routes any frame
whose first byte is at or above `cAppMsgTypeFirst` and hands it to you unopened. You choose the
structs, the field order and the versioning.

## Installing

**Library Manager** — in the Arduino IDE, *Sketch → Include Library → Manage Libraries*, search for
`ESPNowUtilities`, install.

**From this repository** — *Code → Download ZIP*, then *Sketch → Include Library → Add .ZIP
Library*. Use this if you want a version that has not been released yet.

Requires the ESP32 core; there are no other dependencies.

## Using it

Derive from `ESPNowConnection`, define a message, and override `onAppMsg()`:

```cpp
#include <ESPNowUtilities.h>

constexpr uint8_t cMsgPing = ESPNowConnection::cAppMsgTypeFirst;

struct __attribute__((packed)) PingMsg
{
  uint8_t  msgType         = cMsgPing;
  uint8_t  protocolVersion = 1;
  uint32_t counter         = 0;
};

class MyLink : public ESPNowConnection
{
  protected:
    void onAppMsg(uint8_t msgType, const void* data, size_t len, const PeerInfo&) override
    {
      PingMsg msg;
      if (msgType == cMsgPing && copyFrameTo(data, len, msg) == true)
        Serial.println(msg.counter);
    }
};
```

`send(msg)` sends any trivially-copyable struct to the paired peer. `copyFrameTo()` takes a
size-checked copy of a received frame — the length is whatever arrived over the air, so it is
checked rather than trusted.

Everything the library declares is nested inside `ESPNowConnection`, so it adds exactly one name
to the global namespace. A derived class sees `PeerInfo`, `PairingRequestData`, `cNameLen` and
the rest unqualified; anywhere else they need the `ESPNowConnection::` prefix.

## Examples

`PairAndPing` — two boards find each other and exchange a counter. The same sketch goes on both;
one constant at the top decides which one initiates.

`JoystickTransmitter` / `JoystickReceiver` — a controller sending a stick position and a button
to a receiver, with the receiver answering. Needs no hardware: the stick values are generated,
so two bare boards show the link working.

Both examples send at several times the rate a peer is dropped at. That is not incidental — see
below.

## Keeping a link alive

A peer is forgotten when nothing has been heard **from** it for `cPeerTimeoutMs`. Two things
follow, and both examples show them:

- Whatever you send has to arrive comfortably inside that window. Sending at the same period as
  the timeout puts every frame on the deadline, and the link comes and goes. Both examples carry
  a `static_assert` tying their send interval to `cPeerTimeoutMs`, so retuning one without the
  other fails the build instead of the radio.
- A one-way link does not stay up. The timeout measures silence from the peer, so the quiet end
  has to send something of its own however often the talking end transmits. In
  `JoystickReceiver` that is the status frame going back the other way.

## Pairing is a policy, not a mechanism

The library gives you the pieces — `sendToUnpairedAddress()`, `setPeer()`, and the
`onPairingRequestMsg()` / `onPairingResponseMsg()` hooks — but does not decide which side
initiates. That is deliberate: which board broadcasts and which waits depends on your product,
not on the transport. `PairAndPing` shows both halves in about thirty lines.

## Protocol versions

Two independent version bytes:

- `cLinkProtocolVersion` covers the pairing frames and is checked by the library.
- Your payloads carry their own, checked by you.

They are separate so that a change to your message format does not invalidate the handshake, and
a library upgrade does not force your wire format to rev.

## Licence

MIT — see [LICENSE](LICENSE).
