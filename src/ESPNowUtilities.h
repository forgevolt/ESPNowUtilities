#pragma once

// A paired, single-peer ESP-NOW link between two ESP32 boards
//
// This code is based on the following excellent tutorials:
//   https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
//   https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
//   https://randomnerdtutorials.com/esp-now-auto-pairing-esp32-esp8266/
//   https://randomnerdtutorials.com/esp32-esp-now-encrypted-messages/

#include <Arduino.h>   // String
#include <cstddef>     // size_t / std::size_t
#include <cstring>     // memcpy
#include <type_traits> // std::is_trivially_copyable

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ----------------------------------------------------------------------------------------
// ESPNowConnection: Base class for both ends of the link. Derive from it and override
// onAppMsg() and the pairing hooks.
// ----------------------------------------------------------------------------------------

class ESPNowConnection
{
  public:
    // ---- Wire protocol -----------------------------------------------------------------

    // Version of the pairing frames below, sent as their second byte and checked on receipt, so
    // two ends built from different revisions of this header fail with a logged mismatch
    // instead of silently exchanging misinterpreted data. Application payloads carry a
    // version of their own, so the two evolve independently.
    static constexpr uint8_t cLinkProtocolVersion = 1;

    // Message types below this belong to the transport; applications number theirs from here up.
    static constexpr uint8_t cAppMsgTypeFirst = 0xA3;

    enum EMsgType : uint8_t
    {
      // Deliberately not starting at 0. msgType is the first byte of every frame: with values
      // 0..3, an all-zero buffer would parse as a valid pairing request, and any foreign ESP-NOW
      // frame whose first byte happened to be 0..3 could be accepted. 0xA1.. makes both an
      // all-zero and an all-0xFF frame invalid.
      ePairingRequest  = 0xA1,
      ePairingResponse = 0xA2
    };


    // Width of the device-name field in the pairing frames.
    static constexpr size_t cNameLen = 16;

    // ---- Information exchanged during pairing
    struct __attribute__((packed)) PairingRequestData
    {
      uint8_t msgType         = ePairingRequest;
      uint8_t protocolVersion = cLinkProtocolVersion;
      uint8_t device          = 0;          // Application-defined device kind; 0 = unspecified
      char    name[cNameLen]  = {};         // Name of the requesting device
    };

    struct __attribute__((packed)) PairingResponseData
    {
      uint8_t msgType         = ePairingResponse;
      uint8_t protocolVersion = cLinkProtocolVersion;
      uint8_t device          = 0;          // Application-defined device kind; 0 = unspecified
      char    name[cNameLen]  = {};         // Name of the responding device
    };


    // The wire format is frozen deliberately. If either of these fires, the layout has changed and
    // cLinkProtocolVersion above must be bumped - and both ends reflashed.
    static_assert(sizeof(PairingRequestData)  ==  19, "PairingRequestData wire format changed");
    static_assert(sizeof(PairingResponseData) ==  19, "PairingResponseData wire format changed");

    static constexpr uint8_t cDefaultWifiChannel = 1;

    // ESP-NOW's broadcast destination. An initiator sends its pairing request here because the
    // peer's address is not yet known. Kept with the transport so every sketch uses the same six
    // bytes rather than declaring a copy of its own.
    static constexpr uint8_t cBroadcastAddress[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    // A peer is considered lost if nothing has been received from it for this long, at which point
    // pairing has to be established again. Keep any staleness check of your own data shorter than
    // this: a brief dropout then stops the application without destroying the pairing, and the
    // link resumes when it recovers instead of needing a full handshake.
    static constexpr unsigned long cPeerTimeoutMs = 1000;

    // ---- The peer this connection tracks
    struct PeerInfo
    {
      uint64_t id;                       // the 6 MAC bytes packed losslessly into 48 of the 64 bits
      unsigned long timeLastMsgFromPeer; // millis() when the last message from this peer arrived
      esp_now_peer_info_t peer;

      uint8_t device;
      char    name[cNameLen + 1]; // NUL-terminated; one wider than the wire field it comes from
    };

    // PeerInfo is copied in and out while myPeerMutex is held, so it must not allocate.
    static_assert(std::is_trivially_copyable<PeerInfo>::value,
                  "PeerInfo is copied under myPeerMutex - it must stay trivially copyable");

    ESPNowConnection();
    virtual ~ESPNowConnection();

    // Call from setup(). Defaults to channel 1.
    virtual bool begin(uint8_t channel = cDefaultWifiChannel);

    // Send a pairing request to the paired peer, or a pairing response to the device at mac.
    virtual bool send(const PairingRequestData& pd);
    virtual bool send(const PairingResponseData& pd, const uint8_t* mac);
    // Sends any application message to the paired peer. The first byte of T must be its
    // message type, and that type must be >= cAppMsgTypeFirst; the transport checks that byte
    // and forwards the rest unread. Returns false (and logs) if the type is out of range.
    template<typename T>
    bool send(const T& msg)
    {
      static_assert(std::is_trivially_copyable<T>::value,
                    "an ESP-NOW message must be trivially copyable");
      // The receiver drops any frame shorter than msgType + version as too short.
      static_assert(sizeof(T) >= 2,
                    "an ESP-NOW message needs at least a message type and a version byte");
      static_assert(sizeof(T) <= ESP_NOW_MAX_DATA_LEN_V2,
                    "an ESP-NOW message must not exceed ESP_NOW_MAX_DATA_LEN_V2 bytes");

      // A lower type would reach the peer as a pairing frame or as foreign traffic.
      const uint8_t msgType = static_cast<const uint8_t*>(static_cast<const void*>(&msg))[0];
      if (msgType < cAppMsgTypeFirst)
      {
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" -> msgType ");
        Serial.print(int(msgType));
        Serial.println(" is below cAppMsgTypeFirst, not sent");
        return false;
      }

      return sendToPeer(&msg, sizeof(T));
    }

    // Copies a received frame into out, but only if its length matches T exactly. Copies
    // rather than casting in place: the buffer belongs to the WiFi task and is valid only for
    // the duration of the callback.
    template<typename T>
    static bool copyFrameTo(const void* data, size_t len, T& out)
    {
      static_assert(std::is_trivially_copyable<T>::value,
                    "an ESP-NOW message must be trivially copyable");

      if (len != sizeof(T))
        return false;

      memcpy(&out, data, sizeof(T));
      return true;
    }

    // Whether we currently have a paired peer.
    bool isPaired() const;

    // Get information of the current peer. Returns a copy so callers can't be handed a
    // dangling reference if the peer is cleared concurrently, e.g. by removeLostPeer() in
    // another task. Returns false if there is no peer.
    bool getPeerInfo(PeerInfo& outInfo) const;

    // Clears the peer when nothing has been heard from it for cPeerTimeoutMs: the link is lost.
    // Call it regularly from loop(); nothing else drops a silent peer.
    void removeLostPeer();
    void msgReceived(const uint8_t mac[6]);

    // Utilities
    const uint8_t* getMAC() const;
    static uint64_t mac2uint64(const uint8_t mac[6]);
    static String mac2string(const uint8_t mac[6]);
    static void copyStringToBuffer(char* dest, std::size_t size, const String& src); // Safely copies an Arduino String to a destination buffer

    // Copies a fixed-length wire field into a NUL-terminated C string. Wire fields occupy
    // exactly their declared length and carry no guaranteed terminator, so printing one or
    // passing it to String() reads past the end of the frame. dest needs fieldLen + 1 bytes.
    static void copyFieldToCString(char* dest, std::size_t destSize, const char* field, std::size_t fieldLen);

  protected:
    // A pairing message may come from a device we have no record of, so the frame's source
    // address is what identifies it.
    virtual void onPairingRequestMsg (const PairingRequestData&,  const uint8_t /*src*/[6]) {}
    virtual void onPairingResponseMsg(const PairingResponseData&, const uint8_t /*src*/[6]) {}

    // Application frames - any msgType >= cAppMsgTypeFirst - dispatched only after dataRecvCB
    // has confirmed the sender is our peer. The PeerInfo it looked up is passed on, so the
    // handler needs no lookup of its own.
    //
    // data points into the WiFi task's receive buffer and is valid only for this call; use
    // copyFrameTo() to take a checked copy of it.
    virtual void onAppMsg(uint8_t /*msgType*/, const void* /*data*/, size_t /*len*/,
                          const PeerInfo& /*peer*/) {}

    // Sends len bytes to the currently paired peer. Copies the destination MAC out under
    // myPeerMutex and releases it before calling esp_now_send(), so the radio call never runs
    // with the lock held. Returns false (and logs) if there is no data or no peer, or if the
    // send fails.
    bool sendToPeer(const void* data, size_t len);

    // Sets the single peer this connection communicates with. Fails (returns false) if
    // a *different* peer is already set - single-peer design deliberately does not
    // silently replace an existing pairing. The existing peer must be cleared first, by
    // removeLostPeer()'s timeout, before a new device can pair.
    // If mac already matches the current peer, this is a harmless no-op that returns
    // true, so a duplicate pairing request from an already-paired device still gets a
    // response instead of being silently dropped.
    // name is a fixed-width wire field and need not be NUL-terminated - it is copied bounded.
    bool setPeer(const uint8_t mac[6], uint8_t device, const char name[cNameLen]);

    // Returns a copy of the current peer's info, but only if mac matches it. Returns
    // false otherwise (including when there is no current peer at all) - so a
    // successful call is, by construction, confirmation that mac *is* our peer; no
    // separate "is this a registered peer" check is needed before calling this.
    bool getPeer(const uint8_t mac[6], PeerInfo& outInfo) const;

    // Clears the current peer, but only if mac matches it (a different mac is ignored).
    // Not used by the library itself; available to derived classes.
    void removePeer(const uint8_t mac[6]);

    // One-off send to an address that is not (and should not become) our tracked peer, e.g. an
    // initiator's broadcast pairing request. An unknown address is registered for the send only;
    // one that is already registered, including the tracked peer, is used as it is.
    // Does not read or modify the tracked peer in any way.
    bool sendToUnpairedAddress(const uint8_t mac[6], const void* data, size_t len);

  private:
    // Called by ESP-NOW when a frame arrives
    static void dataRecvCB(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len);
    // Called by ESP-NOW when a send has completed
    static void dataSentCB(const esp_now_send_info_t* info, esp_now_send_status_t status);

  protected:
    uint8_t myMACAddr[6]; // MAC of the device

  private:
    // Written once in begin(), before any task that reads it exists; read afterwards from
    // setPeer()/sendToUnpairedAddress(), which can run in WiFi task context. Unsynchronized
    // by design - there is no write after start-up.
    uint8_t myChannel;    // Wi-Fi channel set in begin(), reused by setPeer()

    bool myIsInitialized = false; // begin() succeeded; guards against a second call

    // The single peer this connection communicates with, if any. All access must go
    // through the methods above, which take myPeerMutex internally - do not read
    // myHasPeer/myPeer directly from a derived class.
    bool myHasPeer = false;
    PeerInfo myPeer{};

    mutable SemaphoreHandle_t myPeerMutex;
};
