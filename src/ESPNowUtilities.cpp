#include "ESPNowUtilities.h"

#include <stdint.h>


// Out-of-line definition of the in-class initialized constant. Required before C++17, where a
// constexpr static data member that is ODR-used - here it is passed as a pointer - still needs
// one. From C++17 on the in-class initializer is implicitly inline and this would be redundant.
#if __cplusplus < 201703L
constexpr uint8_t ESPNowConnection::cBroadcastAddress[6];
#endif


// Shortest gap between two reports of a frame the transport had to throw away, or of a send the
// peer did not acknowledge.
//
// All three reports run in the WiFi task, once per frame, at a rate the library does not
// control: other devices set it on the receive path, the sketch on the send path. Each report
// costs about a hundred bytes of serial and a String allocation; printing every one can starve
// the task badly enough to trip the watchdog.
//
// Reporting is kept, because a frame that cannot be placed is worth knowing about; it is the
// per-frame rate that is not survivable.
static constexpr unsigned long cDiscardReportIntervalMs = 1000;

// The single instance, through which the two static callbacks reach the object. Claimed by the
// first successful begin(), released by that instance's destructor.
static ESPNowConnection* theConnection = nullptr;

// ---- ESPNowConnection ------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
ESPNowConnection::ESPNowConnection()
: myChannel(cDefaultWifiChannel)
{
  // theConnection is claimed in begin(), not here, so constructing a second instance cannot
  // take the callbacks over from one already in use.
  myPeerMutex = xSemaphoreCreateMutex();
}

// ----------------------------------------------------------------------------------------
ESPNowConnection::~ESPNowConnection()
{
  // Only the instance that started ESP-NOW shuts it down. Any other - one that never ran
  // begin(), or whose begin() was refused - would otherwise stop ESP-NOW underneath the
  // instance still using it.
  if (myIsInitialized == true && theConnection == this)
  {
    // Unregister first: with the callbacks gone, no frame can reach dataRecvCB/dataSentCB and
    // dereference theConnection or take myPeerMutex while it is being destroyed.
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
  }

  if (theConnection == this)
    theConnection = nullptr;

  // Null if the constructor could not create it - begin() reports that case.
  if (myPeerMutex != nullptr)
    vSemaphoreDelete(myPeerMutex);
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::begin(uint8_t channel)
{
  // A second call would re-set the Wi-Fi channel and then fail in esp_now_init(), leaving the
  // channel changed but nothing else re-established.
  if (myIsInitialized == true)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> already initialized, ignoring");
    return true;
  }

  // Checked here rather than in the constructor: this object is a global, constructed before
  // Serial is up, so a failure there could not be reported. A null handle would make every
  // later xSemaphoreTake() undefined.
  if (myPeerMutex == nullptr)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> peer mutex could not be created");
    return false;
  }

  // One instance per program: the two static callbacks reach the object through theConnection.
  // A second instance is refused rather than allowed to take the callbacks over, which would
  // leave the first one running but deaf.
  if (theConnection != nullptr && theConnection != this)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> another ESPNowConnection is already in use");
    return false;
  }

  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Stop the station from going looking for an access point on its own. ESP-NOW rides on the
  // radio at one fixed channel, but a station that still has credentials stored from an earlier
  // sketch will keep trying to reconnect, and every attempt sweeps the radio across the band.
  // Each sweep takes it off this channel for the better part of a second - long enough for both
  // ends of a link to pass cPeerTimeoutMs and drop each other, over and over.
  //
  // Deliberately does not erase the stored credentials: that is the sketch's to decide, not the
  // transport's. WiFi.disconnect() is left with its default arguments for the same reason.
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();

  // Explicitly set Wi-Fi channel
  esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> failed to set Wi-Fi channel: ");
    Serial.println(err);
    return false;
  }

  myChannel = channel; // remembered so setPeer() can pin the peer to the same channel
                        // without an extra esp_wifi_get_channel() round-trip

  // Disable Wi-Fi power saving to prevent packet loss and latency spikes
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_wifi_get_mac(WIFI_IF_STA, myMACAddr) == ESP_OK)
  {
    Serial.print("Device MAC: ");
    Serial.print(mac2string(myMACAddr));
    Serial.print(" on channel ");
    Serial.println(channel);
  } 
  else 
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> failed to read MAC address");
    return false;
  }

  // Init ESP-NOW 
  if (esp_now_init() != ESP_OK) 
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> failed to initialize ESP-NOW");
    return false;
  }

  // Once ESP-NOW is successfully initialized, claim the callbacks and register them
  theConnection = this;
  esp_now_register_send_cb(dataSentCB);
  esp_now_register_recv_cb(dataRecvCB);

  myIsInitialized = true;
  return true;
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::sendToPeer(const void* data, size_t len)
{
  // A derived class can call this directly, and the failure report below reads data[0].
  if (data == nullptr || len == 0)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> no data to send");
    return false;
  }

  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  if (myHasPeer == false)
  {
    xSemaphoreGive(myPeerMutex);
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> no paired peer");
    return false;
  }

  uint8_t destAddr[6];
  memcpy(destAddr, myPeer.peer.peer_addr, 6);

  xSemaphoreGive(myPeerMutex);

  const esp_err_t result = esp_now_send(destAddr, static_cast<const uint8_t*>(data), len);

  if (result != ESP_OK)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> esp_now_send failed for msgType ");
    Serial.print(int(static_cast<const uint8_t*>(data)[0]));
    Serial.print(": ");
    Serial.println(result);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::send(const PairingRequestData& pd) 
{ 
  return sendToPeer(&pd, sizeof(pd)); 
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::send(const PairingResponseData& pd, const uint8_t* mac)
{
  // The response goes to the device that sent the request, so the caller passes its MAC
  // address.
  //
  // That requester may or may not already be registered - usually it is, because the caller
  // has just made it our peer. sendToUnpairedAddress() handles both cases.
  return sendToUnpairedAddress(mac, &pd, sizeof(PairingResponseData));
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::isPaired() const
{
  xSemaphoreTake(myPeerMutex, portMAX_DELAY);
  bool paired = myHasPeer;
  xSemaphoreGive(myPeerMutex);

  return paired;
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::getPeerInfo(PeerInfo& outInfo) const
{
  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  if (myHasPeer == false)
  {
    xSemaphoreGive(myPeerMutex);
    return false;
  }

  outInfo = myPeer; // copy out while still holding the lock

  xSemaphoreGive(myPeerMutex);
  return true;
}

// ----------------------------------------------------------------------------------------
uint64_t ESPNowConnection::mac2uint64(const uint8_t mac[6]) 
{
  uint64_t id = 0;
  for (int i = 0; i < 6; ++i) 
  {
    id = (id << 8) | mac[i];
  }
  return id;
}

// ----------------------------------------------------------------------------------------
String ESPNowConnection::mac2string(const uint8_t mac[6])
{
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  return String(buf);
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::copyStringToBuffer(char* dest, std::size_t size, const String& src)
{
  snprintf(dest, size, "%s", src.c_str());
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::copyFieldToCString(char* dest, std::size_t destSize, const char* field, std::size_t fieldLen)
{
  if (dest == nullptr || destSize == 0)
    return;

  if (field == nullptr)
  {
    dest[0] = '\0';
    return;
  }

  const std::size_t n = (fieldLen < destSize - 1) ? fieldLen : destSize - 1;

  memcpy(dest, field, n);
  dest[n] = '\0';
}

// ----------------------------------------------------------------------------------------
const uint8_t* ESPNowConnection::getMAC() const
{
  return myMACAddr;
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::dataRecvCB(const esp_now_recv_info_t* info, const uint8_t* incomingData, int len)
{
  if (theConnection == nullptr || info == nullptr)
    return;

  // Every message begins with msgType then protocolVersion, so both can be validated before
  // any struct-specific parsing.
  if (len < 2)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> frame too short: ");
    Serial.print(len);
    Serial.println(" bytes");
    return;
  }

  const uint8_t msgType = incomingData[0];
  const uint8_t version = incomingData[1];

  // Only the pairing frames are the transport's to validate. Application payloads carry their
  // own version in the same byte, checked by whoever owns them - the transport must not reject
  // a frame whose format it does not define.
  const bool isPairingFrame = (msgType == ePairingRequest || msgType == ePairingResponse);

  if (isPairingFrame == true && version != cLinkProtocolVersion)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> link protocol version mismatch: received ");
    Serial.print(int(version));
    Serial.print(", expected ");
    Serial.print(int(cLinkProtocolVersion));
    Serial.println(" - the two ends were built from different revisions of ESPNowUtilities.h");
    return;
  }
  
  switch (msgType)
  {
    case ePairingRequest:
      {
        if (len != sizeof(PairingRequestData))
        {
          Serial.print(__PRETTY_FUNCTION__);
          Serial.print(" -> invalid length for PairingRequestData: ");
          Serial.print(len);
          Serial.print(", expected ");
          Serial.println(int(sizeof(PairingRequestData)));
          return;
        }
        PairingRequestData pd;
        memcpy(&pd, incomingData, sizeof(PairingRequestData));

        // The sender is identified by the frame's source address rather than by a MAC in the
        // payload, which the sender controls. The source address can still be forged on an
        // unencrypted link.
        theConnection->onPairingRequestMsg(pd, info->src_addr);
      }
      break;

    case ePairingResponse:
      {
        if (len != sizeof(PairingResponseData))
        {
          Serial.print(__PRETTY_FUNCTION__);
          Serial.print(" -> invalid length for PairingResponseData: ");
          Serial.print(len);
          Serial.print(", expected ");
          Serial.println(int(sizeof(PairingResponseData)));
          return;
        }
        PairingResponseData pd;
        memcpy(&pd, incomingData, sizeof(PairingResponseData));
        theConnection->onPairingResponseMsg(pd, info->src_addr);   // see ePairingRequest above
      }
      break;

    default:
      if (msgType >= cAppMsgTypeFirst)
      {
        // Application frames are accepted only from the peer we are paired with. getPeer()
        // succeeding is, by construction, that confirmation. Without this, any device in radio
        // range could inject application frames, even while we are paired, because pairing
        // state is not otherwise consulted on this path.
        PeerInfo pi;
        if (theConnection->getPeer(info->src_addr, pi) == false)
        {
          // Rate-limited - see cDiscardReportIntervalMs. A sender that has not noticed the
          // pairing is gone keeps these arriving at its full rate.
          static unsigned long lastReportMs = 0;
          static uint32_t      suppressed   = 0;

          const unsigned long now = millis();

          if (now - lastReportMs >= cDiscardReportIntervalMs)
          {
            lastReportMs = now;

            Serial.print(__PRETTY_FUNCTION__);
            Serial.print(" -> app frame ");
            Serial.print(int(msgType));
            Serial.print(" from ");
            Serial.print(mac2string(info->src_addr));
            Serial.print(" ignored: not our paired peer");

            if (suppressed > 0)
            {
              Serial.print(" (");
              Serial.print(suppressed);
              Serial.print(" more since the last report)");
            }

            Serial.println();
            suppressed = 0;
          }
          else
          {
            suppressed++;
          }

          return;
        }

        // The length is passed through unchecked: only the application knows how long its own
        // frames are, and copyFrameTo() rejects a mismatch when it takes its copy.
        theConnection->msgReceived(info->src_addr);
        theConnection->onAppMsg(msgType, incomingData, static_cast<size_t>(len), pi);
        return;
      }

      // Runs in WiFi task context. Any foreign ESP-NOW frame on this channel lands here, and a
      // device transmitting nearby sets the rate, so this is rate-limited the same way - see
      // cDiscardReportIntervalMs.
      {
        static unsigned long lastReportMs = 0;
        static uint32_t      suppressed   = 0;

        const unsigned long now = millis();

        if (now - lastReportMs >= cDiscardReportIntervalMs)
        {
          lastReportMs = now;

          Serial.print(__PRETTY_FUNCTION__);
          Serial.print(" -> unknown msgType ");
          Serial.print(int(msgType));
          Serial.print(" - foreign traffic on this channel, or a protocol mismatch");

          if (suppressed > 0)
          {
            Serial.print(" (");
            Serial.print(suppressed);
            Serial.print(" more since the last report)");
          }

          Serial.println();
          suppressed = 0;
        }
        else
        {
          suppressed++;
        }
      }
      break;      
  }
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::dataSentCB(const esp_now_send_info_t* info, esp_now_send_status_t status)
{
  if (theConnection == nullptr || info == nullptr)
    return;

  if (status != ESP_NOW_SEND_SUCCESS)
  {
    // Deliberately does not drop the peer. A failed send means one frame went un-ACKed, which
    // is routine at 2.4 GHz. Liveness is decided solely by removeLostPeer(), which
    // keys on how long it has been since the peer was last *heard from* - the direction that
    // tells us whether the peer is still there.
    //
    // Rate-limited - see cDiscardReportIntervalMs. A peer that has gone out of range fails
    // every frame the sketch sends until removeLostPeer() drops it.
    static unsigned long lastReportMs = 0;
    static uint32_t      suppressed   = 0;

    const unsigned long now = millis();

    if (now - lastReportMs >= cDiscardReportIntervalMs)
    {
      lastReportMs = now;

      Serial.print(__PRETTY_FUNCTION__);
      Serial.print(" -> send to ");
      Serial.print(mac2string(info->des_addr));
      Serial.print(" not acknowledged (status ");
      Serial.print(int(status));
      Serial.print(")");

      if (suppressed > 0)
      {
        Serial.print(" (");
        Serial.print(suppressed);
        Serial.print(" more since the last report)");
      }

      Serial.println();
      suppressed = 0;
    }
    else
    {
      suppressed++;
    }
  }
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::setPeer(const uint8_t mac[6], uint8_t device, const char name[cNameLen])
{
  uint64_t id = mac2uint64(mac);

  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  if (myHasPeer == true && myPeer.id == id)
  {
    // Duplicate pairing request from the device we're already paired with - nothing to
    // do, but not an error either: the caller can still (re-)send its response.
    //
    // It does count as a sign of life, though. A peer only asks again because it has lost the
    // pairing, and that is precisely the moment this end must not go on to time it out as well:
    // it has stopped sending application frames, so without this the timer runs out while the
    // two are in the middle of agreeing to start over.
    myPeer.timeLastMsgFromPeer = millis();

    xSemaphoreGive(myPeerMutex);
    return true;
  }

  if (myHasPeer == true)
  {
    // A different peer is already set. Single-peer design: refuse to replace it
    // implicitly - it must be cleared by removeLostPeer()'s inbound-silence timeout first.
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> already paired with ");
    Serial.print(mac2string(myPeer.peer.peer_addr));
    Serial.print(" -> ignoring ");
    Serial.println(mac2string(mac));
    xSemaphoreGive(myPeerMutex);
    return false;
  }

  // Defensive: esp_now_add_peer() below returns ESP_ERR_ESPNOW_EXIST if this address is
  // already in the driver's peer table, so clear it first to make the add idempotent.
  // The return value is deliberately discarded - ESP_ERR_ESPNOW_NOT_FOUND is the normal,
  // expected outcome here (usually there is nothing to delete), not a failure.
  esp_now_del_peer(mac);

  PeerInfo pi; 

  pi.id     = id;
  pi.device = device;
  pi.timeLastMsgFromPeer = millis();
  copyFieldToCString(pi.name, sizeof(pi.name), name, cNameLen);

  memset(&pi.peer, 0, sizeof(esp_now_peer_info_t));
  pi.peer.channel = myChannel; // Lock peer to the channel that was set in begin()
  pi.peer.encrypt = false;
  memcpy(pi.peer.peer_addr, mac, 6);  
  
  if (esp_now_add_peer(&pi.peer) != ESP_OK)
  {
    xSemaphoreGive(myPeerMutex);
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" -> failed to add peer");
    return false;
  }
  
  myPeer    = pi;
  myHasPeer = true;

  xSemaphoreGive(myPeerMutex);
  return true;
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::getPeer(const uint8_t mac[6], PeerInfo& outInfo) const
{
  uint64_t id = mac2uint64(mac); 

  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  if (myHasPeer == false || myPeer.id != id)
  {
    xSemaphoreGive(myPeerMutex);
    return false;
  }

  outInfo = myPeer; // copy out while still holding the lock

  xSemaphoreGive(myPeerMutex);
  return true;
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::removePeer(const uint8_t mac[6])
{
  uint64_t id = mac2uint64(mac);

  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  if (myHasPeer == true && myPeer.id == id)
  {
    esp_now_del_peer(myPeer.peer.peer_addr);
    myHasPeer = false;
  }

  xSemaphoreGive(myPeerMutex);
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::removeLostPeer()
{
  uint8_t lostAddr[6];
  bool    removed = false;

  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  // millis() must be read inside the lock. msgReceived() stamps timeLastMsgFromPeer under this
  // same mutex, so a reading taken beforehand can end up older than the stamp - and since the
  // subtraction below is unsigned, that underflows to a huge value and drops a peer that has
  // just been heard from.
  const unsigned long now = millis();

  if (myHasPeer == true && (now - myPeer.timeLastMsgFromPeer > cPeerTimeoutMs))
  {
    memcpy(lostAddr, myPeer.peer.peer_addr, sizeof(lostAddr));
    // Inside the lock on purpose: released first, a re-pairing setPeer() in the WiFi task could
    // re-add this address in between, and this delete would then remove the new entry.
    esp_now_del_peer(myPeer.peer.peer_addr);
    myHasPeer = false;
    removed   = true;
  }

  xSemaphoreGive(myPeerMutex);

  // Reported outside the lock: mac2string() allocates and Serial blocks when the TX buffer is
  // full. dataRecvCB takes this same mutex in WiFi task context, so holding it across either
  // would stall frame processing.
  if (removed == true)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> peer ");
    Serial.print(mac2string(lostAddr));
    Serial.println(" timed out, removed");
  }
}

// ----------------------------------------------------------------------------------------
void ESPNowConnection::msgReceived(const uint8_t mac[6])
{
  uint64_t id = mac2uint64(mac); 

  xSemaphoreTake(myPeerMutex, portMAX_DELAY);

  if (myHasPeer == true && myPeer.id == id)
    myPeer.timeLastMsgFromPeer = millis();

  xSemaphoreGive(myPeerMutex);
}

// ----------------------------------------------------------------------------------------
bool ESPNowConnection::sendToUnpairedAddress(const uint8_t mac[6], const void* data, size_t len)
{
  // Deliberately does not touch myPeer/myHasPeer or myPeerMutex at all - this is a
  // one-off send to an address (typically the broadcast address) that is not, and
  // should not become, our tracked peer.
  //
  // An address the driver already knows - our tracked peer, or one the sketch registered - is
  // sent to as it is and left registered. Unregistering the tracked peer here would make every
  // later sendToPeer() fail while myHasPeer stays true, with nothing to ever clear it.
  const bool wasRegistered = esp_now_is_peer_exist(mac);

  if (wasRegistered == false)
  {
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    peer.channel = myChannel;
    peer.encrypt = false;
    memcpy(peer.peer_addr, mac, 6);

    if (esp_now_add_peer(&peer) != ESP_OK)
    {
      Serial.print(__PRETTY_FUNCTION__);
      Serial.print(" -> failed to register temporary peer ");
      Serial.println(mac2string(mac));
      return false;
    }
  }

  esp_err_t result = esp_now_send(mac, (const uint8_t*)data, len);

  // Unregister only what was registered above. The result is discarded: there is no useful
  // recovery from a failed unregister.
  if (wasRegistered == false)
    esp_now_del_peer(mac);

  if (result != ESP_OK)
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" -> esp_now_send failed: ");
    Serial.println(result);
    return false;
  }

  return true;
}
