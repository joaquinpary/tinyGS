/*
  MQTT_Pluto.h - Forward station telemetry to a secondary MQTT broker

  Secondary MQTT client, publish-only. Sends a configurable broker the same
  things the station sends to TinyGS (every received packet) plus a retained
  "tracking" message stating which satellite is currently being listened to,
  and a periodic "health" message with receiver diagnostics.

  Deliberately does NOT subscribe to anything: it adds no new command surface
  over the hardware, and an outage of the secondary broker can never affect
  the path to TinyGS.

  Copyright (C) 2026 Joaquin Pary

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef MQTT_PLUTO_H
#define MQTT_PLUTO_H

#include "../ConfigManager/ConfigManager.h"
#include "../Status.h"
#include "MQTT_Client.h" // RxPacketMessage
#include <PubSubClient.h>
#include <WiFiClient.h>

extern Status status;

class MQTT_Pluto : public PubSubClient
{
public:
  static MQTT_Pluto &getInstance()
  {
    static MQTT_Pluto instance;
    return instance;
  }

  void begin();
  void loop();

  // Called from MQTT_Client::processRxQueue, after the packet has been sent to
  // TinyGS. Best effort: does nothing if the secondary broker isn't connected.
  void publishRx(const RxPacketMessage &msg);

private:
  MQTT_Pluto();
  void reconnect();
  void backoff();
  void applyConfig();
  void publishTracking();
  void publishHealth();
  uint32_t trackingFingerprint();
  uint32_t configFingerprint();
  String topic(const char *leaf);

  // Plain TCP: the TinyGS client already keeps a WiFiClientSecure up, and a
  // second TLS handshake would ask for 40-50 KB of heap at the peak. Not needed
  // for a broker on the LAN or behind a VPN.
  WiFiClient net;

  IPAddress serverIp;
  bool serverIpValid = false;
  bool enabled = false;

  uint32_t lastConfigFingerprint = 0;
  unsigned long lastConfigCheck = 0;
  unsigned long lastAttempt = 0;
  unsigned long retryDelay = 30UL * 1000;
  uint8_t failures = 0;
  uint32_t lastFingerprint = 0;
  unsigned long lastTracking = 0;
  unsigned long lastHealth = 0;

  static const unsigned long firstRetryDelay = 30UL * 1000;
  static const unsigned long maxRetryDelay = 300UL * 1000;
  static const unsigned long trackingHeartbeat = 15UL * 60 * 1000;
  static const unsigned long healthInterval = 60UL * 1000;
  static const unsigned long configCheckInterval = 1000;
  static const int32_t socketConnectTimeout = 3000;

  const char *topicTracking PROGMEM = "tracking";
  const char *topicHealth PROGMEM = "health";
  const char *topicRx PROGMEM = "rx";
  const char *topicStatus PROGMEM = "status";
};

#endif
