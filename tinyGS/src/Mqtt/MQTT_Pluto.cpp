/*
  MQTT_Pluto.cpp - Forward station telemetry to a secondary MQTT broker

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

#include "MQTT_Pluto.h"
#include "ArduinoJson.h"
#include "../Logger/Logger.h"
#include "../Radio/Radio.h"
#include <time.h>

MQTT_Pluto::MQTT_Pluto()
    : PubSubClient(net)
{
}

void MQTT_Pluto::begin()
{
  lastConfigFingerprint = configFingerprint();
  applyConfig();
}

// Reads the current portal configuration. Called at startup and every time a
// forwarding field changes, so saving the form takes effect without rebooting
// the board.
void MQTT_Pluto::applyConfig()
{
  ConfigManager &configManager = ConfigManager::getInstance();

  if (connected())
    disconnect();
  net.stop();

  serverIpValid = false;
  failures = 0;
  retryDelay = firstRetryDelay;
  lastAttempt = 0; // so it retries right away, without waiting the old backoff
  status.pluto_connected = false;

  enabled = configManager.getPlutoEnabled();

  if (!enabled)
  {
    Log::console(PSTR("[MQTT2] Forwarding disabled (missing server or checkbox)"));
    return;
  }

  // The buffer stays at MQTT_MAX_PACKET_SIZE (1000). Plenty for LoRa: a
  // packet's JSON doesn't reach ~700 bytes with the topic included, because it
  // doesn't carry data_raw (that field is only added on the FSK branch).
  setSocketTimeout(5);
  setKeepAlive(60);

  Log::console(PSTR("[MQTT2] Forwarding enabled -> %s:%u (prefix '%s')"),
               configManager.getPlutoServer(),
               configManager.getPlutoPort(),
               configManager.getPlutoTopic());
}

// Hash of the portal's forwarding fields, to detect that the user saved
// changes and react without rebooting.
uint32_t MQTT_Pluto::configFingerprint()
{
  ConfigManager &configManager = ConfigManager::getInstance();
  uint32_t hash = 2166136261UL;

  auto mixStr = [&hash](const char *text) {
    for (const char *c = text; *c; c++)
    {
      hash ^= (uint8_t)*c;
      hash *= 16777619UL;
    }
    hash ^= 0xFF; // separator, so "ab"+"c" and "a"+"bc" don't collide
    hash *= 16777619UL;
  };

  mixStr(configManager.getPlutoEnabled() ? "1" : "0");
  mixStr(configManager.getPlutoServer());
  mixStr(configManager.getPlutoUser());
  mixStr(configManager.getPlutoPass());
  mixStr(configManager.getPlutoTopic());
  uint16_t port = configManager.getPlutoPort();
  hash ^= port;
  hash *= 16777619UL;

  return hash;
}

// Builds "<prefix>/<station>/<leaf>". The station name is slugified because the
// factory default is "My TinyGS", with a space: MQTT tolerates it but it's
// awkward to subscribe to from the console. The original name travels
// untouched in the "station" field of every JSON.
String MQTT_Pluto::topic(const char *leaf)
{
  ConfigManager &configManager = ConfigManager::getInstance();

  static char topicBuffer[96];
  char *p = topicBuffer;
  const char *limit = topicBuffer + sizeof(topicBuffer) - 1;

  for (const char *s = configManager.getPlutoTopic(); *s && p < limit; s++)
    *p++ = *s;

  if (p < limit)
    *p++ = '/';

  for (const char *s = configManager.getThingName(); *s && p < limit; s++)
  {
    char c = *s;
    bool safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                (c >= '0' && c <= '9') || c == '-' || c == '_';
    *p++ = safe ? c : '_';
  }

  if (p < limit)
    *p++ = '/';

  for (const char *s = leaf; *s && p < limit; s++)
    *p++ = *s;

  *p = '\0';
  return String(topicBuffer);
}

void MQTT_Pluto::reconnect()
{
  ConfigManager &configManager = ConfigManager::getInstance();

  // PubSubClient::connect() calls _client->connect(domain, port) WITHOUT a
  // timeout, and on the ESP32 that blocks the main loop until the default TCP
  // timeout (on the order of 30 s) if the host doesn't respond. During that
  // time radio.listen() doesn't run and packets from a real pass are lost.
  // That's why we resolve DNS ourselves and open the socket with a short
  // timeout: PubSubClient reuses it, since it checks _client->connected()
  // before trying to connect on its own.
  if (!serverIpValid)
  {
    if (WiFi.hostByName(configManager.getPlutoServer(), serverIp) != 1)
    {
      Log::debug(PSTR("[MQTT2] Could not resolve %s"), configManager.getPlutoServer());
      backoff();
      return;
    }
    serverIpValid = true;
    setServer(serverIp, configManager.getPlutoPort());
  }

  if (!net.connected() &&
      net.connect(serverIp, configManager.getPlutoPort(), socketConnectTimeout) != 1)
  {
    Log::debug(PSTR("[MQTT2] No TCP response from %s:%u"),
               configManager.getPlutoServer(), configManager.getPlutoPort());
    backoff();
    return;
  }

  uint64_t chipId = ESP.getEfuseMac();
  char clientId[20];
  sprintf(clientId, "%04X%08X-p", (uint16_t)(chipId >> 32), (uint32_t)chipId);

  const char *user = configManager.getPlutoUser();
  const char *pass = configManager.getPlutoPass();
  String statusTopic = topic(topicStatus);

  // Retained LWT: if the station crashes or loses the network, the broker
  // publishes "offline" on its own, with no need for the board to say anything.
  if (connect(clientId,
              user[0] ? user : NULL,
              pass[0] ? pass : NULL,
              statusTopic.c_str(), 0, true, "offline"))
  {
    Log::console(PSTR("[MQTT2] Connected to %s:%u"),
                 configManager.getPlutoServer(), configManager.getPlutoPort());
    status.pluto_connected = true;
    retryDelay = firstRetryDelay;
    failures = 0;

    publish(statusTopic.c_str(), "online", true);

    // Current state right after connecting, so a consumer that starts up
    // mid-pass knows what's being listened to without waiting for the next
    // change.
    publishTracking();
    lastFingerprint = trackingFingerprint();
    lastTracking = millis();
  }
  else
  {
    Log::debug(PSTR("[MQTT2] MQTT connection failed, rc=%i"), state());
    net.stop();
    backoff();
  }
}

void MQTT_Pluto::backoff()
{
  status.pluto_connected = false;

  // Every 4 failed attempts we resolve DNS again: the broker may have changed
  // IP (DHCP, a recreated container).
  if (++failures >= 4)
  {
    failures = 0;
    serverIpValid = false;
  }

  retryDelay *= 2;
  if (retryDelay > maxRetryDelay)
    retryDelay = maxRetryDelay;

  // Deliberately NOT replicating the ESP.restart() that MQTT_Client does after
  // several failed attempts: an outage on the secondary broker can't take down
  // the station or interrupt what's sent to TinyGS.
}

void MQTT_Pluto::loop()
{
  if (!WiFi.isConnected())
    return;

  // The configuration is re-read periodically instead of being snapshotted once
  // in begin(): this way, saving the portal form takes effect right away.
  if (millis() - lastConfigCheck > configCheckInterval)
  {
    lastConfigCheck = millis();
    uint32_t configHash = configFingerprint();
    if (configHash != lastConfigFingerprint)
    {
      lastConfigFingerprint = configHash;
      applyConfig();
    }
  }

  if (!enabled)
    return;

  if (!connected())
  {
    status.pluto_connected = false;
    if (millis() - lastAttempt > retryDelay)
    {
      lastAttempt = millis();
      reconnect();
    }
    return;
  }

  PubSubClient::loop();

  // We observe the state instead of hooking every command: the five routes that
  // decide what to listen to (begine, beginp, sat, begin_lora, freq) and
  // startup from EEPROM all end up writing status.modeminfo, so a single
  // comparator covers all of them and keeps working if a sixth one shows up.
  uint32_t fingerprint = trackingFingerprint();
  if (fingerprint != lastFingerprint || millis() - lastTracking > trackingHeartbeat)
  {
    publishTracking();
    lastFingerprint = fingerprint;
    lastTracking = millis();
  }

  if (millis() - lastHealth > healthInterval)
  {
    lastHealth = millis();
    publishHealth();
  }
}

uint32_t MQTT_Pluto::trackingFingerprint()
{
  // FNV-1a over the fields that define WHAT we're listening to.
  // Deliberately excludes currentRssi (the noise floor) and status.tle's
  // doppler: they change on every loop pass and aren't a change of target.
  // Including them would publish "tracking" several times a second.
  uint32_t hash = 2166136261UL;

  auto mix = [&hash](const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++)
    {
      hash ^= bytes[i];
      hash *= 16777619UL;
    }
  };

  const ModemInfo &m = status.modeminfo;
  mix(m.satellite, sizeof(m.satellite));
  mix(m.modem_mode, sizeof(m.modem_mode));
  mix(&m.NORAD, sizeof(m.NORAD));
  mix(&m.frequency, sizeof(m.frequency));
  mix(&m.freqOffset, sizeof(m.freqOffset));
  mix(&m.bw, sizeof(m.bw));
  mix(&m.sf, sizeof(m.sf));
  mix(&m.cr, sizeof(m.cr));
  mix(&m.bitrate, sizeof(m.bitrate));
  mix(&m.freqDev, sizeof(m.freqDev));
  mix(m.tle, sizeof(m.tle));
  mix(m.filter, sizeof(m.filter));

  return hash;
}

void MQTT_Pluto::publishTracking()
{
  if (!connected())
    return;

  ConfigManager &configManager = ConfigManager::getInstance();
  const ModemInfo &m = status.modeminfo;
  time_t now;
  time(&now);

  StaticJsonDocument<512> doc;
  doc["station"] = configManager.getThingName();
  doc["ts"] = now;
  doc["satellite"] = m.satellite;
  doc["NORAD"] = m.NORAD;
  doc["mode"] = m.modem_mode;
  doc["frequency"] = m.frequency;
  doc["frequency_offset"] = m.freqOffset;

  if (strcmp(m.modem_mode, "LoRa") == 0)
  {
    doc["sf"] = m.sf;
    doc["cr"] = m.cr;
    doc["bw"] = m.bw;
    doc["iIQ"] = m.iIQ;
  }
  else
  {
    doc["bitrate"] = m.bitrate;
    doc["freqdev"] = m.freqDev;
    doc["rxBw"] = m.bw;
  }

  doc["doppler_comp"] = status.tle.freqComp;
  doc["has_tle"] = m.tle[0] != 0;

  // Noise floor measured by Radio::begin() right after startReceive(), so it
  // belongs to the frequency reported above and is as fresh as this message.
  // It is the one number that tells a deaf front end (flat, very low floor)
  // apart from a swamped band (high floor), which cannot be told apart from
  // "no satellite was transmitting" without it.
  doc["noise_floor"] = m.currentRssi;

  JsonArray station_location = doc.createNestedArray("station_location");
  station_location.add(configManager.getLatitude());
  station_location.add(configManager.getLongitude());

  char buffer[512];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));

  // Retained: the broker always keeps the last one, so anyone who subscribes
  // instantly knows which satellite the station is following.
  if (!publish(topic(topicTracking).c_str(), (const uint8_t *)buffer, len, true))
    Log::debug(PSTR("[MQTT2] tracking dropped (%u bytes)"), len);
}

// Periodic receiver health. Unlike tracking, this goes out on a timer, so the
// series keeps sampling even while TinyGS leaves the station on one satellite.
void MQTT_Pluto::publishHealth()
{
  if (!connected())
    return;

  ConfigManager &configManager = ConfigManager::getInstance();
  time_t now;
  time(&now);

  // Take the reading here instead of trusting status.modeminfo.currentRssi as
  // it stands. Radio::begin() samples it microseconds after startReceive(),
  // before the AGC has settled, so it comes back pegged at the SX1262 floor
  // (-127.5 dBm, the register saturated at 255) and never changes: the only
  // other writer, Radio::currentRssi(), is never called anywhere in the tree.
  // Sampling now, with the radio settled in RX, is what makes the number mean
  // something.
  if (status.radio_ready)
    Radio::getInstance().currentRssi();

  StaticJsonDocument<384> doc;
  doc["station"] = configManager.getThingName();
  doc["ts"] = now;
  doc["noise_floor"] = status.modeminfo.currentRssi;
  doc["mode"] = status.modeminfo.modem_mode;
  doc["frequency"] = status.modeminfo.frequency;
  doc["radio_ready"] = status.radio_ready;
  doc["radio_error"] = status.radio_error;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();
  doc["heap_min"] = ESP.getMinFreeHeap();
  doc["heap_max_block"] = ESP.getMaxAllocHeap();
  doc["uptime"] = millis() / 1000;

  char buffer[384];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));

  // Not retained: this is a time series, and tracking already carries the
  // retained "what is the station doing right now" snapshot.
  if (!publish(topic(topicHealth).c_str(), (const uint8_t *)buffer, len, false))
    Log::debug(PSTR("[MQTT2] health dropped (%u bytes)"), len);
}

void MQTT_Pluto::publishRx(const RxPacketMessage &msg)
{
  if (!connected())
    return;

  ConfigManager &configManager = ConfigManager::getInstance();

  size_t packetLen = strlen(msg.packet);
  size_t rawLen = strlen(msg.raw_packet);

  // Same field names as MQTT_Client::sendRxFromQueue, so the two sources can be
  // compared field by field with no translation. Only "station" is added.
  const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(25) + 256 + packetLen + rawLen;
  DynamicJsonDocument doc(capacity);

  doc["station"] = configManager.getThingName();
  JsonArray station_location = doc.createNestedArray("station_location");
  station_location.add(configManager.getLatitude());
  station_location.add(configManager.getLongitude());

  doc["mode"] = msg.modem_mode;
  doc["frequency"] = msg.frequency;
  doc["frequency_offset"] = msg.freqOffset;
  if (msg.freqDoppler != 0)
    doc["f_doppler"] = msg.freqDoppler;

  doc["satellite"] = msg.satellite;

  if (strcmp(msg.modem_mode, "LoRa") == 0)
  {
    doc["sf"] = msg.sf;
    doc["cr"] = msg.cr;
    doc["bw"] = msg.bw;
    doc["iIQ"] = msg.iIQ;
  }
  else
  {
    doc["bitrate"] = msg.bitrate;
    doc["freqdev"] = msg.freqDev;
    doc["rxBw"] = msg.bw;
    doc["data_raw"] = msg.raw_packet;
  }

  doc["rssi"] = msg.rssi;
  doc["snr"] = msg.snr;
  doc["frequency_error"] = msg.frequencyerror;
  doc["unix_GS_time"] = msg.unix_time;
  doc["usec_time"] = msg.usec_time;
  doc["crc_error"] = msg.crc_error;
  doc["data"] = msg.packet;
  doc["NORAD"] = msg.NORAD;
  doc["noisy"] = msg.noisy;

  size_t bufferSize = measureJson(doc) + 1;
  char *buffer = (char *)malloc(bufferSize);
  if (buffer == nullptr)
  {
    Log::errorAsync(PSTR("[MQTT2] Out of memory for the rx JSON (%u bytes)"), bufferSize);
    return;
  }

  size_t len = serializeJson(doc, buffer, bufferSize);

  // publish() silently drops the message if it doesn't fit in bufferSize. It
  // shouldn't happen with LoRa, but if the station is ever pointed at an FSK
  // satellite the JSON carries data and data_raw and can go over: we want to
  // find out from the log instead of silently losing packets.
  if (!publish(topic(topicRx).c_str(), (const uint8_t *)buffer, len, false))
    Log::debugAsync(PSTR("[MQTT2] rx dropped (%u bytes)"), len);

  free(buffer);
}
