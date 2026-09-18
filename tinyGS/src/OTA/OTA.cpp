/*
  OTA.cpp - On The Air Update Class
  
  Copyright (C) 2020 -2021 @G4lile0, @gmag12 and @dev_4m1g0

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






#include "./OTA.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)
  #include <Arduino.h>
  #include <WiFiClientSecure.h>
#endif


#include "../ConfigManager/ConfigManager.h"
#include "../Status.h"
#include "../Logger/Logger.h"

extern Status status;
bool usingNewCert = true;
const long MIN_TIME_BEFORE_UPDATE = random(60000, 20*60*1000);

void OTA::update()
{
#ifdef SECURE_OTA
  WiFiClientSecure client;
//  if (usingNewCert)
    client.setCACert(newRoot_CA);
//  else
//    client.setCACert(DSTroot_CA);
#else
  WiFiClient client;
#endif

  uint64_t chipId = ESP.getEfuseMac();
  char clientId[13];
  sprintf(clientId, "%04X%08X",(uint16_t)(chipId>>32), (uint32_t)chipId);

  ConfigManager& c = ConfigManager::getInstance();
  char url[255];
  sprintf_P(url, PSTR("%s?user=%s&name=%s&mac=%s&version=%d&rescue=%s"), OTA_URL, c.getMqttUser(), c.getThingName(), clientId, status.version, (c.isFailSafeActive()?"true":"false"));

  Log::debug(PSTR("Checking for firmware Updates...  "));
  t_httpUpdate_return ret = httpUpdate.update(client, url, status.git_version);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      usingNewCert = !usingNewCert;
      Log::info(PSTR("Update failed Error (%d): %s\n"), httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES: // server 304
      Log::info(PSTR("No updates required"));
      break;

    case HTTP_UPDATE_OK:
      Log::info(PSTR("Update ok but ESP has not restarted!!! (This should never be printed)"));
      break;
  }
}

unsigned static long lastUpdateTime = 0;
void OTA::loop()
{
  if (millis() < MIN_TIME_BEFORE_UPDATE)
    return;

  if (millis() - lastUpdateTime > TIME_BETTWEN_UPDATE_CHECK)
  {
    lastUpdateTime = millis();

    // "Automatic Firmware Update" only ever gated this hourly check in the UI's
    // intent, never in code: the checkbox had no getter and this loop called
    // update() unconditionally. A custom-firmware station (a fork, a local
    // build with unmerged changes) can't safely leave this on, since the OTA
    // server compares against status.git_version and would happily overwrite
    // it with upstream's official build. The on-demand cmnd/update path (also
    // reachable from tinygs/global, i.e. server-pushed to every station) is
    // deliberately left ungated below, for anyone who wants to force an update
    // by hand regardless of this setting.
    if (!ConfigManager::getInstance().getAutoUpdate())
    {
      Log::debug(PSTR("Automatic firmware update disabled, skipping hourly check"));
      return;
    }

    update();
  }
}
