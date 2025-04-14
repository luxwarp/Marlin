/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfigPre.h"

#if ALL(WIFISUPPORT, OTASUPPORT)
#undef _BV
#undef DISABLED
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <driver/gptimer.h> // Change: For IDF 5.1.4
#include "../shared/Delay.h"
#include "timers.h" // Include this to access the timer_config array

void OTA_init() {
  ArduinoOTA
    .onStart([]() {
      // Change: For IDF 5.1.4
      // Pause the stepper and temperature timers
      // We need to stop the timers used by Marlin during OTA
      if (timer_config[MF_TIMER_STEP].timer != NULL) {
        ESP_ERROR_CHECK(gptimer_stop(timer_config[MF_TIMER_STEP].timer));
      }
      if (timer_config[MF_TIMER_TEMP].timer != NULL) {
        ESP_ERROR_CHECK(gptimer_stop(timer_config[MF_TIMER_TEMP].timer));
      }

      // U_FLASH or U_SPIFFS
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      const char *str = "unknown";
      switch (error) {
        case OTA_AUTH_ERROR:    str = "Auth Failed";    break;
        case OTA_BEGIN_ERROR:   str = "Begin Failed";   break;
        case OTA_CONNECT_ERROR: str = "Connect Failed"; break;
        case OTA_RECEIVE_ERROR: str = "Receive Failed"; break;
        case OTA_END_ERROR:     str = "End Failed";     break;
      }
      Serial.println(str);
    });

  ArduinoOTA.begin();
}

void OTA_handle() {
  ArduinoOTA.handle();
}

#endif // WIFISUPPORT && OTASUPPORT
#endif // ARDUINO_ARCH_ESP32
