#define me "SN06"
#define LOG_LEVEL LOG_INFO

#include <jvlib.h>
#include <ArduinoJson.h>
#include "PMS.h"

// Hardwired solder joints matching your S2 Mini clone photo
#define TXpin 5   // Black Wire -> Sensor RX
#define RXpin 18  // Red Wire   -> Sensor TX

PMS pms(Serial1);
PMS::DATA data;

// Measurement tracking counters
unsigned long t1old = 0, t1interval = 5 * 60 * 1000; // Sample every 5 minutes
float pm1_sum = 0, pm25_sum = 0, pm10_sum = 0;
int i = 0;

// Non-blocking state variables
int firstboot = 1;
int t1 = 1, t2 = 1, t3 = 1;

// Linear Interpolation Math Helper
double lerp(double ylo, double yhi, double xlo, double xhi, double x) {
  return ((x - xlo) / (xhi - xlo)) * (yhi - ylo) + ylo;
}

// EPA US-AQI calculation matrix matching your original bounds
int pm25_aqi(float pm25) {
  float c = floor(10.0 * pm25) / 10.0;
  int a = c < 0.0     ? 0  
          : c < 9.1  ? lerp(0, 50, 0.0, 9.0, c)
          : c < 35.5  ? lerp(51, 100, 9.1, 35.4, c)
          : c < 55.5  ? lerp(101, 150, 35.5, 55.4, c)
          : c < 150.5 ? lerp(151, 200, 55.5, 125.4, c)
          : c < 250.5 ? lerp(201, 300, 125.5, 225.4, c)
          : c < 500.5 ? lerp(301, 500, 225.5, 500.4, c)
                      : 500;  
  return round(a);
}

void publishAQI(float final_pm1, float final_pm25, float final_pm10, int final_aqi) {
  // Use Version 6 JSON parsing matching your library standard
  StaticJsonDocument<512> doc;
  doc["name"] = me;
  doc["id"] = jv::deviceId();
  doc["06pm_1.0"] = final_pm1;
  doc["06pm_25"] = final_pm25;
  doc["06pm_10"] = final_pm10;
  doc["06pmAQI"] = final_aqi;
  doc["myip"] = jv::ip();
  doc["rssi"] = jv::rssi();

  char msgBuffer[512];
  serializeJson(doc, msgBuffer, sizeof(msgBuffer));
  
  // Directly pipes payload out using your native jvlib publisher
  bool ok = jv::publishRaw("garage/aqi", msgBuffer, false);
  LOG_INFO("AQI publish results -> %s", ok ? "OK" : "FAIL");
}

void setup() {
  // Fall back entirely on your new library wrapper execution profiles
  jv::begin();
  jvWatchdogSetup();
  jvDailyRebootSetup();

  // Route Serial1 explicitly to your physical pins
  Serial1.begin(9600, SERIAL_8N1, RXpin, TXpin);
  
  // Set initial low-power target states
  pms.sleep();
  LOG_INFO("Plantower AQI Sensor Node SN06 Active under jvlib");
}

void loop() {
  // Feed your central library's background WiFi & update handlers continuously
  jv::loop();
  jvWatchdogFeed();
  jvCheckDailyReboot();

  unsigned long now = millis();

  // State 1: Every 5 minutes, wake up the sensor fan and laser lines
  if (firstboot == 1 || (unsigned long)(now - t1old) >= t1interval) {
    firstboot = 0;
    t1old = now;
    LOG_INFO("Waking up Plantower sensor fan...");
    pms.wakeUp();
    t1 = 0; t2 = 0; t3 = 0;
  }

  // State 2: At 35 seconds of uptime, clear measurement accumulation counters
  if ((unsigned long)(now - t1old) >= 35000 && t2 == 0) {
    t2 = 1;
    LOG_INFO("Clearing sample tracking counters, beginning data catch...");
    pm1_sum = 0.0;
    pm25_sum = 0.0;
    pm10_sum = 0.0;
    i = 0;
  }

  // State 3: Keep grabbing passive stream metrics as long as data hits the UART buffer
  if (t2 == 1 && t3 == 0) {
    if (pms.read(data)) {
      i++;
      pm1_sum += data.PM_AE_UG_1_0;
      pm25_sum += data.PM_AE_UG_2_5;
      pm10_sum += data.PM_AE_UG_10_0;
    }
  }

  // State 4: At 50 seconds of uptime, power down the hardware to preserve laser lifespan
  if ((unsigned long)(now - t1old) >= 50000 && t1 == 0) {
    t1 = 1;
    LOG_INFO("Putting Plantower sensor back to sleep mode...");
    pms.sleep();
  }

  // State 5: At 55 seconds, compute arithmetic averages, compile JSON, and fire to MQTT
  if ((unsigned long)(now - t1old) >= 55000 && t3 == 0) {
    t3 = 1;
    if (i > 0) {
      float final_pm1 = pm1_sum / i;
      float final_pm25 = pm25_sum / i;
      float final_pm10 = pm10_sum / i;
      int final_aqi = pm25_aqi(final_pm25);

      publishAQI(final_pm1, final_pm25, final_pm10, final_aqi);
    } else {
      LOG_WARN("Skipping write sequence - Zero valid reading blocks caught from UART");
    }
  }

  delay(10); // Maintain pacing matching your garage template
}
