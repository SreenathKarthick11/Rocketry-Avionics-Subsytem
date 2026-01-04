#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <LittleFS.h>

// ---------- pins/config ----------
#define BMP_SDA 13
#define BMP_SCL 14
#define MPU_SDA 21
#define MPU_SCL 22

#define XBEE_RX 16
#define XBEE_TX 17
#define GPS_RX 3
#define GPS_TX 1

// Servo
const int servoPin = 32;
const int pwmChannel = 0;
const int pwmFreq = 50;
const int pwmResolution = 16;
const int minPulse = 500;
const int maxPulse = 2400;

// ---------- sensors / serial ----------
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

HardwareSerial xbeeSerial(2);
HardwareSerial gpsSerial(0);

// ---------- shared data ----------
typedef struct {
  float temperature;
  float pressure;
  float altitude_bmp;
  float altitude_gps;
  float latitude;
  float longitude;
  int satellites;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  unsigned long lastUpdateMs;
} SensorData;

static SensorData gSensorData;
static float baselineAltitude = NAN;
SemaphoreHandle_t xDataMutex = NULL;

static unsigned long pktCount = 0;

// ---------- Helper Functions ----------
uint32_t pulseToDuty(int microseconds) {
  const float period_us = 1000000.0 / pwmFreq;
  const uint32_t maxDuty = (1UL << pwmResolution) - 1;
  float duty = (microseconds / period_us) * (float)maxDuty;
  if (duty < 0) duty = 0;
  if (duty > maxDuty) duty = maxDuty;
  return (uint32_t)(duty + 0.5);
}
void setPulseWidth(int microseconds) {
  ledcWrite(pwmChannel, pulseToDuty(microseconds));
}
void setAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  int pw = map(angle, 0, 180, minPulse, maxPulse);
  setPulseWidth(pw);
}

String timeStr(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000;
  unsigned long hours = (totalSeconds / 3600) % 24;
  unsigned long minutes = (totalSeconds / 60) % 60;
  unsigned long seconds = totalSeconds % 60;

  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buf);
}

// ---------- Task declarations ----------
void CalibrationTask(void *pvParameters);
void SensorTask(void *pvParameters);
void XBeeTask(void *pvParameters);
void ServoTask(void *pvParameters);
void LoggingTask(void *pvParameters);

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n🚀 System Booting...");

  // Mutex
  xDataMutex = xSemaphoreCreateMutex();
  if (!xDataMutex) {
    Serial.println("❌ Mutex creation failed!");
    while (1) delay(1000);
  }

  // I2C setup
  Wire.begin(BMP_SDA, BMP_SCL);
  Wire1.begin(MPU_SDA, MPU_SCL);

  // Sensors
  Serial.println("🔹 Initializing BMP280...");
  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("✅ BMP280 OK");
  } else {
    Serial.println("⚠️ BMP280 not found!");
  }

  Serial.println("🔹 Initializing MPU6050...");
  if (mpu.begin(0x68, &Wire1)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("✅ MPU6050 OK");
  } else {
    Serial.println("⚠️ MPU6050 not found!");
  }

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  xbeeSerial.begin(9600, SERIAL_8N1, XBEE_RX, XBEE_TX);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(servoPin, pwmChannel);
  setAngle(0);

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS Mount Failed!");
  } else {
    Serial.println("✅ LittleFS mounted successfully");
    if (!LittleFS.exists("/server_log.csv")) {
      File file = LittleFS.open("/server_log.csv", FILE_WRITE);
      if (file) {
        file.println("TEAM_ID,Time,Packet,RelAlt,Pressure,Temperature,Lat,Lon,AltGPS,Sats,Ax,Ay,Az,Gx,Gy,Gz");
        file.close();
      }
    }
  }

  // Calibration
  CalibrationTask(NULL);

  // Start tasks
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(XBeeTask, "XBeeTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(ServoTask, "ServoTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(LoggingTask, "LoggingTask", 4096, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
}

// ---------- CalibrationTask ----------
void CalibrationTask(void *pvParameters) {
  const int N = 20;
  const TickType_t waitBetween = pdMS_TO_TICKS(200);
  float sum = 0.0f; int count = 0;

  Serial.println("[Calib] Starting baseline calibration...");
  for (int i = 0; i < N; ++i) {
    if (bmp.begin(0x76) || bmp.begin(0x77)) {
      float alt = bmp.readAltitude(1013.25);
      sum += alt; count++;
      Serial.printf("[Calib] Sample %d: %.2fm\n", i + 1, alt);
    }
    vTaskDelay(waitBetween);
  }

  if (count > 0) {
    float avg = sum / count;
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200))) {
      baselineAltitude = avg;
      xSemaphoreGive(xDataMutex);
    }
    Serial.printf("[Calib] Baseline altitude = %.2fm\n", avg);
  } else {
    Serial.println("[Calib] Failed - no valid BMP samples");
  }
}

// ---------- SensorTask ----------
void SensorTask(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(1000);
  sensors_event_t a, g, temp_event;

  for (;;) {
    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

    float t = NAN, p = NAN, alt_bmp = NAN;
    if (bmp.begin(0x76) || bmp.begin(0x77)) {
      t = bmp.readTemperature();
      p = bmp.readPressure() / 100.0F;
      alt_bmp = bmp.readAltitude(1013.25);
    }

    float relAlt = (!isnan(alt_bmp) && !isnan(baselineAltitude)) ? (alt_bmp - baselineAltitude) : NAN;

    mpu.getEvent(&a, &g, &temp_event);

    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100))) {
      gSensorData.temperature = t;
      gSensorData.pressure = p;
      gSensorData.altitude_bmp = relAlt;
      gSensorData.altitude_gps = gps.altitude.isValid() ? gps.altitude.meters() : NAN;
      gSensorData.latitude = gps.location.isValid() ? gps.location.lat() : NAN;
      gSensorData.longitude = gps.location.isValid() ? gps.location.lng() : NAN;
      gSensorData.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
      gSensorData.accel_x = a.acceleration.x;
      gSensorData.accel_y = a.acceleration.y;
      gSensorData.accel_z = a.acceleration.z;
      gSensorData.gyro_x = g.gyro.x;
      gSensorData.gyro_y = g.gyro.y;
      gSensorData.gyro_z = g.gyro.z;
      gSensorData.lastUpdateMs = millis();
      xSemaphoreGive(xDataMutex);
    }

    Serial.printf("[Sensor] T=%.2f°C, P=%.2fhPa, ΔAlt=%.2fm, Lat=%.6f, Lon=%.6f, Sats=%d\n",
                  t, p, relAlt, gSensorData.latitude, gSensorData.longitude, gSensorData.satellites);

    vTaskDelay(period);
  }
}

// ---------- XBeeTask ----------
void XBeeTask(void *pvParameters) {
  const TickType_t sendPeriod = pdMS_TO_TICKS(2000);
  SensorData localCopy;

  for (;;) {
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200))) {
      localCopy = gSensorData;
      xSemaphoreGive(xDataMutex);
    } else memset(&localCopy, 0, sizeof(localCopy));

    pktCount++;
    String tStr = timeStr(localCopy.lastUpdateMs);

    char out[400];
    snprintf(out, sizeof(out),
      "TEAM_04,%s,%lu,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
      tStr.c_str(),
      pktCount,
      localCopy.altitude_bmp,
      localCopy.pressure,
      localCopy.temperature,
      localCopy.latitude,
      localCopy.longitude,
      localCopy.altitude_gps,
      localCopy.satellites,
      localCopy.accel_x,
      localCopy.accel_y,
      localCopy.accel_z,
      localCopy.gyro_x,
      localCopy.gyro_y,
      localCopy.gyro_z
    );

    xbeeSerial.println(out);
    Serial.println("[XBee] " + String(out));

    vTaskDelay(sendPeriod);
  }
}

// ---------- ServoTask ----------
void ServoTask(void *pvParameters) {
  const TickType_t checkPeriod = pdMS_TO_TICKS(500);
  const float triggerDrop = 2;
  bool servoBusy = false;

  for (;;) {
    float alt = NAN;
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200))) {
      alt = gSensorData.altitude_bmp;
      xSemaphoreGive(xDataMutex);
    }

    if (!isnan(alt) && alt <= -triggerDrop && !servoBusy) {
      Serial.printf("[Servo] ΔAlt %.2fm -> trigger\n", alt);

      servoBusy = true;
      setAngle(90);
      vTaskDelay(pdMS_TO_TICKS(5000));
      setAngle(0);
      vTaskDelay(pdMS_TO_TICKS(2000));
      servoBusy = false;
    }

    vTaskDelay(checkPeriod);
  }
}

// ---------- LoggingTask ----------
void LoggingTask(void *pvParameters) {
  const TickType_t logInterval = pdMS_TO_TICKS(1000);
  SensorData localCopy;
  unsigned long lastLoggedMs = 0;

  for (;;) {
    bool hasNewData = false;
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200))) {
      localCopy = gSensorData;
      xSemaphoreGive(xDataMutex);

      if (localCopy.lastUpdateMs != lastLoggedMs) {
        hasNewData = true;
        lastLoggedMs = localCopy.lastUpdateMs;
      }
    }

    if (hasNewData) {
      pktCount++;

      String tStr = timeStr(localCopy.lastUpdateMs);

      if (LittleFS.begin(false)) {
        File file = LittleFS.open("/server_log.csv", FILE_APPEND);
        if (file) {
          file.printf("TEAM_04,%s,%lu,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            tStr.c_str(),
            pktCount,
            localCopy.altitude_bmp,
            localCopy.pressure,
            localCopy.temperature,
            localCopy.latitude,
            localCopy.longitude,
            localCopy.altitude_gps,
            localCopy.satellites,
            localCopy.accel_x,
            localCopy.accel_y,
            localCopy.accel_z,
            localCopy.gyro_x,
            localCopy.gyro_y,
            localCopy.gyro_z
          );
          file.close();
          Serial.println("[LOG] Data written to server_log.csv");
        } else {
          Serial.println("⚠️ Failed to open log file for writing");
        }
      }
    }

    vTaskDelay(logInterval);
  }
}
