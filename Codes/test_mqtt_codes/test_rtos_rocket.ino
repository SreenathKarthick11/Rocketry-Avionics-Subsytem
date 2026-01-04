#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <PubSubClient.h>

// ---------------- CONFIG ----------------
const char* ssid = "Murali";
const char* password = "Suj1thr@";
const char* mqtt_server = "test.mosquitto.org";

// ---------------- PIN CONFIG ----------------
#define BMP_SDA 13
#define BMP_SCL 14
#define MPU_SDA 21
#define MPU_SCL 22

#define XBEE_RX 16
#define XBEE_TX 17
#define GPS_RX 3
#define GPS_TX 1

#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18
#define SD_CS   5

#define RELAY_PIN 32

#define QUEUE_LENGTH 12
#define MEASURE_INTERVAL_MS 1000UL
#define LOG_INTERVAL_MS 5000UL
#define APOGEE_DROP_M 0.1

// ---------------- GLOBALS ----------------
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

HardwareSerial xbeeSerial(2);
HardwareSerial gpsSerial(0);

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t sdMutex;

// THREE separate queues - one for each consumer task
QueueHandle_t dataQueue;   // For XBee
QueueHandle_t logQueue;    // For SD Logger  
QueueHandle_t stateQueue;  // For Apogee/State Machine

float baselineAltitude = 0.0;
const char* CSV_FILE = "/data_log.csv";

const char* TEAM_ID = "2024-ASI-01";
uint32_t packetCount = 0;

char currentState[16] = "BOOT";

// ---------------- STRUCT ----------------
typedef struct {
  uint32_t t_ms;
  float alt_bmp;
  float rel_alt;
  float pressure;
  float temperature;
  double lat;
  double lon;
  float alt_gps;
  int sats;
  float ax, ay, az;
  float gx, gy, gz;
  char state[16];
  char time_str[16];
  uint32_t pktCount;
} SensorData_t;

// ---------------- UTILS ----------------
WiFiClient espClient;
PubSubClient client(espClient);

void safePublish(const char* topic, const char* msg) {
  if (client.connected()) client.publish(topic, msg);
  else if (WiFi.status() == WL_CONNECTED) {
    if (client.connect("esp32_client")) {
      client.setServer(mqtt_server, 1883);
      client.publish(topic, msg);
    }
  }
}

// ---------------- TASKS ----------------

// Measurement Task - BROADCASTS to all three queues
void measurementTask(void* pvParameters) {
  SensorData_t data;
  while (true) {
    uint32_t now = millis();
    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      float alt_bmp = bmp.readAltitude(1013.25);
      float pressure = bmp.readPressure() / 100.0F;
      float temp = bmp.readTemperature();
      sensors_event_t a, g, temp_event;
      mpu.getEvent(&a, &g, &temp_event);
      xSemaphoreGive(i2cMutex);

      data.t_ms = now;
      data.alt_bmp = alt_bmp;
      data.rel_alt = alt_bmp - baselineAltitude;
      data.pressure = pressure;
      data.temperature = temp;
      data.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
      data.lon = gps.location.isValid() ? gps.location.lng() : 0.0;
      data.alt_gps = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
      data.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
      data.ax = a.acceleration.x; data.ay = a.acceleration.y; data.az = a.acceleration.z;
      data.gx = g.gyro.x; data.gy = g.gyro.y; data.gz = g.gyro.z;
      strncpy(data.state, currentState, sizeof(data.state));
      snprintf(data.time_str, sizeof(data.time_str), "%02d:%02d:%02d",
               gps.time.isValid() ? gps.time.hour() : 0,
               gps.time.isValid() ? gps.time.minute() : 0,
               gps.time.isValid() ? gps.time.second() : 0);
      data.pktCount = ++packetCount;

      // Send to ALL THREE queues (broadcast pattern)
      xQueueSend(dataQueue, &data, 0);   // XBee
      xQueueSend(logQueue, &data, 0);    // SD Logger
      xQueueSend(stateQueue, &data, 0);  // State Machine
    }
    vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
  }
}

// XBee Task - receives from dataQueue
void xbeeTask(void* pvParameters) {
  SensorData_t item;
  while (true) {
    if (xQueueReceive(dataQueue, &item, portMAX_DELAY) == pdTRUE) {
      char msg[512];
      snprintf(msg, sizeof(msg),
               "%s,%s,%u,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.2f,%.2f,%.2f,%s,%.2f,%.2f,%.2f",
               TEAM_ID, item.time_str, item.pktCount,
               item.rel_alt, item.pressure, item.temperature,
               item.lat, item.lon, item.alt_gps, item.sats,
               item.ax, item.ay, item.az, item.state,
               item.gx, item.gy, item.gz);
      xbeeSerial.println(msg);
      Serial.println(msg);
      safePublish("esp32/status", msg);
    }
  }
}

// SD Logger Task - receives from logQueue
void sdLoggerTask(void* pvParameters) {
  SensorData_t row;
  static uint32_t lastLogTime = 0;
  while (true) {
    if (xQueueReceive(logQueue, &row, portMAX_DELAY) == pdTRUE) {
      if (millis() - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = millis();
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500))) {
          File f = SD.open(CSV_FILE, FILE_APPEND);
          if (f) {
            f.printf("%s,%u,%s,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",
                     TEAM_ID, row.pktCount, row.time_str,
                     row.rel_alt, row.pressure, row.temperature,
                     row.lat, row.lon, row.alt_gps, row.sats,
                     row.ax, row.ay, row.az,
                     row.gx, row.gy, row.gz,
                     row.state);
            f.close();
          }
          xSemaphoreGive(sdMutex);
        }
      }
    }
  }
}

// Apogee & State Machine Task - receives from stateQueue
void apogeeTask(void* pvParameters) {
  float maxAltitude = -1e6f;
  SensorData_t data;
  char lastState[16];
  strncpy(lastState, currentState, sizeof(lastState));
  uint32_t debugCounter = 0;

  while (true) {
    // Now we RECEIVE (not peek) because we have our own dedicated queue
    if (xQueueReceive(stateQueue, &data, pdMS_TO_TICKS(500)) == pdTRUE) {
      float rel = data.rel_alt;
      
      // Reduced debug output - only every 10th reading to prevent stack overflow
      if (debugCounter++ % 10 == 0) {
        Serial.print("State=");
        Serial.print(currentState);
        Serial.print(" Alt=");
        Serial.println(rel);
      }

      if (strncmp(currentState, "LAUNCHPAD", 9) == 0 && rel > 0.1) {
        strncpy(currentState, "ASCENT", sizeof(currentState));
        Serial.println(">>> ASCENT");
      }
      else if (strncmp(currentState, "ASCENT", 6) == 0) {
        if (rel > maxAltitude) maxAltitude = rel;
        if ((maxAltitude - rel) >= APOGEE_DROP_M) {
          strncpy(currentState, "PAYLOAD_SEP", sizeof(currentState));
          digitalWrite(RELAY_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(2000));
          digitalWrite(RELAY_PIN, LOW);
        }
      } 
      else if (strncmp(currentState, "PAYLOAD_SEP", 11) == 0 && rel < maxAltitude - 0.1f)
        strncpy(currentState, "DESCENT", sizeof(currentState)); 
      else if (strncmp(currentState, "DESCENT", 7) == 0) {
        if (rel < 500.0f) strncpy(currentState, "AEROBRAKE", sizeof(currentState));
        if (rel < 2.0f) strncpy(currentState, "IMPACT", sizeof(currentState));
      }

      if (strncmp(lastState, currentState, sizeof(currentState)) != 0) {
        strncpy(lastState, currentState, sizeof(lastState));
        Serial.print(">>> State: ");
        Serial.println(currentState);
        
        char msg[48];
        snprintf(msg, sizeof(msg), "State:%s", currentState);
        safePublish("esp32/status", msg);
      }
    }
    // Feed watchdog
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- SETUP ----------------
void setupSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD fail");
  } else if (!SD.exists(CSV_FILE)) {
    File f = SD.open(CSV_FILE, FILE_WRITE);
    if (f) {
      f.println("TeamID,Packet,Time,Alt,Pressure,Temp,Lat,Lng,Alt_GPS,Sats,Ax,Ay,Az,Gx,Gy,Gz,State");
      f.close();
    }
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    client.setServer(mqtt_server, 1883);
    client.connect("esp32_client");
    client.publish("esp32/status", "WiFi+MQTT OK");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("\n\n=== ROCKET BOOT ===");
  
  Wire.begin(BMP_SDA, BMP_SCL);
  Wire1.begin(MPU_SDA, MPU_SCL);

  i2cMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  
  // Create THREE separate queues
  dataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorData_t));
  logQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorData_t));
  stateQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorData_t));

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  xbeeSerial.begin(9600, SERIAL_8N1, XBEE_RX, XBEE_TX);

  if (!bmp.begin(0x76) && !bmp.begin(0x77)) Serial.println("BMP280 fail");
  if (!mpu.begin()) Serial.println("MPU6050 fail");

  setup_wifi();
  setupSD();
  
  safePublish("esp32/status","Calibrating");
  
  // Calibration
  strncpy(currentState, "CALIBRATION", sizeof(currentState));
  Serial.println("Calibrating...");
  
  float sum = 0;
  for (int i = 0; i < 20; i++) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      sum += bmp.readAltitude(1013.25);
      xSemaphoreGive(i2cMutex);
    }
    delay(100);
  }
  
  baselineAltitude = sum / 20.0f;
  strncpy(currentState, "LAUNCHPAD", sizeof(currentState));
  
  Serial.print("Baseline: ");
  Serial.print(baselineAltitude);
  Serial.println(" m");
  
  safePublish("esp32/status","LAUNCHPAD");
  delay(500);
  
  // Create tasks with LARGER stack sizes
  xTaskCreatePinnedToCore(measurementTask, "measure", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(xbeeTask, "xbee", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(sdLoggerTask, "sdlog", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(apogeeTask, "apogee", 4096, NULL, 2, NULL, 1);  // Changed to Core 1 with 4KB stack
  
  Serial.println("=== TASKS STARTED ===\n");
}

void loop() {
  if (client.connected()) client.loop();
  vTaskDelay(pdMS_TO_TICKS(1000));
}