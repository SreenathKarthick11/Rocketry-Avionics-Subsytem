# Flight Computer - Transmitter (Rocket-Side)

## Overview

This is an ESP32-based flight computer designed for rocket telemetry and data logging. It collects sensor data from an MPU6050 IMU, logs it to an SD card, detects high-g events, and transmits telemetry wirelessly via XBee.

---

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **ESP32** | Main microcontroller |
| **MPU6050** | 6-axis IMU (accelerometer + gyroscope) |
| **SD Card Module** | Data logging storage |
| **XBee Module** | Wireless telemetry transmission |

---

## Pin Configuration

### XBee Serial Communication

| Pin | GPIO | Function |
|-----|------|----------|
| RX | GPIO 16 | Receive data from XBee |
| TX | GPIO 17 | Transmit data to XBee |

**Baud Rate:** 9600

---

## System Architecture

The system uses **FreeRTOS** multitasking with four concurrent tasks:

```
┌─────────────────┐
│  Sensor Task    │ ──┐
│  (Priority: 2)  │   │
└─────────────────┘   │
                      ├──> ┌──────────────┐
┌─────────────────┐   │    │ Data Queue   │
│   Log Task      │ <─┼────│  (10 items)  │
│  (Priority: 1)  │   │    └──────────────┘
└─────────────────┘   │
                      │
┌─────────────────┐   │
│  Alert Task     │ <─┤
│  (Priority: 3)  │   │
└─────────────────┘   │
                      │
┌─────────────────┐   │
│  XBee Task      │ <─┘
│  (Priority: 1)  │
└─────────────────┘
```

---

## Data Structure

The sensor data is encapsulated in a structured format for efficient queue handling:

```cpp
typedef struct {
  float ax, ay, az;     // Acceleration (m/s²)
  float gx, gy, gz;     // Gyroscope (rad/s)
  float temp;           // Temperature (°C)
} SensorData;
```

---

## Complete Code

```cpp
/*
 * ============================================================================
 * FLIGHT COMPUTER - TRANSMITTER (ROCKET-SIDE)
 * ============================================================================
 * Purpose: Onboard flight computer with data logging and telemetry
 * Hardware: ESP32 + MPU6050 + SD Card + XBee
 * 
 * Features:
 * - 6-axis IMU data collection
 * - SD card data logging
 * - Real-time wireless telemetry transmission
 * - High-g event detection for launch/impact
 * - Optimized task priorities for mission-critical operations
 * 
 * XBee Configuration:
 * RX: GPIO 16
 * TX: GPIO 17
 * Baud: 9600
 * ============================================================================
 */

#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_MPU6050 mpu;
QueueHandle_t dataQueue;
HardwareSerial XBeeSerial(2);

// ============================================================================
// DATA STRUCTURE
// ============================================================================

typedef struct {
  float ax, ay, az;     // Acceleration (m/s²)
  float gx, gy, gz;     // Gyroscope (rad/s)
  float temp;           // Temperature (°C)
} SensorData;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  XBeeSerial.begin(9600, SERIAL_8N1, 16, 17);

  // ---- Initialize MPU6050 ----
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ---- Initialize SD Card ----
  if (!SD.begin()) {
    Serial.println("❌ Card Mount Failed");
    return;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("❌ No SD card attached");
    return;
  }

  // Create CSV file with header
  File file = SD.open("/data.txt", FILE_WRITE);
  if (file) {
    file.println("ax,ay,az,gx,gy,gz,temp");
    file.close();
    Serial.println("✅ SD file initialized");
  } else {
    Serial.println("❌ Failed to create file");
  }

  // ---- Create Queue ----
  dataQueue = xQueueCreate(10, sizeof(SensorData));

  // ---- Create Tasks ----
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(logTask, "Log Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(alertTask, "Alert Task", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(xbeeTask, "XBee Task", 2048, NULL, 1, NULL, 1);
}

// ============================================================================
// TASK 1: SENSOR READING
// ============================================================================

void sensorTask(void *pvParameters) {
  SensorData data;

  while (1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    data.temp = temp.temperature;

    xQueueSend(dataQueue, &data, portMAX_DELAY);
    vTaskDelay(200 / portTICK_PERIOD_MS);  // 5 Hz
  }
}

// ============================================================================
// TASK 2: SD CARD LOGGING
// ============================================================================

void logTask(void *pvParameters) {
  SensorData received;

  while (1) {
    if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
      File file = SD.open("/data.txt", FILE_APPEND);
      if (file) {
        file.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    received.ax, received.ay, received.az,
                    received.gx, received.gy, received.gz,
                    received.temp);
        file.close();
        Serial.println("💾 Data logged to SD");
      } else {
        Serial.println("❌ Failed to open file for appending");
      }
    }
  }
}

// ============================================================================
// TASK 3: HIGH-G DETECTION
// ============================================================================

void alertTask(void *pvParameters) {
  SensorData received;

  while (1) {
    if (xQueuePeek(dataQueue, &received, portMAX_DELAY)) {
      // Detect launch or impact (>7g on X-axis)
      if (received.ax > 7.0) {
        Serial.println("⚠️  High Acceleration Detected on X-axis!");
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}

// ============================================================================
// TASK 4: TELEMETRY TRANSMISSION
// ============================================================================

void xbeeTask(void *pvParameters) {
  SensorData data;

  while (1) {
    if (xQueuePeek(dataQueue, &data, portMAX_DELAY)) {
      // Transmit formatted telemetry packet
      XBeeSerial.printf("AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f T:%.2f\n",
                        data.ax, data.ay, data.az,
                        data.gx, data.gy, data.gz,
                        data.temp);
      Serial.println("📡 Data sent to XBee");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 Hz
    }
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // FreeRTOS handles everything
}
```

---

## Detailed Explanation

### 1. Setup Function

The `setup()` function initializes all hardware components and creates the FreeRTOS tasks.

#### Serial Communication
```cpp
Serial.begin(115200);
XBeeSerial.begin(9600, SERIAL_8N1, 16, 17);
```
- **Serial**: Debug output at 115200 baud
- **XBeeSerial**: Configured on UART2 with RX=GPIO16, TX=GPIO17

#### MPU6050 Configuration
```cpp
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
```

| Setting | Value | Purpose |
|---------|-------|---------|
| Accelerometer Range | ±8g | Suitable for rocket launch detection |
| Gyroscope Range | ±500°/s | Captures rotation during flight |
| Filter Bandwidth | 21 Hz | Reduces noise while maintaining responsiveness |

#### SD Card Initialization
```cpp
File file = SD.open("/data.txt", FILE_WRITE);
if (file) {
  file.println("ax,ay,az,gx,gy,gz,temp");
  file.close();
}
```
Creates a CSV file with headers for easy data analysis.

#### Task Creation

| Task Name | Stack Size | Priority | Core | Function |
|-----------|------------|----------|------|----------|
| Sensor Task | 2048 bytes | 2 (Medium) | Core 0 | Read IMU data |
| Log Task | 4096 bytes | 1 (Low) | Core 1 | Write to SD card |
| Alert Task | 2048 bytes | 3 (High) | Core 1 | Detect high-g events |
| XBee Task | 2048 bytes | 1 (Low) | Core 1 | Transmit telemetry |

**Priority Strategy:** Alert task has highest priority to ensure critical events are detected immediately.

---

### 2. Task 1: Sensor Reading (5 Hz)

```cpp
void sensorTask(void *pvParameters) {
  SensorData data;

  while (1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    data.temp = temp.temperature;

    xQueueSend(dataQueue, &data, portMAX_DELAY);
    vTaskDelay(200 / portTICK_PERIOD_MS);  // 5 Hz
  }
}
```

**Operation:**
1. Reads acceleration, gyroscope, and temperature from MPU6050
2. Packages data into `SensorData` structure
3. Sends to queue for other tasks to consume
4. Runs at **5 Hz** (every 200ms)

---

### 3. Task 2: SD Card Logging

```cpp
void logTask(void *pvParameters) {
  SensorData received;

  while (1) {
    if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
      File file = SD.open("/data.txt", FILE_APPEND);
      if (file) {
        file.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    received.ax, received.ay, received.az,
                    received.gx, received.gy, received.gz,
                    received.temp);
        file.close();
      }
    }
  }
}
```

**Operation:**
1. Receives data from queue (blocking call)
2. Opens SD card file in append mode
3. Writes CSV-formatted data
4. Closes file to ensure data is saved

**CSV Format:**
```
ax,ay,az,gx,gy,gz,temp
9.81,0.12,-0.05,0.01,0.02,0.00,25.30
```

---

### 4. Task 3: High-G Detection

```cpp
void alertTask(void *pvParameters) {
  SensorData received;

  while (1) {
    if (xQueuePeek(dataQueue, &received, portMAX_DELAY)) {
      // Detect launch or impact (>7g on X-axis)
      if (received.ax > 7.0) {
        Serial.println("⚠️  High Acceleration Detected on X-axis!");
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}
```

**Operation:**
1. Uses `xQueuePeek()` to read data without removing it from queue
2. Monitors X-axis acceleration for values > 7g
3. Triggers alert for launch/impact detection
4. **Highest priority** ensures immediate response

**Detection Threshold:** 7g is approximately 68.67 m/s²

---

### 5. Task 4: Telemetry Transmission (1 Hz)

```cpp
void xbeeTask(void *pvParameters) {
  SensorData data;

  while (1) {
    if (xQueuePeek(dataQueue, &data, portMAX_DELAY)) {
      XBeeSerial.printf("AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f T:%.2f\n",
                        data.ax, data.ay, data.az,
                        data.gx, data.gy, data.gz,
                        data.temp);
      Serial.println("📡 Data sent to XBee");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 Hz
    }
  }
}
```

**Operation:**
1. Peeks at latest sensor data
2. Formats as human-readable string
3. Transmits via XBee at **1 Hz** (every 1000ms)
4. Lower transmission rate conserves power and bandwidth

**Telemetry Packet Format:**
```
AX:9.81 AY:0.12 AZ:-0.05 GX:0.01 GY:0.02 GZ:0.00 T:25.30
```

---

## Queue Communication

The `dataQueue` acts as a buffer between the sensor task and consuming tasks:

```cpp
dataQueue = xQueueCreate(10, sizeof(SensorData));
```

- **Capacity:** 10 items
- **Item Size:** `sizeof(SensorData)` bytes
- **Type:** Thread-safe FreeRTOS queue

### Queue Operations

| Function | Task | Behavior |
|----------|------|----------|
| `xQueueSend()` | Sensor Task | Adds data to queue |
| `xQueueReceive()` | Log Task | Removes and consumes data |
| `xQueuePeek()` | Alert & XBee Tasks | Reads without removing |

---

## System Performance

### Sampling Rates

| Task | Rate | Period |
|------|------|--------|
| Sensor Reading | 5 Hz | 200 ms |
| SD Logging | 5 Hz | 200 ms |
| High-G Detection | 5 Hz | 200 ms |
| XBee Transmission | 1 Hz | 1000 ms |

### Data Flow Rate

- **Logged data points:** 5 per second
- **Transmitted packets:** 1 per second
- **CSV file growth:** ~50 bytes/second (estimated)

---

## Required Libraries

Add these to your Arduino IDE or PlatformIO:

```cpp
#include <Adafruit_MPU6050.h>  // MPU6050 sensor library
#include <Wire.h>               // I2C communication
#include <SD.h>                 // SD card file operations
#include <SPI.h>                // SPI communication for SD
```

**Installation:**
- **Arduino IDE:** Library Manager → Search for "Adafruit MPU6050"
- **PlatformIO:** Add to `platformio.ini`:
  ```ini
  lib_deps = 
      adafruit/Adafruit MPU6050@^2.2.4
  ```

---

## Usage Notes

### Power Consumption
- **Active Logging:** ~150-200 mA
- **XBee Transmission:** Additional ~45 mA during TX

### SD Card Recommendations
- Use **Class 10** or higher for reliable writes
- Format as **FAT32**
- Minimum **2GB** capacity

### Calibration
For accurate measurements, the MPU6050 should be calibrated before flight. The current configuration assumes factory calibration is sufficient.

### Safety Considerations
⚠️ **High-G Threshold:** The 7g threshold is suitable for model rockets. Adjust based on your specific application.

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "MPU6050 not found" | Check I2C connections (SDA/SCL) and power |
| "Card Mount Failed" | Verify SD card formatting and connections |
| No XBee transmission | Confirm GPIO 16/17 wiring and XBee pairing |
| Data gaps in log | Increase queue size or reduce logging frequency |

---

