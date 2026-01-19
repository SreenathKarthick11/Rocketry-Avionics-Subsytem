# Flight Computer (No GPS) - Complete Documentation

## Overview

This is a **complete flight computer system** for model rocketry, integrating multiple sensors, data logging, wireless telemetry, and real-time event detection. It uses **FreeRTOS** for concurrent task management, ensuring reliable data collection and transmission during flight.

### Key Features
- ✅ Multi-sensor integration (IMU + Barometer)
- ✅ High-speed SD card logging in CSV format
- ✅ Real-time wireless telemetry via XBee
- ✅ Altitude-based apogee detection
- ✅ FreeRTOS task management with priority scheduling
- ✅ Dual-core processing (ESP32)
- ✅ Thread-safe data sharing via queues

---

## Hardware Requirements

| Component | Model | Purpose |
|-----------|-------|---------|
| **Microcontroller** | ESP32 Dev Board | Main processor (dual-core) |
| **IMU Sensor** | MPU6050 | 6-axis motion tracking |
| **Barometer** | BMP280 | Altitude measurement |
| **SD Card Module** | SPI compatible | Flight data logging |
| **Radio Module** | XBee Series 1/2 | Wireless telemetry |
| **SD Card** | 4-32GB FAT32 | Data storage |
| **Battery** | LiPo 3.7V | Power supply |

---

## Complete Pin Connections

### I2C Bus (Custom Pins)

| Sensor | Pin | ESP32 GPIO | Description |
|--------|-----|------------|-------------|
| **MPU6050** | VCC | 3.3V | Power |
| | GND | GND | Ground |
| | SDA | **GPIO 13** | I2C Data (custom) |
| | SCL | **GPIO 14** | I2C Clock (custom) |
| **BMP280** | VCC | 3.3V | Power |
| | GND | GND | Ground |
| | SDA | **GPIO 13** | I2C Data (shared) |
| | SCL | **GPIO 14** | I2C Clock (shared) |

### SD Card Module (SPI)

| SD Pin | ESP32 Pin | Description |
|--------|-----------|-------------|
| **MISO** | GPIO 19 | Master In Slave Out |
| **MOSI** | GPIO 23 | Master Out Slave In |
| **SCK** | GPIO 18 | SPI Clock |
| **CS** | GPIO 5 | Chip Select |
| **VCC** | 3.3V | Power |
| **GND** | GND | Ground |

### XBee Module (UART2)

| XBee Pin | ESP32 Pin | Description |
|----------|-----------|-------------|
| **DOUT** | GPIO 16 (RX2) | XBee TX → ESP32 RX |
| **DIN** | GPIO 17 (TX2) | ESP32 TX → XBee RX |
| **VCC** | 3.3V | Power |
| **GND** | GND | Ground |

### Complete Wiring Diagram

```
                    ESP32 Flight Computer
        ┌───────────────────────────────────────────┐
        │                                           │
I2C Bus │  SDA (GPIO 13) ◄──┬── MPU6050            │
        │  SCL (GPIO 14) ◄──┼── BMP280             │
        │                   │                       │
SPI Bus │  MISO (GPIO 19) ◄─┤                      │
        │  MOSI (GPIO 23) ──┤                      │
        │  SCK  (GPIO 18) ──┼── SD Card Module     │
        │  CS   (GPIO 5)  ──┤                      │
        │                   │                       │
UART2   │  RX2  (GPIO 16) ◄─┤                      │
        │  TX2  (GPIO 17) ──┼── XBee Radio         │
        │                   │                       │
Power   │  3.3V ────────────┴── All Modules        │
        │  GND  ────────────── Common Ground       │
        └───────────────────────────────────────────┘
```

---

## Required Libraries

Install via Arduino IDE Library Manager:

| Library | Provider | Version |
|---------|----------|---------|
| **Adafruit MPU6050** | Adafruit | Latest |
| **Adafruit BMP280** | Adafruit | Latest |
| **Adafruit Unified Sensor** | Adafruit | Latest (dependency) |
| **Wire** | Arduino | Built-in |
| **SD** | Arduino | Built-in |
| **SPI** | Arduino | Built-in |

---

## Code Breakdown

### 1. Library Includes

```cpp
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
```

**Explanation:**

| Library | Purpose |
|---------|---------|
| `Adafruit_MPU6050.h` | Interface for 6-axis IMU sensor |
| `Wire.h` | I2C communication protocol |
| `SD.h` | SD card file system operations |
| `SPI.h` | SPI communication for SD card |
| `Adafruit_BMP280.h` | Barometric pressure/altitude sensor |

---

### 2. Pin Definitions

```cpp
// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define CUSTOM_SDA 13
#define CUSTOM_SCL 14
```

**Why Custom I2C Pins?**

| Reason | Explanation |
|--------|-------------|
| **Flexibility** | Avoids conflicts with other peripherals |
| **Hardware Design** | PCB layout optimization |
| **Multiple I2C Buses** | Can use default pins for other sensors |

**Default vs Custom I2C:**

| Type | SDA | SCL |
|------|-----|-----|
| **Default** | GPIO 21 | GPIO 22 |
| **Custom (This Project)** | GPIO 13 | GPIO 14 |

---

### 3. Global Objects

```cpp
// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
QueueHandle_t dataQueue;
HardwareSerial XBeeSerial(2);  // UART2 for XBee communication
```

**Object Overview:**

| Object | Type | Purpose |
|--------|------|---------|
| `mpu` | `Adafruit_MPU6050` | MPU6050 sensor interface |
| `bmp` | `Adafruit_BMP280` | BMP280 sensor interface |
| `dataQueue` | `QueueHandle_t` | FreeRTOS queue for task communication |
| `XBeeSerial` | `HardwareSerial(2)` | UART2 for XBee telemetry |

**What is a FreeRTOS Queue?**

```
┌──────────────────────────────────────────┐
│          FreeRTOS Queue                  │
├──────────────────────────────────────────┤
│  [Data1] [Data2] [Data3] [____] [____]  │
│     ↑                            ↑       │
│   Send here                Read here     │
└──────────────────────────────────────────┘
```

- **Thread-safe** data sharing between tasks
- **Blocking** operations - tasks wait if queue full/empty
- **FIFO** - First In, First Out

---

### 4. Data Structure

```cpp
// ============================================================================
// DATA STRUCTURE
// ============================================================================

// Sensor data packet for inter-task communication
typedef struct {
  unsigned long time;      // Timestamp in milliseconds
  float ax, ay, az;        // Acceleration (m/s²)
  float gx, gy, gz;        // Gyroscope (rad/s)
  float temp;              // Temperature (°C)
  float altitude;          // Altitude (meters)
} SensorData;
```

**Structure Layout:**

| Field | Type | Size | Description |
|-------|------|------|-------------|
| `time` | `unsigned long` | 4 bytes | Milliseconds since boot |
| `ax`, `ay`, `az` | `float` | 4 bytes each | Acceleration (X, Y, Z axes) |
| `gx`, `gy`, `gz` | `float` | 4 bytes each | Angular velocity (X, Y, Z axes) |
| `temp` | `float` | 4 bytes | Temperature |
| `altitude` | `float` | 4 bytes | Calculated altitude |
| **Total** | - | **40 bytes** | Complete packet size |

**Why Use a Structure?**
- ✅ **Organized** - All related data in one package
- ✅ **Type-safe** - Compiler checks data types
- ✅ **Easy to pass** - Single variable contains all data
- ✅ **Queue-compatible** - Perfect for FreeRTOS queues

**Example Data Packet:**
```cpp
SensorData packet = {
  .time = 5234,
  .ax = 12.34,
  .ay = 5.67,
  .az = 9.81,
  .gx = 0.12,
  .gy = -0.05,
  .gz = 0.03,
  .temp = 24.5,
  .altitude = 125.8
};
```

---

## Setup Function

### Complete Initialization Sequence

```cpp
// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  XBeeSerial.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(CUSTOM_SDA, CUSTOM_SCL);

  // ---- Initialize MPU6050 ----
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ---- Initialize BMP280 ----
  if (!bmp.begin(0x76, &Wire)) {
    Serial.println("⚠️  Could not find BMP280 sensor!");
  } else {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("✅ BMP280 initialized");
  }

  // ---- Initialize SD Card ----
  if (!SD.begin()) {
    Serial.println("❌ Card Mount Failed");
    return;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("❌ No SD card attached");
    return;
  }

  // Create data file with CSV header
  File file = SD.open("/data.txt", FILE_WRITE);
  if (file) {
    file.println("timestamp,ax,ay,az,gx,gy,gz,temp,altitude");
    file.close();
    Serial.println("✅ SD file initialized");
  } else {
    Serial.println("❌ Failed to create file");
  }

  // ---- Create FreeRTOS Queue ----
  dataQueue = xQueueCreate(10, sizeof(SensorData));

  // ---- Create Tasks ----
  // Sensor reading: Priority 2, Core 0
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 2, NULL, 0);
  
  // Data logging: Priority 1, Core 1
  xTaskCreatePinnedToCore(logTask, "Log Task", 4096, NULL, 1, NULL, 1);
  
  // Alert monitoring: Priority 3 (highest), Core 1
  xTaskCreatePinnedToCore(alertTask, "Alert Task", 2048, NULL, 3, NULL, 1);
  
  // XBee telemetry: Priority 1, Core 1
  xTaskCreatePinnedToCore(xbeeTask, "XBee Task", 2048, NULL, 1, NULL, 1);
}
```

---

### Step 1: Communication Setup

```cpp
Serial.begin(115200);
XBeeSerial.begin(9600, SERIAL_8N1, 16, 17);
Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
```

**Initialization Table:**

| Interface | Baud/Speed | Pins | Purpose |
|-----------|------------|------|---------|
| **USB Serial** | 115200 | Default | Debug monitoring |
| **XBee UART2** | 9600 | RX=16, TX=17 | Wireless telemetry |
| **I2C** | 100kHz | SDA=13, SCL=14 | Sensor communication |

---

### Step 2: MPU6050 Initialization

```cpp
// ---- Initialize MPU6050 ----
if (!mpu.begin()) {
  Serial.println("❌ MPU6050 not found!");
  while (1) delay(10);
}

mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
```

**MPU6050 Configuration:**

| Setting | Value | Range | Purpose |
|---------|-------|-------|---------|
| **Accelerometer Range** | `MPU6050_RANGE_8_G` | ±8g | Measures up to 8g acceleration |
| **Gyroscope Range** | `MPU6050_RANGE_500_DEG` | ±500°/s | Measures up to 500°/s rotation |
| **Filter Bandwidth** | `MPU6050_BAND_21_HZ` | 21 Hz | Low-pass filter cutoff |

**Why These Settings?**

| Choice | Reason |
|--------|--------|
| **8g range** | Model rockets: 5-10g during launch, 2-5g landing |
| **500°/s** | Captures fast tumbling/spin recovery |
| **21 Hz filter** | Removes vibration noise, keeps real motion |

**Available Accelerometer Ranges:**
```cpp
MPU6050_RANGE_2_G   // ±2g  (high precision, low range)
MPU6050_RANGE_4_G   // ±4g
MPU6050_RANGE_8_G   // ±8g  (balanced) ← USED
MPU6050_RANGE_16_G  // ±16g (low precision, high range)
```

---

### Step 3: BMP280 Initialization

```cpp
// ---- Initialize BMP280 ----
if (!bmp.begin(0x76, &Wire)) {
  Serial.println("⚠️  Could not find BMP280 sensor!");
} else {
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  Serial.println("✅ BMP280 initialized");
}
```

**BMP280 I2C Address:**

| Address | Jumper Setting | Default |
|---------|----------------|---------|
| **0x76** | SDO → GND | ← Most common |
| **0x77** | SDO → VCC | Alternative |

**Sampling Configuration:**

| Parameter | Setting | Description |
|-----------|---------|-------------|
| **Mode** | `MODE_NORMAL` | Continuous measurement |
| **Temperature** | `SAMPLING_X2` | 2x oversampling (±0.5°C accuracy) |
| **Pressure** | `SAMPLING_X16` | 16x oversampling (±0.12 hPa) |
| **Filter** | `FILTER_X16` | 16-sample averaging |
| **Standby** | `STANDBY_MS_500` | 500ms between measurements |

**Why These Settings?**

```
Higher Oversampling = Better Accuracy + Slower Speed
SAMPLING_X16: ±0.12 hPa ≈ ±1 meter altitude accuracy
```

**Oversampling Options:**
```cpp
SAMPLING_NONE  // No oversampling (fast, noisy)
SAMPLING_X1    // 1x (default)
SAMPLING_X2    // 2x  ← Temperature
SAMPLING_X4    // 4x
SAMPLING_X8    // 8x
SAMPLING_X16   // 16x ← Pressure (best accuracy)
```

---

### Step 4: SD Card Initialization

```cpp
// ---- Initialize SD Card ----
if (!SD.begin()) {
  Serial.println("❌ Card Mount Failed");
  return;
}

if (SD.cardType() == CARD_NONE) {
  Serial.println("❌ No SD card attached");
  return;
}

// Create data file with CSV header
File file = SD.open("/data.txt", FILE_WRITE);
if (file) {
  file.println("timestamp,ax,ay,az,gx,gy,gz,temp,altitude");
  file.close();
  Serial.println("✅ SD file initialized");
} else {
  Serial.println("❌ Failed to create file");
}
```

**CSV File Structure:**

```csv
timestamp,ax,ay,az,gx,gy,gz,temp,altitude
1234,0.12,-0.08,9.78,0.00,0.01,-0.01,23.45,125.5
1434,0.15,-0.10,9.81,0.01,0.00,0.00,23.47,128.3
1634,8.45,-0.12,9.82,0.02,0.01,-0.01,23.48,156.2
```

**Column Definitions:**

| Column | Unit | Description |
|--------|------|-------------|
| `timestamp` | ms | Time since boot |
| `ax, ay, az` | m/s² | Acceleration (3 axes) |
| `gx, gy, gz` | rad/s | Angular velocity (3 axes) |
| `temp` | °C | Temperature |
| `altitude` | m | Calculated altitude |

---

### Step 5: Create FreeRTOS Queue

```cpp
// ---- Create FreeRTOS Queue ----
dataQueue = xQueueCreate(10, sizeof(SensorData));
```

**Queue Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Length** | 10 | Can hold 10 data packets |
| **Item Size** | `sizeof(SensorData)` | 40 bytes per packet |
| **Total Buffer** | 400 bytes | 10 × 40 bytes |

**Why 10 Items?**
- Sensor task produces at 5 Hz (200ms)
- Consumer tasks process within 200ms
- 10 items = 2 second buffer
- Prevents data loss during SD card slowdowns

---

### Step 6: Create FreeRTOS Tasks

```cpp
// ---- Create Tasks ----
// Sensor reading: Priority 2, Core 0
xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 2, NULL, 0);

// Data logging: Priority 1, Core 1
xTaskCreatePinnedToCore(logTask, "Log Task", 4096, NULL, 1, NULL, 1);

// Alert monitoring: Priority 3 (highest), Core 1
xTaskCreatePinnedToCore(alertTask, "Alert Task", 2048, NULL, 3, NULL, 1);

// XBee telemetry: Priority 1, Core 1
xTaskCreatePinnedToCore(xbeeTask, "XBee Task", 2048, NULL, 1, NULL, 1);
```

**Task Configuration Table:**

| Task | Priority | Stack (bytes) | Core | Purpose |
|------|----------|---------------|------|---------|
| **sensorTask** | 2 | 2048 | 0 | Read sensors at 5 Hz |
| **logTask** | 1 | 4096 | 1 | Write to SD card |
| **alertTask** | 3 (highest) | 2048 | 1 | Detect critical events |
| **xbeeTask** | 1 | 2048 | 1 | Transmit telemetry at 1 Hz |

**Priority System:**

```
Priority 3 (Highest) ─┐
                      ├─ alertTask (critical events)
Priority 2           ─┤
                      ├─ sensorTask (data collection)
Priority 1 (Lowest)  ─┤
                      ├─ logTask, xbeeTask (background)
```

**Core Assignment Strategy:**

```
ESP32 Dual-Core Architecture:

Core 0:                    Core 1:
┌──────────────┐          ┌──────────────┐
│ sensorTask   │          │ logTask      │
│ (Priority 2) │          │ (Priority 1) │
│              │          ├──────────────┤
│ Reads        │          │ alertTask    │
│ sensors      │   Queue  │ (Priority 3) │
│ continuously │◄────────►├──────────────┤
│              │          │ xbeeTask     │
│              │          │ (Priority 1) │
└──────────────┘          └──────────────┘
```

**Why This Assignment?**
- **Core 0**: Dedicated to time-critical sensor reading
- **Core 1**: Handles I/O operations (SD, XBee, alerts)
- **Prevents**: Sensor timing jitter from SD card delays

---

## Task 1: Sensor Data Acquisition

```cpp
// ============================================================================
// TASK 1: SENSOR DATA ACQUISITION
// ============================================================================
// Reads all sensors at 5 Hz and pushes data to queue

void sensorTask(void *pvParameters) {
  SensorData data;

  while (1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long timestamp = millis();
    
    // Populate data structure
    data.time = timestamp;
    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    data.temp = temp.temperature;
    data.altitude = bmp.readAltitude(1013.25);  // Sea level pressure in hPa

    // Send to queue (blocks if queue is full)
    xQueueSend(dataQueue, &data, portMAX_DELAY);
    
    vTaskDelay(200 / portTICK_PERIOD_MS);  // 5 Hz update rate
  }
}
```

**Task Properties:**

| Property | Value | Explanation |
|----------|-------|-------------|
| **Update Rate** | 5 Hz (200ms) | Balance between data resolution and storage |
| **Priority** | 2 (Medium) | Important but not critical |
| **Core** | 0 | Dedicated core for consistent timing |
| **Stack** | 2048 bytes | Enough for sensor objects and calculations |

### Step-by-Step Breakdown

#### Step 1: Read MPU6050

```cpp
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);
```

**Event Structure:**
```cpp
struct sensors_event_t {
  sensors_vec_t acceleration;  // .x, .y, .z in m/s²
  sensors_vec_t gyro;          // .x, .y, .z in rad/s
  float temperature;           // in °C
};
```

---

#### Step 2: Get Timestamp

```cpp
unsigned long timestamp = millis();
```

**Why `millis()`?**
- Returns milliseconds since boot
- Useful for calculating flight duration
- Synchronized across all tasks
- Example: Launch at t=1000ms, apogee at t=8500ms → 7.5s flight time

---

#### Step 3: Populate Data Packet

```cpp
data.time = timestamp;
data.ax = a.acceleration.x;
data.ay = a.acceleration.y;
data.az = a.acceleration.z;
data.gx = g.gyro.x;
data.gy = g.gyro.y;
data.gz = g.gyro.z;
data.temp = temp.temperature;
data.altitude = bmp.readAltitude(1013.25);
```

**Altitude Calculation:**

```cpp
data.altitude = bmp.readAltitude(1013.25);
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `1013.25` | hPa | Standard sea level pressure |
| Returns | meters | Altitude above sea level |

**Barometric Altitude Formula:**
```
h = 44330 * (1 - (P/P0)^0.1903)

Where:
h  = altitude (meters)
P  = measured pressure
P0 = sea level pressure (1013.25 hPa)
```

**Accuracy Considerations:**
- ±1 meter accuracy with proper calibration
- Affected by weather (pressure changes)
- Best to calibrate at launch site: `bmp.readAltitude(localPressure)`

---

#### Step 4: Send to Queue

```cpp
xQueueSend(dataQueue, &data, portMAX_DELAY);
```

**Parameters:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `dataQueue` | Queue handle | Which queue to send to |
| `&data` | Pointer | Address of data to copy |
| `portMAX_DELAY` | Infinite | Wait forever if queue full |

**Queue Operation:**

```
Before Send:
Queue: [Data1] [Data2] [____] [____] [____]
                         ↑
                    Send here

After Send:
Queue: [Data1] [Data2] [Data3] [____] [____]
```

**What if Queue is Full?**
- Task **blocks** (waits) until space available
- `portMAX_DELAY` = wait forever
- Alternative: `0` = return immediately if full

---

#### Step 5: Delay for Timing

```cpp
vTaskDelay(200 / portTICK_PERIOD_MS);  // 5 Hz update rate
```

**Timing Calculation:**

```
5 Hz = 5 samples per second
1 second / 5 samples = 0.2 seconds = 200 milliseconds

portTICK_PERIOD_MS converts ms to FreeRTOS ticks
```

**Why Not `delay()`?**

| Function | Type | Use Case |
|----------|------|----------|
| `delay()` | Blocks everything | Single-threaded Arduino |
| `vTaskDelay()` | Blocks only this task | FreeRTOS multitasking |

---

## Task 2: SD Card Data Logging

```cpp
// ============================================================================
// TASK 2: SD CARD DATA LOGGING
// ============================================================================
// Receives data from queue and writes to SD card in CSV format

void logTask(void *pvParameters) {
  SensorData received;

  while (1) {
    if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
      File file = SD.open("/data.txt", FILE_APPEND);
      if (file) {
        file.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    received.time,
                    received.ax, received.ay, received.az,
                    received.gx, received.gy, received.gz,
                    received.temp, received.altitude);
        file.close();
        Serial.println("💾 Data logged to SD");
      } else {
        Serial.println("❌ Failed to open file for appending");
      }
    }
  }
}
```

**Task Properties:**

| Property | Value | Reason |
|----------|-------|--------|
| **Priority** | 1 (Low) | Background operation |
| **Core** | 1 | Dedicated I/O core |
| **Stack** | 4096 bytes | Large for file operations |

### Breakdown

#### Step 1: Wait for Data

```cpp
if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
```

**Queue Receive:**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `dataQueue` | Queue handle | Which queue to read from |
| `&received` | Pointer | Where to copy data |
| `portMAX_DELAY` | Infinite | Wait forever if queue empty |

**Blocking Behavior:**
```
Queue Empty:
Task waits here ──►  xQueueReceive() ──► (blocked)
                                          ↓
Queue Has Data:                          ↓
[Data arrives] ─────────────────────────►  Continue
```

---

#### Step 2: Open File for Append

```cpp
File file = SD.open("/data.txt", FILE_APPEND);
```

**File Modes:**

| Mode | Behavior |
|------|----------|
| `FILE_WRITE` | Create or overwrite |
| `FILE_APPEND` | Create or add to end |
| `FILE_READ` | Read only |

---

#### Step 3: Write CSV Row

```cpp
file.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            received.time,
            received.ax, received.ay, received.az,
            received.gx, received.gy, received.gz,
            received.temp, received.altitude);
```

**Format String Breakdown:**

| Format | Type | Example | Description |
|--------|------|---------|-------------|
| `%lu` | unsigned long | `5234` | Timestamp |
| `%.2f` | float (2 decimals) | `12.34` | Sensor values |
| `\n` | newline | - | End of row |

**Example Output:**
```csv
5234,12.34,5.67,9.81,0.12,-0.05,0.03,24.50,125.80
```

---

#### Step 4: Close File

```cpp
file.close();
```

**Why Close Every Time?**
- ✅ **Data safety**: Flushes buffer to SD card
- ✅ **Crash recovery**: Data saved even if power lost
- ❌ **Slower**: Opening/closing has overhead

**Alternative (Faster but Riskier):**
```cpp
// Open once in setup
File logFile = SD.open("/data.txt", FILE_APPEND);

// In loop, just write
logFile.printf(...);
logFile.flush();  // Periodic flush
```

---

## Task 3: Flight Event Detection

```cpp
// ============================================================================
// TASK 3: FLIGHT EVENT DETECTION
// ============================================================================
// Monitors altitude for apogee detection

void alertTask(void *pvParameters) {
  SensorData received;

  while (1) {
    if (xQueuePeek(dataQueue, &received, portMAX_DELAY)) {
      // Apogee detection threshold (100m in this example)
      if (received.altitude > 100.0) {
        Serial.println("🚀 Reached peak altitude!");
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}
```

**Task Properties:**

| Property | Value | Reason |
|----------|-------|--------|
| **Priority** | 3 (Highest) | Critical events need immediate detection |
| **Core** | 1 | Can interrupt logging/telemetry |
| **Stack** | 2048 bytes | Minimal processing |

### Key Difference: `xQueuePeek()` vs `xQueueReceive()`

```cpp
xQueuePeek(dataQueue, &received, portMAX_DELAY);
```

**Comparison:**

| Function | Removes from Queue? | Use Case |
|----------|---------------------|----------|
| `xQueueReceive()` | ✅ Yes | Consume data (logging) |
| `xQueuePeek()` | ❌ No | Inspect without consuming (alerts) |

**Why Peek?**
```
Queue: [Data1] [Data2] [Data3]
                 ↑
            Peek reads here
            But doesn't remove

Still available for:
- logTask (xQueueReceive)
- xbeeTask (xQueuePeek)
```

---

### Apogee Detection Logic

```cpp
if (received.altitude > 100.0) {
  Serial.println("🚀 Reached peak altitude!");
}
```

**Simple Threshold Detection:**

| Condition | Action |
|-----------|--------|
| Altitude > 100m | Trigger alert |
| Altitude ≤ 100m | No action |

**⚠️ Limitation:**
- This is a **simple example** - not production-ready
- Real apogee detection needs:
  - Vertical velocity calculation
  - Acceleration trending
  - Hysteresis (avoid false triggers)

**Better Apogee Detection:**
```cpp
float lastAltitude = 0;
bool apogeeDetected = false;

// Detect when altitude stops increasing
if (received.altitude < lastAltitude && !apogeeDetected) {
  Serial.println("🚀 APOGEE DETECTED!");
  apogeeDetected = true;
  // Trigger parachute deployment here
}
lastAltitude = received.altitude;
```

---

## Task 4: XBee Telemetry

```cpp
// ============================================================================
// TASK 4: XBEE TELEMETRY
// ============================================================================
// Transmits sensor data via XBee radio at 1 Hz

void xbeeTask(void *pvParameters) {
  SensorData data;

  while (1) {
    if (xQueuePeek(dataQueue, &data, portMAX_DELAY)) {
      // Format and transmit telemetry packet
      XBeeSerial.printf("Time:%lu AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f T:%.2f Alt:%.2f\n",
                        data.time,
                        data.ax, data.ay, data.az,
                        data.gx, data.gy, data.gz,
                        data.temp, data.altitude);
      Serial.println("📡 Data sent to XBee");
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 Hz transmission rate
    }
  }
}
```

**Task Properties:**

| Property | Value | Reason |
|----------|-------|--------|
| **Priority** | 1 (Low) | Not time-critical |
| **Transmission Rate** | 1 Hz (1000ms) | Conserves bandwidth |
| **Core** | 1 | I/O operations |

### Telemetry Packet Format

```cpp
XBeeSerial.printf("Time:%lu AX:%.2f AY:%.2f AZ:%.2f GX:%.2f GY:%.2f GZ:%.2f T:%.2f Alt:%.2f\n",
                  data.time, data.ax, data.ay, data.az,
                  data.gx, data.gy, data.gz,
                  data.temp, data.altitude);
```

**Example Transmitted String:**
```
Time:5234 AX:12.34 AY:5.67 AZ:9.81 GX:0.12 GY:-0.05 GZ:0.03 T:24.50 Alt:125.80
```

**Packet Size Calculation:**
```
"Time:5234 AX:12.34 AY:5.67 AZ:9.81 GX:0.12 GY:-0.05 GZ:0.03 T:24.50 Alt:125.80\n"
= Approximately 85 characters = 85 bytes
```

**Bandwidth Usage:**
```
1 packet/second × 85 bytes = 85 bytes/second
At 9600 baud = 960 bytes/second max
Usage: 85/960 = 8.85% bandwidth
```

---

### Why 1 Hz for Telemetry?

| Frequency | Pros | Cons |
|-----------|------|------|
| **5 Hz (same as sensors)** | Real-time | Excessive bandwidth, battery drain |
| **1 Hz (current)** | Good balance | Slight delay in updates |
| **0.5 Hz** | Conserves power | Too slow for monitoring |

---

## Main Loop

```cpp
// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // FreeRTOS scheduler handles all tasks
}
```

**Why Empty?**
- All work done by FreeRTOS tasks
- Scheduler automatically manages:
  - Task switching
  - Priority handling
  - Core assignment
  - Timing

---

## System Architecture

### Task Communication Flow

```
┌─────────────────────────────────────────────────────────┐
│                    FLIGHT COMPUTER                      │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Core 0:                    Core 1:                     │
│  ┌──────────────┐          ┌──────────────┐            │
│  │ sensorTask   │          │ alertTask    │            │
│  │ (Priority 2) │          │ (Priority 3) │            │
│  │              │          │              │            │
│  │ Read MPU6050 │          │ Detect       │            │
│  │ Read BMP280  │          │ Apogee       │            │
│  │              │          │              │            │
│  │ Every 200ms  │          └──────┬───────┘            │
│  └──────┬───────┘                 │                    │
│         │                         │                    │
│         │  xQueueSend()          │ xQueuePeek()       │
│         ▼                         ▼                    │
│  ┌─────────────────────────────────────┐              │
│  │       FreeRTOS Queue (10 items)     │              │
│  │  [Data] [Data] [Data] [__] [__]    │              │
│  └─────────────────────────────────────┘              │
│         │                         │                    │
│         │ xQueueReceive()        │ xQueuePeek()       │
│         ▼                         ▼                    │
│  ┌──────────────┐          ┌──────────────┐            │
│  │ logTask      │          │ xbeeTask     │            │
│  │ (Priority 1) │          │ (Priority 1) │            │
│  │              │          │              │            │
│  │ Write to     │          │ Transmit via │            │
│  │ SD Card      │          │ XBee Radio   │            │
│  │              │          │              │            │
│  │ Every packet │          │ Every 1000ms │            │
│  └──────────────┘          └──────────────┘            │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Sample Flight Data

### Pre-Launch (On Pad)

```csv
timestamp,ax,ay,az,gx,gy,gz,temp,altitude
1000,0.12,-0.08,9.78,0.00,0.01,-0.01,23.45,0.00
1200,0.15,-0.10,9.81,0.01,0.00,0.00,23.47,0.00
1400,0.11,-0.09,9.79,-0.01,0.01,0.00,23.46,0.00
```

### Launch Detection (High Acceleration)

```csv
2000,45.23,1.23,12.45,0.34,0.12,0.05,23.50,5.20
2200,67.89,2.34,15.67,0.45,0.23,0.08,23.55,15.80
2400,82.45,3.12,18.90,0.56,0.34,0.12,23.60,32.40
```

### Coast Phase (Ascending)

```csv
4000,2.34,0.45,9.81,0.12,0.05,0.02,22.10,145.60
4200,1.89,0.38,9.80,0.10,0.04,0.01,21.80,178.30
4400,1.45,0.32,9.79,0.08,0.03,0.01,21.50,205.70
```

### Apogee (Peak Altitude)

```csv
6000,0.23,0.05,9.81,0.02,0.01,0.00,20.30,312.50
6200,0.18,0.04,9.80,0.01,0.01,0.00,20.20,312.80
6400,-0.15,-0.03,9.79,-0.01,0.00,0.00,20.10,312.60
```

### Descent (Parachute Deployed)

```csv
8000,-1.23,-0.34,8.56,-0.12,-0.05,-0.02,21.50,198.40
8200,-1.18,-0.32,8.54,-0.11,-0.04,-0.02,21.80,165.20
8400,-1.15,-0.30,8.52,-0.10,-0.04,-0.01,22.10,132.80
```

---

## Troubleshooting

### Issue 1: SD Card Write Failures

**Symptoms:**
```
❌ Failed to open file for appending
```

**Solutions:**

| Check | Fix |
|-------|-----|
| Card formatted? | Format as FAT32 |
| Write protect? | Check physical switch |
| Card full? | Delete old files |
| Bad connection? | Reseat SD card |

---

### Issue 2: No XBee Transmission

**Symptoms:**
```
📡 Data sent to XBee
(But ground station receives nothing)
```

**Solutions:**

| Check | Fix |
|-------|-----|
| XBee powered? | Check 3.3V |
| PAN ID match? | Configure via XCTU |
| Antenna attached? | Screw on antenna |
| Baud rate? | Verify 9600 on both sides |

---

### Issue 3: Sensor Read Failures

**Symptoms:**
```
❌ MPU6050 not found!
⚠️ Could not find BMP280 sensor!
```

**Solutions:**

| Sensor | Common Issues |
|--------|---------------|
| **MPU6050** | Wrong I2C address (try 0x68 or 0x69) |
| **BMP280** | Wrong I2C address (try 0x76 or 0x77) |
| **Both** | SDA/SCL swapped, pull-up resistors needed |

---

### Issue 4: Task Stack Overflow

**Symptoms:**
```
***ERROR*** A stack overflow in task logTask has been detected.
```

**Solutions:**
```cpp
// Increase stack size
xTaskCreatePinnedToCore(logTask, "Log Task", 8192, NULL, 1, NULL, 1);
//                                            ^^^^ was 4096
```

---

## Performance Optimization

### 1. Reduce SD Card Opens/Closes

**Current (Slow):**
```cpp
// Opens and closes every write
File file = SD.open("/data.txt", FILE_APPEND);
file.printf(...);
file.close();
```

**Optimized (Fast):**
```cpp
// Global file handle
File logFile;

// Open once in setup
logFile = SD.open("/data.txt", FILE_APPEND);

// In task, just write
logFile.printf(...);
logFile.flush();  // Every 10 writes
```

---

### 2. Buffer XBee Transmissions

**Current:**
```cpp
// Transmits every piece of data
XBeeSerial.printf("Time:%lu AX:%.2f...\n", ...);
```

**Optimized:**
```cpp
char buffer[256];
int bufferPos = 0;

// Accumulate data
bufferPos += sprintf(buffer + bufferPos, "Time:%lu...\n", ...);

// Send when buffer nearly full
if (bufferPos > 200) {
  XBeeSerial.write(buffer, bufferPos);
  bufferPos = 0;
}
```

---

### 3. Use Binary Data Format

**Text (Current):**
```
"Time:5234 AX:12.34 AY:5.67..."  = 85 bytes
```

**Binary:**
```cpp
struct {
  uint32_t time;
  float ax, ay, az, gx, gy, gz, temp, altitude;
} packet;  // = 40 bytes (47% smaller!)
```

---

## Applications in Rocketry

| Feature | Use Case |
|---------|----------|
| **Accelerometer** | Launch detection, impact detection |
| **Gyroscope** | Spin rate, stability monitoring |
| **Barometer** | Apogee detection, altitude tracking |
| **SD Logging** | Post-flight analysis, flight profile |
| **Telemetry** | Real-time monitoring, range safety |
| **Multi-tasking** | Simultaneous logging + transmission |

---