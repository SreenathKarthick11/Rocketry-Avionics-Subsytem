# Simple Data Logger - ESP32 Flight Computer

## Overview

This is a streamlined flight data logger designed for rocket or high-acceleration vehicle applications. It continuously records IMU sensor data to an SD card while monitoring for high-g events that indicate launch or impact.

---

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **ESP32** | Main microcontroller with dual-core processor |
| **MPU6050** | 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) |
| **SD Card Module** | Non-volatile storage for flight data |
| **SD Card** | FAT32 formatted, Class 10 recommended |

---

## System Architecture

This system uses **FreeRTOS** to run three concurrent tasks that work together through a shared queue:

```
┌──────────────────┐
│  Sensor Task     │
│  (Priority: 2)   │ ──┐
│  Core 0          │   │
│  5 Hz sampling   │   │
└──────────────────┘   │
                       │
                       ├──> ┌────────────────┐
                       │    │  Data Queue    │
┌──────────────────┐   │    │  (10 items)    │
│   Log Task       │ <─┼────│  Thread-safe   │
│  (Priority: 1)   │   │    └────────────────┘
│  Core 1          │   │
│  Writes to SD    │   │
└──────────────────┘   │
                       │
┌──────────────────┐   │
│  Alert Task      │ <─┘
│  (Priority: 3)   │
│  Core 1          │
│  Event detection │
└──────────────────┘
```

**Key Design Decisions:**
- **Sensor Task** runs on Core 0 for dedicated data acquisition
- **Log & Alert Tasks** run on Core 1 for parallel processing
- **Alert Task** has highest priority for immediate event detection

---

## Data Structure

Sensor readings are encapsulated in a compact structure for efficient queue transmission:

```cpp
typedef struct {
  float ax, ay, az;     // Acceleration (m/s²)
  float gx, gy, gz;     // Angular velocity (rad/s)
  float temp;           // Temperature (°C)
} SensorData;
```

**Memory Footprint:** 7 floats × 4 bytes = 28 bytes per sample

---

## Complete Code

```cpp
/*
 * ============================================================================
 * SIMPLE DATA LOGGER (YOUR SUBMITTED CODE - IMPROVED)
 * ============================================================================
 * Purpose: Basic flight data logger with event detection
 * Hardware: ESP32 + MPU6050 + SD Card
 * 
 * Features:
 * - Continuous IMU data logging to SD card
 * - High-g event detection
 * - FreeRTOS multi-tasking
 * 
 * Note: This is your original code with improved comments and formatting
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

// ============================================================================
// DATA STRUCTURE
// ============================================================================

typedef struct {
  float ax, ay, az;     // Acceleration (m/s²)
  float gx, gy, gz;     // Angular velocity (rad/s)
  float temp;           // Temperature (°C)
} SensorData;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // ---- Initialize MPU6050 ----
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    while (1) delay(10);
  }

  // Configure sensor ranges
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

  // Create file with CSV header
  File file = SD.open("/data.txt", FILE_WRITE);
  if (file) {
    file.println("ax,ay,az,gx,gy,gz,temp");
    file.close();
    Serial.println("✅ SD card ready");
  }

  // ---- Create FreeRTOS Components ----
  dataQueue = xQueueCreate(10, sizeof(SensorData));

  // Sensor reading task (Priority 2, Core 0)
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 2, NULL, 0);
  
  // Data logging task (Priority 1, Core 1)
  xTaskCreatePinnedToCore(logTask, "Log Task", 4096, NULL, 1, NULL, 1);
  
  // Alert monitoring task (Priority 3, Core 1)
  xTaskCreatePinnedToCore(alertTask, "Alert Task", 2048, NULL, 3, NULL, 1);
}

// ============================================================================
// TASK 1: SENSOR DATA COLLECTION
// ============================================================================

void sensorTask(void *pvParameters) {
  SensorData data;

  while (1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Package sensor data
    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    data.temp = temp.temperature;

    // Send to queue
    xQueueSend(dataQueue, &data, portMAX_DELAY);
    
    // 200ms delay = 5 Hz sampling rate
    delay(200);
  }
}

// ============================================================================
// TASK 2: SD CARD LOGGING
// ============================================================================

void logTask(void *pvParameters) {
  SensorData received;

  while (1) {
    // Wait for data from queue
    if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
      File file = SD.open("/data.txt", FILE_APPEND);
      if (file) {
        // Write CSV row
        file.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    received.ax, received.ay, received.az,
                    received.gx, received.gy, received.gz,
                    received.temp);
        file.close();
        Serial.println("💾 Logged");
      } else {
        Serial.println("❌ Failed to open file for appending");
      }
    }
  }
}

// ============================================================================
// TASK 3: HIGH ACCELERATION ALERT
// ============================================================================

void alertTask(void *pvParameters) {
  SensorData received;

  while (1) {
    // Peek at queue without removing data
    if (xQueuePeek(dataQueue, &received, portMAX_DELAY)) {
      // Check for high-g event (launch/impact detection)
      if (received.ax > 7.0) {
        Serial.println("⚠️  High Acceleration Detected on X-axis!");
      }
      
      // Check every 200ms
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // All functionality handled by FreeRTOS tasks
}
```

---

## Detailed Code Breakdown

### 1. Library Dependencies

```cpp
#include <Adafruit_MPU6050.h>  // MPU6050 sensor driver
#include <Wire.h>               // I2C communication protocol
#include <SD.h>                 // SD card file system operations
#include <SPI.h>                // SPI protocol for SD card module
```

**Installation Instructions:**

**Arduino IDE:**
1. Go to **Sketch** → **Include Library** → **Manage Libraries**
2. Search for "Adafruit MPU6050"
3. Install "Adafruit MPU6050" by Adafruit

**PlatformIO:**
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    adafruit/Adafruit MPU6050@^2.2.4
```

---

### 2. Global Objects

```cpp
Adafruit_MPU6050 mpu;
QueueHandle_t dataQueue;
```

| Object | Type | Purpose |
|--------|------|---------|
| `mpu` | Adafruit_MPU6050 | Interface to the IMU sensor |
| `dataQueue` | QueueHandle_t | FreeRTOS queue for inter-task communication |

---

### 3. Setup Function

The setup function initializes all hardware and creates the FreeRTOS tasks.

#### Serial Initialization
```cpp
Serial.begin(115200);
```
Opens debug serial port at **115200 baud** for monitoring.

#### MPU6050 Configuration

```cpp
if (!mpu.begin()) {
  Serial.println("❌ MPU6050 not found!");
  while (1) delay(10);
}
```

**Error Handling:** If sensor not found, enters infinite loop with error message.

**Sensor Range Configuration:**

```cpp
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
```

| Parameter | Setting | Explanation |
|-----------|---------|-------------|
| **Accelerometer Range** | ±8g | Measurement range: -78.4 to +78.4 m/s² |
| **Gyroscope Range** | ±500°/s | Suitable for moderate rotation rates |
| **Filter Bandwidth** | 21 Hz | Low-pass filter to reduce sensor noise |

**Why ±8g?** Most model rockets experience 5-10g during launch, so ±8g provides adequate range without sacrificing resolution.

#### SD Card Initialization

```cpp
if (!SD.begin()) {
  Serial.println("❌ Card Mount Failed");
  return;
}

if (SD.cardType() == CARD_NONE) {
  Serial.println("❌ No SD card attached");
  return;
}
```

**Validation Steps:**
1. Attempts to mount the SD card
2. Verifies a card is actually inserted
3. Returns from setup if either check fails

**Creating the Data File:**

```cpp
File file = SD.open("/data.txt", FILE_WRITE);
if (file) {
  file.println("ax,ay,az,gx,gy,gz,temp");
  file.close();
  Serial.println("✅ SD card ready");
}
```

Creates `/data.txt` with CSV header row:
```
ax,ay,az,gx,gy,gz,temp
```

#### FreeRTOS Task Creation

```cpp
dataQueue = xQueueCreate(10, sizeof(SensorData));
```

**Queue Parameters:**
- **Capacity:** 10 items
- **Item Size:** 28 bytes (7 floats)
- **Total Memory:** ~280 bytes

**Task Creation Table:**

| Task | Function | Stack | Priority | Core | Notes |
|------|----------|-------|----------|------|-------|
| Sensor | `sensorTask` | 2048 bytes | 2 | 0 | Data producer |
| Log | `logTask` | 4096 bytes | 1 | 1 | Data consumer (SD writes need more stack) |
| Alert | `alertTask` | 2048 bytes | 3 | 1 | Highest priority for immediate detection |

```cpp
xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 2, NULL, 0);
xTaskCreatePinnedToCore(logTask, "Log Task", 4096, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(alertTask, "Alert Task", 2048, NULL, 3, NULL, 1);
```

**Function Signature Breakdown:**
```cpp
xTaskCreatePinnedToCore(
  TaskFunction_t pvTaskCode,    // Function to run
  const char * pcName,          // Task name (for debugging)
  uint32_t usStackDepth,        // Stack size in bytes
  void * pvParameters,          // Parameters passed to task
  UBaseType_t uxPriority,       // Priority (0-24, higher = more important)
  TaskHandle_t * pvCreatedTask, // Handle to created task (NULL if not needed)
  BaseType_t xCoreID            // Which core to pin to (0 or 1)
);
```

---

### 4. Task 1: Sensor Data Collection

```cpp
void sensorTask(void *pvParameters) {
  SensorData data;

  while (1) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Package sensor data
    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    data.temp = temp.temperature;

    // Send to queue
    xQueueSend(dataQueue, &data, portMAX_DELAY);
    
    // 200ms delay = 5 Hz sampling rate
    delay(200);
  }
}
```

#### Operation Flow

```
┌─────────────────────┐
│ Read MPU6050 sensor │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Extract 7 values    │
│ (ax,ay,az,gx,gy,gz,T)│
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Send to queue       │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Wait 200ms          │
└──────────┬──────────┘
           │
           └──> Repeat
```

#### Key Functions

**`mpu.getEvent(&a, &g, &temp)`**
- Reads all sensor values in a single operation
- More efficient than reading individually
- Returns acceleration, gyroscope, and temperature

**`xQueueSend(dataQueue, &data, portMAX_DELAY)`**

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Queue | `dataQueue` | Target queue |
| Data | `&data` | Pointer to data structure |
| Timeout | `portMAX_DELAY` | Wait forever if queue is full |

**Sampling Rate:** 5 Hz (200ms delay between readings)

---

### 5. Task 2: SD Card Logging

```cpp
void logTask(void *pvParameters) {
  SensorData received;

  while (1) {
    // Wait for data from queue
    if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
      File file = SD.open("/data.txt", FILE_APPEND);
      if (file) {
        // Write CSV row
        file.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    received.ax, received.ay, received.az,
                    received.gx, received.gy, received.gz,
                    received.temp);
        file.close();
        Serial.println("💾 Logged");
      } else {
        Serial.println("❌ Failed to open file for appending");
      }
    }
  }
}
```

#### Operation Flow

```
┌─────────────────────┐
│ Wait for queue data │ ◄─── Blocking call
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Open file (append)  │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Write CSV line      │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Close file          │ ◄─── Ensures data is saved
└──────────┬──────────┘
           │
           └──> Repeat
```

#### File Operations

**Opening in Append Mode:**
```cpp
File file = SD.open("/data.txt", FILE_APPEND);
```
- Opens existing file
- Positions write pointer at end
- Creates file if it doesn't exist

**Writing CSV Data:**
```cpp
file.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", ...);
```

**Format Specifier `%.2f`:**
- Floating-point number
- 2 decimal places precision
- Example: `9.81` instead of `9.8100004`

**Sample Output:**
```csv
ax,ay,az,gx,gy,gz,temp
9.81,0.12,-0.05,0.01,0.02,0.00,25.30
10.45,0.15,-0.03,0.02,0.01,-0.01,25.32
```

#### Why Close the File Every Time?

```cpp
file.close();
```

**Advantages:**
- ✅ **Data integrity:** Ensures data is flushed to SD card
- ✅ **Crash recovery:** Data saved even if power loss occurs
- ✅ **Prevents corruption:** Reduces risk of file system errors

**Disadvantages:**
- ❌ **Performance overhead:** Opening/closing is slower than keeping file open
- ❌ **SD wear:** More write cycles to file allocation table

**Best for:** Applications where data integrity is more important than speed (like flight logging).

---

### 6. Task 3: High Acceleration Alert

```cpp
void alertTask(void *pvParameters) {
  SensorData received;

  while (1) {
    // Peek at queue without removing data
    if (xQueuePeek(dataQueue, &received, portMAX_DELAY)) {
      // Check for high-g event (launch/impact detection)
      if (received.ax > 7.0) {
        Serial.println("⚠️  High Acceleration Detected on X-axis!");
      }
      
      // Check every 200ms
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}
```

#### Operation Flow

```
┌─────────────────────┐
│ Peek at queue       │ ◄─── Non-destructive read
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│ Check if ax > 7g    │
└──────────┬──────────┘
           │
     ┌─────┴─────┐
     │           │
   YES          NO
     │           │
┌────▼────┐      │
│ Trigger │      │
│ Alert   │      │
└────┬────┘      │
     │           │
     └─────┬─────┘
           │
┌──────────▼──────────┐
│ Wait 200ms          │
└──────────┬──────────┘
           │
           └──> Repeat
```

#### Queue Peek vs Receive

**Key Difference:**

| Function | Behavior | Use Case |
|----------|----------|----------|
| `xQueueReceive()` | **Removes** item from queue | Log task (consumes data) |
| `xQueuePeek()` | **Keeps** item in queue | Alert task (monitors only) |

**Why Peek?**
- Alert task only needs to **monitor** data, not consume it
- Allows log task to still receive and save the same data
- Multiple tasks can peek at the same data simultaneously

#### Detection Threshold

```cpp
if (received.ax > 7.0) {
  Serial.println("⚠️  High Acceleration Detected on X-axis!");
}
```

**Threshold:** 7g = 68.67 m/s²

**Typical Acceleration Events:**

| Event | Acceleration | Would Trigger? |
|-------|--------------|----------------|
| Gravity | 1g (9.81 m/s²) | ❌ No |
| Car braking | 0.5-1g | ❌ No |
| Model rocket launch | 5-15g | ✅ **Yes** |
| Impact with ground | 10-50g | ✅ **Yes** |
| High-power rocket | 10-30g | ✅ **Yes** |

#### Task Delay

```cpp
vTaskDelay(200 / portTICK_PERIOD_MS);
```

**Why 200ms?**
- Matches sensor sampling rate (5 Hz)
- Prevents checking the same data multiple times
- Reduces CPU usage

**Alternative (Less Efficient):**
```cpp
delay(200);  // Works but not recommended in FreeRTOS tasks
```

**Best Practice:** Use `vTaskDelay()` in FreeRTOS tasks to yield CPU to other tasks.

---

### 7. Main Loop

```cpp
void loop() {
  // All functionality handled by FreeRTOS tasks
}
```

**Why is it empty?**

In traditional Arduino programs, `loop()` runs continuously. However, with FreeRTOS:
- Tasks run independently in parallel
- FreeRTOS scheduler manages task execution
- `loop()` effectively becomes unnecessary

**What Actually Happens:**
- Arduino's main loop still runs
- But does nothing (empty function)
- FreeRTOS tasks handle all operations
- Scheduler decides which task runs when based on priority

---

## System Performance Metrics

### Timing Diagram

```
Time (ms)  0    200   400   600   800   1000
           |     |     |     |     |     |
Sensor:    ●─────●─────●─────●─────●─────●     (5 Hz)
           │     │     │     │     │     │
Log:       └─●───└─●───└─●───└─●───└─●───     (Triggered by queue)
           │     │     │     │     │     │
Alert:     ●─────●─────●─────●─────●─────●     (5 Hz)
```

### Data Throughput

| Metric | Value |
|--------|-------|
| **Sampling Rate** | 5 Hz (5 samples/second) |
| **Bytes per Sample** | ~50 bytes (CSV formatted) |
| **Data Rate** | ~250 bytes/second |
| **Hourly Data** | ~900 KB/hour |
| **Daily Data** | ~21 MB/day |

### Memory Usage

| Component | Size |
|-----------|------|
| Sensor Task Stack | 2048 bytes |
| Log Task Stack | 4096 bytes |
| Alert Task Stack | 2048 bytes |
| Data Queue | ~280 bytes |
| **Total RTOS Overhead** | **~8.4 KB** |

---

## SD Card File Output

### Sample Data File

```csv
ax,ay,az,gx,gy,gz,temp
0.15,0.08,9.78,0.01,0.00,-0.01,24.50
0.12,0.10,9.81,0.00,0.01,0.00,24.52
0.14,0.09,9.79,0.01,0.00,0.01,24.51
10.25,0.45,8.15,2.15,0.85,1.25,25.10  ← Launch detected!
15.80,1.20,6.45,3.50,1.95,2.10,26.45
```

### Data Analysis Tips

**Excel/Google Sheets:**
1. Import as CSV
2. Create charts for acceleration over time
3. Calculate max acceleration: `=MAX(A:A)`

**Python (Pandas):**
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('data.txt')
plt.plot(df['ax'], label='X-axis')
plt.xlabel('Sample')
plt.ylabel('Acceleration (m/s²)')
plt.legend()
plt.show()
```

---

## Task Priority Strategy

### Why These Priorities?

```
Priority 3 (Highest) → Alert Task
Priority 2 (Medium)  → Sensor Task  
Priority 1 (Low)     → Log Task
```

**Rationale:**

1. **Alert Task (Priority 3)**
   - Detects critical events (launch/impact)
   - Must respond immediately
   - Missing an event is unacceptable

2. **Sensor Task (Priority 2)**
   - Consistent sampling is important
   - Produces data for other tasks
   - Timing-sensitive

3. **Log Task (Priority 1)**
   - Can tolerate slight delays
   - SD writes are slow anyway
   - Queue buffers data during delays

### What Happens During High Load?

```
Scenario: All tasks want to run

1. Alert Task runs first (highest priority)
2. Sensor Task runs second
3. Log Task runs when higher-priority tasks are waiting

Result: Critical detection never delayed, 
        logging might occasionally lag but queue buffers it
```

---

## Common Issues & Solutions

### Issue 1: MPU6050 Not Found

**Error Message:**
```
❌ MPU6050 not found!
```

**Solutions:**

| Check | Solution |
|-------|----------|
| **Wiring** | Verify SDA → GPIO21, SCL → GPIO22 |
| **Power** | Ensure 3.3V and GND connected |
| **I2C Address** | Try `mpu.begin(0x69)` if default fails |
| **Pull-up Resistors** | Add 4.7kΩ resistors on SDA/SCL if needed |

### Issue 2: Card Mount Failed

**Error Message:**
```
❌ Card Mount Failed
```

**Solutions:**

1. **Check SD Card Format**
   - Must be FAT32
   - Reformat if necessary

2. **Verify Wiring**
   ```
   SD Card    ESP32
   MISO   →   GPIO19
   MOSI   →   GPIO23
   SCK    →   GPIO18
   CS     →   GPIO5 (default)
   ```

3. **Try Different Card**
   - Use Class 10 or higher
   - Some cards are incompatible

### Issue 3: Queue Overflow

**Symptom:** Data loss, missed samples

**Diagnosis:**
```cpp
UBaseType_t waiting = uxQueueMessagesWaiting(dataQueue);
Serial.printf("Queue depth: %d/10\n", waiting);
```

**Solutions:**

1. **Increase Queue Size**
   ```cpp
   dataQueue = xQueueCreate(20, sizeof(SensorData));  // Was 10
   ```

2. **Speed Up Log Task**
   - Keep file open longer
   - Batch multiple writes

3. **Reduce Sampling Rate**
   ```cpp
   delay(500);  // 2 Hz instead of 5 Hz
   ```

### Issue 4: Data Not Saving on Power Loss

**Problem:** File is empty after unexpected shutdown

**Solution:** Current code already handles this well by closing the file after each write. If still experiencing issues:

```cpp
file.flush();  // Force write before close
file.close();
```

---

## Optimization Opportunities

### For Longer Battery Life

```cpp
// Reduce sampling rate
delay(1000);  // 1 Hz instead of 5 Hz

// Use deep sleep between samples (advanced)
esp_sleep_enable_timer_wakeup(1000000);  // 1 second
esp_deep_sleep_start();
```

### For Faster Logging

Keep file open and use buffering:

```cpp
File file;  // Global variable

void setup() {
  // ...
  file = SD.open("/data.txt", FILE_APPEND);
}

void logTask(void *pvParameters) {
  SensorData received;
  int writeCount = 0;

  while (1) {
    if (xQueueReceive(dataQueue, &received, portMAX_DELAY)) {
      file.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", ...);
      
      writeCount++;
      if (writeCount >= 10) {
        file.flush();  // Flush every 10 samples
        writeCount = 0;
      }
    }
  }
}
```

### For More Channels

Add additional sensors:

```cpp
typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  float temp;
  float pressure;     // BMP280
  float altitude;     // Calculated
  float gps_lat;      // GPS
  float gps_lon;
} EnhancedSensorData;
```

---

## Comparison with Transmitter Version

### Key Differences

| Feature | Simple Logger | Transmitter Version |
|---------|---------------|---------------------|
| **XBee Module** | ❌ Not included | ✅ Real-time telemetry |
| **Tasks** | 3 tasks | 4 tasks |
| **Remote Monitoring** | ❌ No | ✅ Ground station receives data |
| **Complexity** | Simpler | More complex |
| **Power Usage** | Lower | Higher (XBee draws ~45mA) |

### When to Use Each

**Use Simple Logger when:**
- You want maximum simplicity
- Battery life is critical
- Post-flight data analysis is sufficient
- Learning FreeRTOS basics

**Use Transmitter version when:**
- Real-time monitoring needed
- Multiple rockets tracked simultaneously
- Live data visualization required
- Safety monitoring during flight

---

## Testing Checklist

Before flight, verify:

- [ ] SD card inserted and formatted (FAT32)
- [ ] Serial monitor shows "✅ SD card ready"
- [ ] Manually trigger high-g by shaking along X-axis
- [ ] Verify alert message appears
- [ ] Check SD card has CSV file with data
- [ ] Power cycle and verify file persists
- [ ] Check battery voltage (minimum 3.7V)
- [ ] Secure all connections with hot glue or tape

---