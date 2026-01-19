# MPU6050 Basic RTOS Test - Complete Documentation

## Overview

This sketch demonstrates **FreeRTOS multitasking** on the ESP32 microcontroller using the MPU6050 6-axis IMU (Inertial Measurement Unit) sensor. It showcases how to run multiple concurrent tasks with different priorities for real-time sensor monitoring and event detection.

### Key Features
- ✅ Concurrent sensor reading and alert monitoring
- ✅ High acceleration detection (>7 m/s²) for launch/impact events
- ✅ Priority-based task scheduling
- ✅ Real-time data streaming to Serial Monitor

---

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **ESP32 Dev Board** | Any ESP32 development board with I2C support |
| **MPU6050 Sensor** | 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) |
| **Jumper Wires** | For I2C connections |

---

## Pin Connections

| MPU6050 Pin | ESP32 Pin | Description |
|-------------|-----------|-------------|
| **VCC** | **3.3V** | Power supply (3.3V recommended) |
| **GND** | **GND** | Ground |
| **SDA** | **GPIO 21** | I2C Data line (default) |
| **SCL** | **GPIO 22** | I2C Clock line (default) |

### Wiring Diagram
```
MPU6050          ESP32
┌─────┐         ┌─────┐
│ VCC ├─────────┤ 3.3V│
│ GND ├─────────┤ GND │
│ SDA ├─────────┤ G21 │
│ SCL ├─────────┤ G22 │
└─────┘         └─────┘
```

---

## Required Libraries

Install these libraries via Arduino IDE Library Manager:
- **Adafruit MPU6050** (by Adafruit)
- **Adafruit Unified Sensor** (dependency)
- **Wire** (built-in I2C library)

---

## Code Breakdown

### 1. Library Includes

```cpp
#include <Adafruit_MPU6050.h>
#include <Wire.h>
```

**Explanation:**
- `Adafruit_MPU6050.h` - Provides easy-to-use functions for reading MPU6050 sensor data
- `Wire.h` - Arduino's I2C communication library for connecting to the sensor

---

### 2. Global Objects and Variables

```cpp
// ============================================================================
// GLOBAL OBJECTS AND VARIABLES
// ============================================================================

Adafruit_MPU6050 mpu;

// Shared variable accessed by multiple tasks
// Note: In production, use mutex/semaphore for thread safety
volatile float accX = 0;
```

**Explanation:**

| Variable | Type | Purpose |
|----------|------|---------|
| `mpu` | `Adafruit_MPU6050` | Sensor object for communication |
| `accX` | `volatile float` | Shared X-axis acceleration value |

**Why `volatile`?**
- The `volatile` keyword tells the compiler that this variable can change unexpectedly (modified by one task, read by another)
- Prevents compiler optimization that might cache the value
- ⚠️ **Production Note:** For robust multi-threaded access, use FreeRTOS mutexes or semaphores

---

### 3. Task 1: Sensor Reading

```cpp
// ============================================================================
// TASK 1: MPU6050 SENSOR READING
// ============================================================================
// Continuously reads accelerometer, gyroscope, and temperature data
// Priority: 1 (Lower) - Runs at 2 Hz (500ms interval)

void readMPU6050Task(void *parameter) {
  sensors_event_t acc, gyro, temp;

  while (1) {
    // Get sensor readings
    mpu.getEvent(&acc, &gyro, &temp);

    // Update shared acceleration value
    accX = acc.acceleration.x;

    // Print sensor data to Serial monitor
    Serial.print("Accel: ");
    Serial.printf("%.2f, %.2f, %.2f m/s² | ", 
                  acc.acceleration.x, 
                  acc.acceleration.y, 
                  acc.acceleration.z);

    Serial.print("Gyro: ");
    Serial.printf("%.2f, %.2f, %.2f rad/s | ", 
                  gyro.gyro.x, 
                  gyro.gyro.y, 
                  gyro.gyro.z);

    Serial.printf("Temp: %.2f °C\n", temp.temperature);

    // Delay for 500ms (2 Hz update rate)
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
```

**Task Properties:**

| Property | Value | Reason |
|----------|-------|--------|
| **Priority** | 1 (Lower) | Background data collection |
| **Update Rate** | 2 Hz (500ms) | Sufficient for most flight data |
| **Stack Size** | 4096 bytes | Handles Serial print operations |

**What This Task Does:**

1. **Reads Sensor Data**
   ```cpp
   mpu.getEvent(&acc, &gyro, &temp);
   ```
   - Gets all sensor readings in one call
   - `acc` - Acceleration data (m/s²)
   - `gyro` - Angular velocity data (rad/s)
   - `temp` - Temperature (°C)

2. **Updates Shared Variable**
   ```cpp
   accX = acc.acceleration.x;
   ```
   - Stores X-axis acceleration for the alert task

3. **Prints Formatted Data**
   ```cpp
   Serial.printf("%.2f, %.2f, %.2f m/s² | ", ...);
   ```
   - Displays all sensor values with 2 decimal precision
   - Example output:
   ```
   Accel: 0.23, -0.15, 9.81 m/s² | Gyro: 0.01, -0.02, 0.00 rad/s | Temp: 24.50 °C
   ```

4. **Yields to Other Tasks**
   ```cpp
   vTaskDelay(500 / portTICK_PERIOD_MS);
   ```
   - Suspends this task for 500ms
   - Allows FreeRTOS scheduler to run other tasks
   - `portTICK_PERIOD_MS` converts milliseconds to FreeRTOS ticks

---

### 4. Task 2: High Acceleration Detection

```cpp
// ============================================================================
// TASK 2: HIGH ACCELERATION DETECTION
// ============================================================================
// Monitors X-axis acceleration for threshold exceedance
// Priority: 2 (Higher) - Critical for flight events detection

void detectHighAccTask(void *parameter) {
  while (1) {
    // Check if acceleration exceeds threshold (launch/impact detection)
    if (accX > 7) {
      Serial.println("🚨 High acceleration detected on X-axis!");
      
      // Wait 1 second to avoid spam during sustained high-g events
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
      // Check more frequently when below threshold
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}
```

**Task Properties:**

| Property | Value | Reason |
|----------|-------|--------|
| **Priority** | 2 (Higher) | Critical event detection |
| **Check Rate (Normal)** | 10 Hz (100ms) | Fast response to events |
| **Check Rate (Alert)** | 1 Hz (1000ms) | Prevents console spam |
| **Stack Size** | 2048 bytes | Minimal processing needs |

**What This Task Does:**

1. **Monitors Acceleration Threshold**
   ```cpp
   if (accX > 7) {
   ```
   - Checks if X-axis acceleration exceeds 7 m/s² (~0.71g)
   - Useful for detecting rocket launch or landing impact

2. **Adaptive Checking Rate**
   ```cpp
   vTaskDelay(1000 / portTICK_PERIOD_MS);  // Alert mode: 1 Hz
   vTaskDelay(100 / portTICK_PERIOD_MS);   // Normal mode: 10 Hz
   ```
   - **Normal mode:** Checks every 100ms for quick detection
   - **Alert mode:** Waits 1 second after detection to prevent spam

**Why Higher Priority?**
- Can interrupt the sensor reading task
- Ensures critical events are detected quickly
- In rocketry, missing a launch or impact event could be costly

---

### 5. Setup Function

```cpp
// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    while (1) delay(10);  // Halt if sensor not detected
  }

  Serial.println("✅ MPU6050 ready");

  // Configure sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);      // ±8g range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);           // ±500°/s range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);        // Low-pass filter

  // Create sensor reading task
  // Stack: 4096 bytes, Priority: 1 (lower)
  xTaskCreate(
    readMPU6050Task,
    "MPU6050 Reader",
    4096,
    NULL,
    1,
    NULL
  );

  // Create high-acceleration detection task
  // Stack: 2048 bytes, Priority: 2 (higher - more critical)
  xTaskCreate(
    detectHighAccTask,
    "High Acc Watcher",
    2048,
    NULL,
    2,
    NULL
  );
}
```

**Step-by-Step Initialization:**

#### Step 1: Communication Setup
```cpp
Serial.begin(115200);
Wire.begin();
```
- **Serial:** 115200 baud for fast debugging output
- **Wire:** Initializes I2C bus on default pins (21=SDA, 22=SCL)

#### Step 2: Sensor Detection
```cpp
if (!mpu.begin()) {
  Serial.println("❌ MPU6050 not found!");
  while (1) delay(10);
}
```
- Attempts to communicate with MPU6050
- If failed, halts the program (infinite loop)
- Prevents running tasks with non-functional sensor

#### Step 3: Sensor Configuration

| Function | Setting | Range | Purpose |
|----------|---------|-------|---------|
| `setAccelerometerRange()` | `MPU6050_RANGE_8_G` | ±8g | Suitable for rocket launches |
| `setGyroRange()` | `MPU6050_RANGE_500_DEG` | ±500°/s | Captures fast rotations |
| `setFilterBandwidth()` | `MPU6050_BAND_21_HZ` | 21 Hz | Reduces high-frequency noise |

**Why these settings?**
- **8g range:** Rockets can experience 5-10g during launch
- **500°/s:** Captures tumbling or unstable flight
- **21 Hz filter:** Smooths vibrations while preserving real motion

#### Step 4: Task Creation

**Task 1 - Sensor Reader:**
```cpp
xTaskCreate(
  readMPU6050Task,    // Function to call
  "MPU6050 Reader",   // Task name (debugging only)
  4096,               // Stack size (bytes)
  NULL,               // Parameter to pass (none)
  1,                  // Priority (1 = lower)
  NULL                // Task handle (not needed)
);
```

**Task 2 - Alert Monitor:**
```cpp
xTaskCreate(
  detectHighAccTask,
  "High Acc Watcher",
  2048,               // Smaller stack (less work)
  NULL,
  2,                  // Priority (2 = higher)
  NULL
);
```

**Priority System:**
| Priority | Task | Why? |
|----------|------|------|
| **2** (Higher) | Alert Detection | Critical events need immediate response |
| **1** (Lower) | Sensor Reading | Background data collection can wait |

---

### 6. Main Loop

```cpp
// ============================================================================
// MAIN LOOP
// ============================================================================
// Empty - all work handled by RTOS tasks

void loop() {
  // FreeRTOS scheduler handles all tasks
}
```

**Why is `loop()` empty?**
- FreeRTOS takes over task scheduling
- Both tasks run in their own infinite loops
- No need for traditional Arduino `loop()` code
- The scheduler automatically switches between tasks based on priorities and delays

---

## How It Works - Execution Flow

```
┌─────────────────────────────────────────────────┐
│                   ESP32 Boots                   │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  setup() runs once:                             │
│  1. Initialize Serial & I2C                     │
│  2. Connect to MPU6050                          │
│  3. Configure sensor ranges                     │
│  4. Create Task 1 (Priority 1)                  │
│  5. Create Task 2 (Priority 2)                  │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│          FreeRTOS Scheduler Starts              │
└──────────────────┬──────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        ▼                     ▼
┌──────────────┐      ┌──────────────┐
│   Task 1     │      │   Task 2     │
│ (Priority 1) │      │ (Priority 2) │
├──────────────┤      ├──────────────┤
│ Read sensor  │      │ Check accX   │
│ Update accX  │      │ If > 7:      │
│ Print data   │      │   Alert!     │
│ Wait 500ms   │      │ Wait 100ms   │
└──────┬───────┘      └──────┬───────┘
       │                     │
       └──────────┬──────────┘
                  ▼
           Repeat Forever
```

---

## Sample Output

When running on the Serial Monitor (115200 baud):

```
✅ MPU6050 ready
Accel: 0.12, -0.08, 9.78 m/s² | Gyro: 0.00, 0.01, -0.01 rad/s | Temp: 23.45 °C
Accel: 0.15, -0.10, 9.81 m/s² | Gyro: 0.01, 0.00, 0.00 rad/s | Temp: 23.47 °C
Accel: 0.11, -0.09, 9.79 m/s² | Gyro: -0.01, 0.01, 0.00 rad/s | Temp: 23.46 °C
Accel: 8.45, -0.12, 9.82 m/s² | Gyro: 0.02, 0.01, -0.01 rad/s | Temp: 23.48 °C
🚨 High acceleration detected on X-axis!
Accel: 9.23, -0.15, 9.77 m/s² | Gyro: 0.01, 0.00, 0.01 rad/s | Temp: 23.50 °C
Accel: 7.89, -0.11, 9.80 m/s² | Gyro: 0.00, -0.01, 0.00 rad/s | Temp: 23.51 °C
Accel: 0.13, -0.10, 9.81 m/s² | Gyro: 0.01, 0.00, 0.00 rad/s | Temp: 23.49 °C
```

---

## Understanding the Sensor Data

### Accelerometer (m/s²)
- **X-axis:** Forward/backward acceleration
- **Y-axis:** Left/right acceleration  
- **Z-axis:** Up/down acceleration (gravity ≈ 9.8 m/s²)

### Gyroscope (rad/s)
- **X-axis:** Pitch rotation (nose up/down)
- **Y-axis:** Roll rotation (spinning)
- **Z-axis:** Yaw rotation (turning left/right)

### Temperature (°C)
- Internal sensor temperature
- Useful for thermal compensation
- Not the same as ambient temperature

---

## Troubleshooting

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| `MPU6050 not found!` | Wiring issue | Check connections, try pull-up resistors |
| No data output | Wrong baud rate | Set Serial Monitor to 115200 |
| Erratic readings | Loose connections | Secure all wires, check solder joints |
| Task crashes | Stack overflow | Increase stack size in `xTaskCreate()` |

---

## Applications in Rocketry

This code pattern is perfect for:

1. **Launch Detection** - Detect sudden acceleration spike
2. **Apogee Detection** - Monitor when acceleration approaches 0
3. **Landing Detection** - Detect impact deceleration
4. **Stability Monitoring** - Track rotation rates
5. **Flight Data Logging** - Record full flight profile

---
