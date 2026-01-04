# Rocketry Avionics Subsystem

**Team Euclidean | Team Number: 2024-ASI-ROCKETRY-063**

A comprehensive avionics system designed for a model rocket project targeting a 1000-meter apogee with CANSAT payload deployment capability.

---

## Overview

This repository contains the flight software and avionics implementation for both the **main rocket** and the **CANSAT payload**. The system is built around the ESP32-WROOM microcontroller running FreeRTOS for real-time mission-critical operations.

### Key Features

- **Real-time flight state management** (Boot → Launchpad → Ascent → Payload Separation → Descent → Impact)
- **Multi-sensor data acquisition** (BMP280, MPU6050, NEO-6M GPS)
- **Dual telemetry system** (XBee wireless @ 1Hz + onboard SD/LittleFS logging)
- **Autonomous payload deployment** at apogee detection
- **FreeRTOS task-based architecture** for deterministic real-time behavior
- **Ground station integration** with live telemetry visualization

---

## System Architecture

### Hardware Components

| Component | Model | Purpose | Interface |
|-----------|-------|---------|-----------|
| **Microcontroller** | ESP32-WROOM-32 | Flight computer | - |
| **Pressure/Temp/Alt Sensor** | BMP280 | Altitude estimation | I²C (GPIO 13/14) |
| **IMU** | MPU6050 | Acceleration & gyro data | I²C (GPIO 21/22) |
| **GPS Module** | NEO-6M | Position tracking | UART (GPIO 3/1) |
| **Telemetry Radio** | XBee Pro S2C | Ground communication | UART (GPIO 16/17) |
| **Data Storage** | SD Card / LittleFS | Flight data logging | SPI / Flash |
| **Actuator** | Relay/Servo | Payload deployment | GPIO 32 |

### Pin Configuration

```
BMP280:  SDA=13, SCL=14
MPU6050: SDA=21, SCL=22
GPS:     RX=3,   TX=1
XBee:    RX=16,  TX=17
SD Card: MOSI=23, MISO=19, SCK=18, CS=5
Relay:   GPIO=32
```

---

## Software Architecture

### FreeRTOS Task Structure

The flight software operates as a set of concurrent tasks with prioritized execution:

| Task | Priority | Stack | Core | Function |
|------|----------|-------|------|----------|
| **CalibrationTask** | - | 4KB | 1 | Initial sensor baseline calibration |
| **SensorTask** | 1-2 | 8KB | 1 | Continuous sensor data acquisition |
| **XBeeTask** | 2-3 | 4KB | 1 | Telemetry transmission @ 1Hz |
| **LoggingTask** | 2-4 | 4KB | 1 | Data storage to SD/LittleFS |
| **ApogeeTask** | 2 | 4KB | 1 | Flight state machine & deployment |
| **ServoTask** | 1 | 4KB | 0 | Payload release control (CANSAT) |

### Flight State Machine

```
BOOT → CALIBRATION → LAUNCHPAD → ASCENT → PAYLOAD_SEP → DESCENT → AEROBRAKE → IMPACT
```

**State Transitions:**
- **LAUNCHPAD → ASCENT:** Altitude > 2m (rocket) / 0.1m (CANSAT)
- **ASCENT → PAYLOAD_SEP:** Drop of 2m from max altitude detected
- **PAYLOAD_SEP → DESCENT:** Altitude continues decreasing
- **DESCENT → AEROBRAKE:** Altitude < 500m
- **DESCENT → IMPACT:** Altitude < 10m (rocket) / 2m (CANSAT)

---

## Repository Structure

```
├── cansat_code.ino           # CANSAT flight software (standalone)
├── rokect_code.ino           # Main rocket flight software (production)
├── test_rtos_cansat.ino      # CANSAT test code with WiFi/MQTT
├── test_rtos_rocket.ino      # Rocket test code with WiFi/MQTT
├── CDR.pdf                   # Critical Design Review document
└── README.md                 # This file
```

---

## Installation & Setup

### Prerequisites

1. **Arduino IDE** (v2.x) or **PlatformIO**
2. **ESP32 Board Support** (`esp32` by Espressif)
3. **Required Libraries:**
   ```
   - Adafruit BMP280
   - Adafruit MPU6050
   - TinyGPSPlus
   - SD (built-in)
   - LittleFS (built-in for ESP32)
   - WiFi (for test versions)
   - PubSubClient (for MQTT in test versions)
   ```
---

## Running the Code

### For On board Flight

**Rocket:**
```cpp
// Upload rokect_code.ino to main rocket ESP32
// No WiFi/MQTT - optimized for flight operations
```

**CANSAT:**
```cpp
// Upload cansat_code.ino to CANSAT ESP32
// Includes servo-based parachute deployment
```

### For Ground Testing

**With Telemetry Monitoring:**
```cpp
// Upload test_rtos_rocket.ino or test_rtos_cansat.ino
// Configure WiFi credentials:
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_PASSWORD";

// Monitor via MQTT:
// - Broker: test.mosquitto.org:1883
// - Topic: esp32/status
```

---

## Telemetry Data Format

### CSV Packet Structure

```csv
TEAM_ID,TIME,PACKET_COUNT,REL_ALT,PRESSURE,TEMP,LAT,LON,ALT_GPS,SATS,AX,AY,AZ,GX,GY,GZ[,STATE]
```

**Example:**
```
2024-ASI-063,12:34:56,142,325.50,1013.25,22.3,28.6139,77.2090,328.2,8,0.12,0.05,9.81,0.02,-0.01,0.03,ASCENT
```

### Data Logging

- **Rocket:** SD card @ 5-second intervals
- **CANSAT:** LittleFS flash @ 1-second intervals
- **XBee Telemetry:** 1 Hz transmission rate
- **File Format:** CSV with headers

---

## Key Design Decisions

### Why ESP32 over STM32?

- **Simpler peripheral interfacing** (GPIO, I²C, SPI)
- **Extensive community support** and examples
- **Integrated FreeRTOS** with mature implementation
- **Dual-core architecture** for task parallelism
- **Better WiFi debugging** during development

### Why Neo-6M over Bharat NavIC?

- **Proven reliability** and accuracy
- **Mature software libraries** (TinyGPSPlus)
- **Simpler antenna requirements**
- **Better ESP32 compatibility**

### Why FreeRTOS Task-Based Design?

- **Deterministic real-time behavior** for critical events
- **Clear separation of concerns** (sensing, logging, telemetry, state management)
- **Resource efficiency** without framework overhead (KubOS discontinued)
- **Robust error handling** via watchdog mechanisms

### Data Storage Strategy

- **Rocket:** SD card for large capacity (full flight duration logging)
- **CANSAT:** LittleFS for reliability (no moving parts, shock-resistant)

---

## Critical Safety Features

1. **Baseline Calibration:** 20-sample altitude averaging before launch
2. **Apogee Detection:** 2-meter drop threshold prevents false triggers
3. **Watchdog Integration:** Task-level monitoring prevents lockups
4. **Data Redundancy:** Simultaneous XBee transmission + local storage
5. **State Recovery:** No dependency on software resets

---

## Performance Metrics

| Metric | Value |
|--------|-------|
| **Telemetry Rate** | 1 Hz (per competition requirements) |
| **Sensor Update Rate** | 1 Hz (altitude), 1 Hz (IMU), 1 Hz (GPS) |
| **State Machine Response** | <100ms (apogee detection to deployment) |
| **Data Logging Interval** | 1s (CANSAT), 5s (Rocket) |
| **Calibration Time** | ~4 seconds (20 samples @ 200ms intervals) |

---


## Documentation

- **CDR (Critical Design Review):** Complete system documentation in `CDR.pdf`
- **Sensor Datasheets:** See Bill of Materials in CDR Section 7.2.1
- **Testing Plans:** See CDR Chapter 5.2 & Chapter 6.1
- **State Diagram:** See CDR Figure 3.22




