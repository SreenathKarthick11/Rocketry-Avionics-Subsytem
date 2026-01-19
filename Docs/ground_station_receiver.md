# Ground Station Telemetry Receiver - Complete Documentation

## Overview

This sketch implements a **ground station receiver** for rocket telemetry using XBee wireless modules. It receives real-time flight data transmitted from the rocket and displays it on your computer's Serial Monitor for live monitoring and data recording.

### Key Features
- ✅ Continuous wireless telemetry reception via XBee
- ✅ Real-time display on Serial Monitor
- ✅ Line-buffered packet reception (complete messages)
- ✅ Simple, reliable communication protocol
- ✅ Dual UART architecture (USB + XBee)

---

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **ESP32 Dev Board** | Ground station controller |
| **XBee Module** | Series 1 or Series 2 (2.4GHz or 900MHz) |
| **XBee Explorer/Adapter** | XBee to breadboard adapter |
| **USB Cable** | For computer connection |
| **Antenna** | Attached to XBee module |

---

## Pin Connections

### XBee Module to ESP32

| XBee Pin | Signal | ESP32 Pin | Description |
|----------|--------|-----------|-------------|
| **1** | VCC | **3.3V** | Power (3.3V only!) |
| **2** | DOUT | **GPIO 16 (RX2)** | XBee transmit → ESP32 receive |
| **3** | DIN | **GPIO 17 (TX2)** | ESP32 transmit → XBee receive |
| **10** | GND | **GND** | Ground |

### Wiring Diagram

```
XBee Module          ESP32
┌──────────┐        ┌─────┐
│  1 (VCC) ├────────┤ 3.3V│
│  2 (DOUT)├────────┤ G16 │ (RX2)
│  3 (DIN) ├────────┤ G17 │ (TX2)
│ 10 (GND) ├────────┤ GND │
└──────────┘        └─────┘
```

### System Architecture

```
┌─────────────────────────────────────────────────┐
│                  GROUND STATION                 │
│                                                 │
│  ┌──────────┐      ┌──────────┐      ┌──────┐ │
│  │   USB    │◄─────┤   ESP32  │◄─────┤ XBee │ │
│  │ Computer │      │  Ground  │      │ RX   │ │
│  │          │      │ Station  │      │      │ │
│  └──────────┘      └──────────┘      └───┬──┘ │
│                                          │    │
└──────────────────────────────────────────┼────┘
                                           │
                                    ~~~~ WIRELESS ~~~~
                                           │
┌──────────────────────────────────────────┼────┐
│                     ROCKET                │    │
│                                       ┌───┴──┐ │
│  ┌──────────┐      ┌──────────┐      │ XBee │ │
│  │ Sensors  │─────►│   ESP32  │─────►│  TX  │ │
│  │ MPU6050  │      │  Flight  │      │      │ │
│  │  BMP280  │      │ Computer │      └──────┘ │
│  └──────────┘      └──────────┘               │
└─────────────────────────────────────────────────┘
```

**⚠️ Critical Warning:**
- XBee modules are **3.3V devices** - connecting to 5V will permanently damage them
- Always use 3.3V power supply
- ESP32 GPIO pins are 3.3V compatible (safe)

---

## Required Libraries

All libraries are built-in to Arduino ESP32 core:
- **HardwareSerial** - Built-in UART communication

**No external libraries needed!**

---

## Code Breakdown

### 1. UART Configuration

```cpp
// ============================================================================
// UART CONFIGURATION
// ============================================================================

HardwareSerial XBeeSerial(2);  // Use UART2 for XBee
```

**Explanation:**

| Concept | Detail |
|---------|--------|
| `HardwareSerial` | ESP32's hardware UART (not software serial) |
| `(2)` | UART2 - one of three available UARTs |
| Why UART2? | UART0 is used for USB programming |

**ESP32 UART Overview:**

| UART | Default Pins | Typical Use |
|------|--------------|-------------|
| **UART0** | RX=3, TX=1 | USB programming/Serial Monitor |
| **UART1** | RX=9, TX=10 | Usually reserved for flash memory |
| **UART2** | RX=16, TX=17 | Free for custom use (XBee) |

**Why Hardware Serial?**
- ✅ **Reliable** - Hardware-based, no timing issues
- ✅ **Fast** - Can handle high baud rates
- ✅ **Non-blocking** - Runs independently of main code
- ✅ **Buffered** - Built-in 128-byte receive buffer

---

### 2. Setup Function

```cpp
// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Serial monitor (USB connection to computer)
  Serial.begin(115200);
  
  // Initialize XBee serial (RX=GPIO16, TX=GPIO17)
  XBeeSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("✅ XBee Receiver Ready");
  Serial.println("Waiting for telemetry...");
}
```

#### Step 1: Initialize USB Serial

```cpp
Serial.begin(115200);
```

**Parameters:**

| Parameter | Value | Purpose |
|-----------|-------|---------|
| Baud Rate | 115200 | Fast USB communication for real-time data |

**Why 115200?**
- Fast enough for real-time telemetry display
- Standard baud rate supported by all Serial Monitors
- Much faster than XBee's 9600 (no bottleneck)

---

#### Step 2: Initialize XBee Serial

```cpp
XBeeSerial.begin(9600, SERIAL_8N1, 16, 17);
```

**Parameters Breakdown:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Baud Rate** | `9600` | Communication speed with XBee |
| **Config** | `SERIAL_8N1` | 8 data bits, No parity, 1 stop bit |
| **RX Pin** | `16` | GPIO 16 receives data from XBee |
| **TX Pin** | `17` | GPIO 17 sends data to XBee |

**SERIAL_8N1 Format:**

```
SERIAL_8N1 = 8 data bits + No parity + 1 stop bit

Bit Stream Example:
┌────┬───┬───┬───┬───┬───┬───┬───┬───┬────┐
│Start│ 0 │ 1 │ 0 │ 1 │ 1 │ 0 │ 0 │ 1 │Stop│
└────┴───┴───┴───┴───┴───┴───┴───┴───┴────┘
        └─────────8 data bits─────────┘
```

**Available Configurations:**

| Config | Data Bits | Parity | Stop Bits |
|--------|-----------|--------|-----------|
| `SERIAL_8N1` | 8 | None | 1 |
| `SERIAL_8N2` | 8 | None | 2 |
| `SERIAL_8E1` | 8 | Even | 1 |
| `SERIAL_8O1` | 8 | Odd | 1 |

**Why 9600 baud?**
- Default XBee baud rate (can be changed via configuration)
- Reliable for wireless transmission
- Sufficient for telemetry data (not video/audio)
- Better range than higher speeds

---

#### Step 3: Print Ready Message

```cpp
Serial.println("✅ XBee Receiver Ready");
Serial.println("Waiting for telemetry...");
```

**Output to Serial Monitor:**
```
✅ XBee Receiver Ready
Waiting for telemetry...
```

This confirms:
- ✅ Ground station powered on
- ✅ Both serial ports initialized
- ✅ Ready to receive data

---

### 3. Main Loop - Receive and Display

```cpp
// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check if data is available from XBee
  if (XBeeSerial.available()) {
    // Read complete line (until newline character)
    String received = XBeeSerial.readStringUntil('\n');
    
    // Display on Serial monitor
    Serial.print("📡 Received: ");
    Serial.println(received);
  }
}
```

#### Step 1: Check for Incoming Data

```cpp
if (XBeeSerial.available()) {
```

**What `available()` Does:**
- Returns number of bytes in receive buffer
- Returns `0` if no data available
- Non-blocking - doesn't wait for data

**How It Works:**

```
XBee UART Receive Buffer (128 bytes)
┌─────────────────────────────────────┐
│ A X : 1 2 . 3 4 \n A Y : 5 . 6 7 \n │
└─────────────────────────────────────┘
         ↑
    available() = 28 bytes
```

---

#### Step 2: Read Complete Line

```cpp
String received = XBeeSerial.readStringUntil('\n');
```

**What This Does:**

| Function | Behavior |
|----------|----------|
| `readStringUntil('\n')` | Reads until newline character (`\n`) |
| Returns | Complete message as `String` |
| Blocking | Waits up to 1 second for `\n` (timeout) |

**Example Data Flow:**

**Rocket Transmits:**
```cpp
XBeeSerial.println("AX:12.34 AY:5.67 AZ:9.81");
```

**Ground Station Receives:**
```
Buffer: "AX:12.34 AY:5.67 AZ:9.81\n"
                                  ↑
                            Stops here
                            
received = "AX:12.34 AY:5.67 AZ:9.81"  (without \n)
```

**Why `readStringUntil()` instead of `readString()`?**

| Method | Behavior | Problem |
|--------|----------|---------|
| `readString()` | Reads until timeout (1s) | Wastes time waiting |
| `readStringUntil('\n')` | Reads until newline | Immediate, message-based |

---

#### Step 3: Display Received Data

```cpp
Serial.print("📡 Received: ");
Serial.println(received);
```

**Output Format:**
```
📡 Received: AX:12.34 AY:5.67 AZ:9.81
📡 Received: Time:5234 Alt:125.5 T:24.3
📡 Received: AX:15.67 AY:2.34 AZ:9.78
```

**Why the Emoji?**
- 📡 Visually indicates wireless reception
- Easy to spot telemetry vs system messages
- Makes monitoring more intuitive

---

## How It Works - Execution Flow

```
┌─────────────────────────────────────┐
│      ESP32 Ground Station Boots     │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│ setup() runs once:                  │
│ 1. Initialize USB Serial (115200)   │
│ 2. Initialize XBee Serial (9600)    │
│ 3. Print "Ready" message            │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│          Enter loop()               │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  Check: Is data available?          │
│  XBeeSerial.available() > 0?        │
└──────┬──────────────────┬───────────┘
       │ NO               │ YES
       │                  ▼
       │         ┌─────────────────────┐
       │         │ Read until '\n'     │
       │         │ readStringUntil()   │
       │         └─────────┬───────────┘
       │                   │
       │                   ▼
       │         ┌─────────────────────┐
       │         │ Print to Serial     │
       │         │ Monitor via USB     │
       │         └─────────┬───────────┘
       │                   │
       └───────────────────┘
               │
               ▼
         (Repeat forever)
```

---

## Understanding UART Communication

### Data Format

**Transmitted Packet Structure:**
```
Start Bit │ Data Bits (8) │ Stop Bit
─────┬────┼───────────────┼────┬─────
     0    │ 0 1 0 1 1 0 0 1│    1
```

### Baud Rate

**What is 9600 baud?**
- Transmits 9600 bits per second
- At 10 bits per byte (8 data + 1 start + 1 stop):
  - **960 bytes/second** max throughput
  - **~960 characters/second**

**Telemetry Rate Calculation:**

```
Example telemetry packet:
"Time:12345 AX:12.34 AY:5.67 AZ:9.81\n"
= 38 characters = 38 bytes

Max packets per second = 960 / 38 = 25 packets/second
```

### Buffer Management

**ESP32 UART Buffer:**
```
┌────────────────────────────────────┐
│     Receive Buffer (128 bytes)     │
├────────────────────────────────────┤
│  New data writes here →  [______] │
│  Read removes from here ← [______] │
└────────────────────────────────────┘
```

**What happens if buffer overflows?**
- ⚠️ Oldest data is overwritten (data loss!)
- Solution: Read data faster than it arrives
- Monitor buffer usage: `XBeeSerial.available()`

---

## Sample Output

### Typical Ground Station Serial Monitor Display

```
✅ XBee Receiver Ready
Waiting for telemetry...
📡 Received: AX:0.12 AY:-0.08 AZ:9.78 GX:0.00 GY:0.01 GZ:-0.01 T:23.45
📡 Received: AX:0.15 AY:-0.10 AZ:9.81 GX:0.01 GY:0.00 GZ:0.00 T:23.47
📡 Received: AX:0.11 AY:-0.09 AZ:9.79 GX:-0.01 GY:0.01 GZ:0.00 T:23.46
📡 Received: Time:5234 AX:8.45 AY:-0.12 AZ:9.82 Alt:125.5 T:23.48
📡 Received: Time:5434 AX:12.23 AY:-0.15 AZ:9.77 Alt:156.2 T:23.50
📡 Received: Time:5634 AX:15.89 AY:-0.11 AZ:9.80 Alt:189.7 T:23.51
```

---

## XBee Configuration

### XBee Default Settings

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| **Baud Rate** | 9600 | Serial communication speed |
| **PAN ID** | 0x3332 | Network ID (must match) |
| **Channel** | 0x0C | Frequency channel |
| **Destination** | 0xFFFF | Broadcast to all |

### Configuring XBee Modules

**Using XCTU Software (Recommended):**

1. Connect XBee to USB explorer
2. Open XCTU (free from Digi)
3. Discover module
4. Configure parameters:
   - **BD (Baud Rate):** `3` (9600)
   - **ID (PAN ID):** `1234` (same for TX and RX)
   - **DH (Destination High):** `0`
   - **DL (Destination Low):** `FFFF` (broadcast)

**⚠️ Critical: Both XBee modules must have:**
- ✅ Same PAN ID
- ✅ Same channel
- ✅ Same baud rate
- ✅ Compatible firmware

---

## Troubleshooting

### Issue 1: No Data Received

**Symptoms:**
```
✅ XBee Receiver Ready
Waiting for telemetry...
(nothing appears)
```

**Solutions:**

| Check | Test | Fix |
|-------|------|-----|
| **XBee Power** | LED on XBee lit? | Check 3.3V connection |
| **PAN ID Match** | XCTU config | Ensure TX and RX have same ID |
| **Baud Rate** | `begin(9600)` | Verify both code and XBee config |
| **Antenna** | Physically attached | Screw on antenna |
| **Range** | Move modules closer | Start at <10 feet for testing |
| **Transmitter** | Rocket code running? | Verify transmitter is sending |

---

### Issue 2: Garbled/Corrupted Data

**Symptoms:**
```
📡 Received: AX:��.�� AY:��
📡 Received: ���������
```

**Causes & Solutions:**

| Cause | Solution |
|-------|----------|
| Baud rate mismatch | Verify both use 9600 |
| Weak signal | Reduce distance, check antenna |
| Electrical interference | Move away from motors/WiFi |
| Wrong UART config | Use `SERIAL_8N1` |

---

### Issue 3: Incomplete Messages

**Symptoms:**
```
📡 Received: AX:12.34 AY
📡 Received: :5.67 AZ:9.81
```

**Causes:**
- Data arriving faster than read speed
- Buffer overflow

**Solutions:**
```cpp
void loop() {
  // Read all available data immediately
  while (XBeeSerial.available()) {
    String received = XBeeSerial.readStringUntil('\n');
    Serial.print("📡 Received: ");
    Serial.println(received);
  }
}
```

---

### Issue 4: Random Characters

**Symptoms:**
```
📡 Received: AX:12.34 þÿ AY:5.67
```

**Cause:** Noise on transmission

**Solutions:**
- Add error checking (checksum)
- Reduce transmission power interference
- Improve antenna placement

---

## Communication Protocol Design

### Simple Protocol (Current)

```
Format: "KEY:VALUE KEY:VALUE ...\n"
Example: "AX:12.34 AY:5.67 AZ:9.81\n"
```

**Advantages:**
- ✅ Human-readable
- ✅ Easy to debug
- ✅ Flexible (add fields easily)

**Disadvantages:**
- ❌ No error detection
- ❌ Verbose (more bytes)
- ❌ No packet validation

---

### Enhanced Protocol with Checksum

```cpp
// TRANSMITTER (Rocket):
void sendTelemetry() {
  String data = "AX:" + String(ax) + " AY:" + String(ay);
  uint8_t checksum = calculateChecksum(data);
  XBeeSerial.println(data + " CRC:" + String(checksum));
}

// RECEIVER (Ground):
void loop() {
  if (XBeeSerial.available()) {
    String received = XBeeSerial.readStringUntil('\n');
    
    if (validateChecksum(received)) {
      Serial.print("✅ Valid: ");
      Serial.println(received);
    } else {
      Serial.println("❌ Checksum failed!");
    }
  }
}
```

---

### Binary Protocol (Advanced)

```cpp
// More efficient but less readable
struct TelemetryPacket {
  uint8_t header = 0xAA;
  uint32_t timestamp;
  float ax, ay, az;
  float gx, gy, gz;
  uint8_t checksum;
} __attribute__((packed));

// Send:
XBeeSerial.write((uint8_t*)&packet, sizeof(packet));

// Receive:
XBeeSerial.readBytes((uint8_t*)&packet, sizeof(packet));
```

**Advantages:**
- ✅ Compact (fewer bytes)
- ✅ Faster transmission
- ✅ Fixed structure

**Disadvantages:**
- ❌ Not human-readable
- ❌ Harder to debug
- ❌ Endianness issues

---

## Data Logging to Computer

### Option 1: Copy/Paste from Serial Monitor

**Steps:**
1. Open Serial Monitor
2. Let data accumulate
3. Select all (Ctrl+A)
4. Copy (Ctrl+C)
5. Paste into text editor
6. Save as CSV

---

### Option 2: Serial Monitor Auto-Save

**Arduino IDE 2.0:**
1. Tools → Serial Monitor
2. Click settings gear
3. Enable "Autoscroll"
4. Use screenshot or copy periodically

---

### Option 3: Python Serial Logger (Recommended)

```python
import serial
import datetime

# Open serial port
ser = serial.Serial('COM3', 115200)  # Change COM3 to your port

# Open log file
filename = f"telemetry_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
with open(filename, 'w') as f:
    f.write("timestamp,data\n")
    
    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                timestamp = datetime.datetime.now().isoformat()
                f.write(f"{timestamp},{line}\n")
                f.flush()  # Force write
                print(line)
    except KeyboardInterrupt:
        print(f"\nSaved to {filename}")
```

**Run:**
```bash
python log_telemetry.py
```

---

## Real-Time Data Visualization

### Option: Processing Sketch

```java
// Processing code for real-time plotting
import processing.serial.*;

Serial port;
float ax, ay, az;

void setup() {
  size(800, 600);
  port = new Serial(this, "COM3", 115200);
}

void draw() {
  background(0);
  
  // Parse incoming data
  if (port.available() > 0) {
    String data = port.readStringUntil('\n');
    if (data != null) {
      parseData(data);
    }
  }
  
  // Draw accelerometer bars
  fill(255, 0, 0);
  rect(100, 500, ax * 20, 20);
  // ... more visualization
}
```

---

## Range Testing

### Typical XBee Ranges

| XBee Model | Indoor | Outdoor (Line of Sight) |
|------------|--------|-------------------------|
| **Series 1 (2.4GHz)** | 30m (100ft) | 100m (300ft) |
| **Series 2 (2.4GHz)** | 40m (120ft) | 120m (400ft) |
| **900HP (900MHz)** | 600m (2000ft) | 9000m (28,000ft) |

**Factors Affecting Range:**
- ✅ Antenna type (wire vs chip)
- ✅ Obstacles (walls, trees)
- ✅ Interference (WiFi, Bluetooth)
- ✅ Power level (configurable)
- ✅ Baud rate (lower = longer range)

---

## Power Consumption

| Component | Current Draw |
|-----------|--------------|
| **ESP32** | ~80mA (active) |
| **XBee Series 1** | ~45mA (TX), ~50mA (RX) |
| **Total** | ~130mA @ 3.3V |

**Battery Life Calculation:**
```
Battery: 2000mAh (typical USB power bank)
Current: 130mA
Runtime: 2000 / 130 = 15.4 hours
```

---

## Applications in Rocketry

This ground station is perfect for:

| Application | How It's Used |
|-------------|---------------|
| **Real-Time Monitoring** | Watch flight data as it happens |
| **Launch Confirmation** | Verify ignition detected |
| **Apogee Detection** | Know when parachute should deploy |
| **Landing Confirmation** | Detect touchdown |
| **Range Safety** | Monitor trajectory data |
| **Post-Flight Analysis** | Log all data for later review |

---

## Next Steps - Enhancements

### 1. Add RSSI Monitoring

```cpp
void loop() {
  if (XBeeSerial.available()) {
    String received = XBeeSerial.readStringUntil('\n');
    int rssi = getRSSI();  // Read signal strength
    Serial.printf("📡 [RSSI:%d] %s\n", rssi, received.c_str());
  }
}
```

### 2. Implement Data Filtering

```cpp
void loop() {
  if (XBeeSerial.available()) {
    String received = XBeeSerial.readStringUntil('\n');
    
    // Only show critical alerts
    if (received.indexOf("ALERT") >= 0) {
      Serial.println("🚨 " + received);
    } else {
      Serial.println("📡 " + received);
    }
  }
}
```

### 3. Add Timestamping

```cpp
void loop() {
  if (XBeeSerial.available()) {
    String received = XBeeSerial.readStringUntil('\n');
    Serial.printf("[%lu ms] 📡 %s\n", millis(), received.c_str());
  }
}
```

---

## Safety Considerations

### Pre-Flight Checklist

- [ ] XBee modules powered on
- [ ] Green LED on both XBee modules
- [ ] Same PAN ID on both modules
- [ ] Antennas attached
- [ ] Serial Monitor open at 115200 baud
- [ ] Test transmission from rocket (static test)
- [ ] Verify reception at launch distance
- [ ] Battery fully charged

### During Flight

- ⚠️ Do NOT disconnect Serial Monitor during flight
- ⚠️ Keep laptop plugged in (don't run on battery)
- ⚠️ Position antenna with clear line of sight
- ⚠️ Have backup recorder (phone video of screen)

---
