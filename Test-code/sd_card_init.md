# SD Card Functionality Test - Complete Documentation

## Overview

This comprehensive test suite demonstrates **all SD card operations** needed for flight data logging systems. It validates directory management, file operations, and measures read/write performance to ensure your SD card is ready for high-speed data acquisition during rocket flights.

### Key Features
- ✅ Directory operations (create, list, remove)
- ✅ File operations (create, read, write, append, rename, delete)
- ✅ Performance benchmarking (read/write speed testing)
- ✅ Card information reporting (type, size, space usage)
- ✅ Complete diagnostic suite

---

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **ESP32 Dev Board** | Any ESP32 with SPI support |
| **SD Card Module** | SPI-based SD card reader/writer |
| **Micro SD Card** | FAT32 formatted, 4GB-32GB recommended |
| **Jumper Wires** | For SPI connections |

---

## Pin Connections

### SD Card Module Pinout

| SD Card Pin | Signal | ESP32 Pin | Description |
|-------------|--------|-----------|-------------|
| **D0** | MISO | **GPIO 19** | Master In Slave Out (data from SD) |
| **D1** | - | (unused) | Not used in SPI mode |
| **D2** | - | (unused) | Not used in SPI mode |
| **D3** | SS/CS | **GPIO 5** | Chip Select (default) |
| **CMD** | MOSI | **GPIO 23** | Master Out Slave In (data to SD) |
| **CLK** | SCK | **GPIO 18** | SPI Clock |
| **VDD** | Power | **3.3V** | Power supply |
| **VSS** | Ground | **GND** | Ground (connect twice) |

### Wiring Diagram
```
SD Card Module       ESP32
┌──────────┐        ┌─────┐
│ GND      ├────────┤ GND │
│ VCC      ├────────┤ 3.3V│
│ MISO (D0)├────────┤ G19 │
│ MOSI(CMD)├────────┤ G23 │
│ SCK (CLK)├────────┤ G18 │
│ CS   (D3)├────────┤ G5  │
└──────────┘        └─────┘
```

**⚠️ Important Notes:**
- Use **3.3V**, NOT 5V (can damage SD card)
- Some modules have built-in voltage regulators and level shifters
- Ensure good connections - SD cards are sensitive to signal integrity

---

## Required Libraries

All libraries are built-in to ESP32 Arduino core:
- **FS.h** - File System abstraction
- **SD.h** - SD card driver
- **SPI.h** - SPI communication

---

## Code Breakdown

### 1. Library Includes

```cpp
#include "FS.h"
#include "SD.h"
#include "SPI.h"
```

**Explanation:**
- `FS.h` - Provides file system abstraction layer (works with SD, SPIFFS, LittleFS)
- `SD.h` - SD card-specific implementation
- `SPI.h` - Serial Peripheral Interface for high-speed communication

---

## Directory Operations

### Function 1: List Directory Contents

```cpp
// ============================================================================
// DIRECTORY OPERATIONS
// ============================================================================

// List all files and directories recursively
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root) {
        Serial.println("❌ Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("❌ Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            
            // Recursively list subdirectories
            if (levels) {
                listDir(fs, file.path(), levels - 1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}
```

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `fs` | `fs::FS&` | Reference to file system (SD) |
| `dirname` | `const char*` | Directory path to list (e.g., "/") |
| `levels` | `uint8_t` | Recursion depth (0 = current only) |

**How It Works:**

1. **Open Directory**
   ```cpp
   File root = fs.open(dirname);
   ```
   - Opens the specified directory path
   - Returns a File object representing the directory

2. **Validate It's a Directory**
   ```cpp
   if (!root.isDirectory()) {
       Serial.println("❌ Not a directory");
       return;
   }
   ```

3. **Iterate Through Contents**
   ```cpp
   File file = root.openNextFile();
   while (file) {
       // Process each file/directory
       file = root.openNextFile();
   }
   ```

4. **Recursive Subdirectory Listing**
   ```cpp
   if (levels) {
       listDir(fs, file.path(), levels - 1);
   }
   ```
   - If `levels > 0`, recursively list subdirectories
   - Each recursion decrements the level counter

**Example Output:**
```
Listing directory: /
  FILE: data.txt  SIZE: 1024
  DIR : logs
  FILE: config.ini  SIZE: 256
```

---

### Function 2: Create Directory

```cpp
// Create a new directory
void createDir(fs::FS &fs, const char * path) {
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path)) {
        Serial.println("✅ Dir created");
    } else {
        Serial.println("❌ mkdir failed");
    }
}
```

**What It Does:**
- Creates a new directory at the specified path
- Returns success/failure status
- **Note:** Cannot create nested directories in one call (use `/test` not `/test/nested`)

**Usage Example:**
```cpp
createDir(SD, "/flight_data");  // ✅ Creates /flight_data/
createDir(SD, "/a/b/c");        // ❌ Fails if /a/ doesn't exist
```

---

### Function 3: Remove Directory

```cpp
// Remove a directory (must be empty)
void removeDir(fs::FS &fs, const char * path) {
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path)) {
        Serial.println("✅ Dir removed");
    } else {
        Serial.println("❌ rmdir failed");
    }
}
```

**Important Limitation:**
- Directory **must be empty** before removal
- Delete all files inside first, then remove directory
- Root directory `/` cannot be removed

---

## File Operations

### Function 4: Read File

```cpp
// ============================================================================
// FILE OPERATIONS
// ============================================================================

// Read and print file contents
void readFile(fs::FS &fs, const char * path) {
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file) {
        Serial.println("❌ Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available()) {
        Serial.write(file.read());
    }
    file.close();
}
```

**How It Works:**

| Step | Code | Purpose |
|------|------|---------|
| 1 | `fs.open(path)` | Open file in read mode (default) |
| 2 | `file.available()` | Check if more bytes to read |
| 3 | `file.read()` | Read one byte at a time |
| 4 | `Serial.write()` | Print byte to Serial Monitor |
| 5 | `file.close()` | Release file handle |

**Better Alternative for Large Files:**
```cpp
// Read in chunks for better performance
char buffer[64];
while (file.available()) {
    int bytesRead = file.readBytes(buffer, 64);
    Serial.write(buffer, bytesRead);
}
```

---

### Function 5: Write File

```cpp
// Write data to file (overwrites existing content)
void writeFile(fs::FS &fs, const char * path, const char * message) {
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("❌ Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("✅ File written");
    } else {
        Serial.println("❌ Write failed");
    }
    file.close();
}
```

**Key Points:**

| Aspect | Detail |
|--------|--------|
| **Mode** | `FILE_WRITE` - Opens or creates file, overwrites if exists |
| **Method** | `file.print()` - Writes string data |
| **Auto-create** | Creates file if it doesn't exist |
| **Overwrite** | ⚠️ Destroys existing content |

**Available Write Methods:**
```cpp
file.print("Hello");      // String
file.println("Hello");    // String + newline
file.write(byte);         // Single byte
file.write(buffer, len);  // Byte array
```

---

### Function 6: Append to File

```cpp
// Append data to existing file
void appendFile(fs::FS &fs, const char * path, const char * message) {
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("❌ Failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        Serial.println("✅ Message appended");
    } else {
        Serial.println("❌ Append failed");
    }
    file.close();
}
```

**FILE_WRITE vs FILE_APPEND:**

| Mode | Behavior |
|------|----------|
| `FILE_WRITE` | Overwrites from beginning |
| `FILE_APPEND` | Adds to end of file |

**Perfect for Data Logging:**
```cpp
// Flight data logging example
File log = SD.open("/flight.csv", FILE_APPEND);
log.printf("%lu,%.2f,%.2f,%.2f\n", millis(), ax, ay, az);
log.close();
```

---

### Function 7: Rename File

```cpp
// Rename a file
void renameFile(fs::FS &fs, const char * path1, const char * path2) {
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("✅ File renamed");
    } else {
        Serial.println("❌ Rename failed");
    }
}
```

**Use Cases:**
- Rename data files after flight: `/data.txt` → `/flight_001.txt`
- Move files between directories: `/temp.log` → `/archive/temp.log`
- Backup files: `/config.ini` → `/config_backup.ini`

**Limitations:**
- Both paths must be on same SD card
- Cannot rename open files
- Destination file will be overwritten if exists

---

### Function 8: Delete File

```cpp
// Delete a file
void deleteFile(fs::FS &fs, const char * path) {
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path)) {
        Serial.println("✅ File deleted");
    } else {
        Serial.println("❌ Delete failed");
    }
}
```

**Safety Tips:**
- Cannot undo - deletion is permanent
- Always close file before deleting
- Check return value before assuming success
- Consider renaming to `.old` instead of deleting

---

## Performance Testing

### Function 9: Benchmark Read/Write Speed

```cpp
// ============================================================================
// PERFORMANCE TESTING
// ============================================================================

// Benchmark read/write performance
void testFileIO(fs::FS &fs, const char * path) {
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    
    // ---- READ TEST ----
    if (file) {
        len = file.size();
        size_t flen = len;
        start = millis();
        
        while (len) {
            size_t toRead = len;
            if (toRead > 512) {
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        
        end = millis() - start;
        Serial.printf("📖 %u bytes read in %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("❌ Failed to open file for reading");
    }

    // ---- WRITE TEST ----
    file = fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("❌ Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++) {
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("💾 %u bytes written in %u ms\n", 2048 * 512, end);
    file.close();
}
```

**What Gets Tested:**

| Test | Size | Purpose |
|------|------|---------|
| **Read** | Full file | Measures sequential read speed |
| **Write** | 1 MB (2048×512 bytes) | Measures sequential write speed |

**Read Test Breakdown:**

1. **Open Existing File**
   ```cpp
   File file = fs.open(path);
   len = file.size();
   ```

2. **Read in 512-Byte Chunks**
   ```cpp
   while (len) {
       size_t toRead = len;
       if (toRead > 512) {
           toRead = 512;
       }
       file.read(buf, toRead);
       len -= toRead;
   }
   ```
   - Chunks improve performance vs byte-by-byte
   - 512 bytes matches SD card sector size

3. **Calculate Performance**
   ```cpp
   end = millis() - start;
   Serial.printf("📖 %u bytes read in %u ms\n", flen, end);
   ```

**Write Test Breakdown:**

1. **Write 1 MB of Data**
   ```cpp
   for (i = 0; i < 2048; i++) {
       file.write(buf, 512);
   }
   ```
   - 2048 iterations × 512 bytes = 1,048,576 bytes (1 MB)

2. **Measure Time**
   ```cpp
   end = millis() - start;
   Serial.printf("💾 %u bytes written in %u ms\n", 2048 * 512, end);
   ```

**Typical Performance:**

| Card Class | Read Speed | Write Speed |
|------------|------------|-------------|
| Class 4 | ~4 MB/s | ~4 MB/s |
| Class 10 | ~10 MB/s | ~10 MB/s |
| UHS-I | ~20 MB/s | ~15 MB/s |

**Example Output:**
```
📖 524288 bytes read in 120 ms     (4.37 MB/s)
💾 1048576 bytes written in 250 ms (4.19 MB/s)
```

---

## Setup Function

### Main Initialization and Test Sequence

```cpp
// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    // Initialize SD card
    if (!SD.begin()) {
        Serial.println("❌ Card Mount Failed");
        return;
    }
    
    // Check card type
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("❌ No SD card attached");
        return;
    }

    // Print card information
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    // Run test sequence
    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}
```

### Step-by-Step Breakdown

#### Step 1: Initialize SD Card

```cpp
if (!SD.begin()) {
    Serial.println("❌ Card Mount Failed");
    return;
}
```

**What `SD.begin()` Does:**
- Initializes SPI bus
- Sends initialization commands to SD card
- Reads card identification data
- Mounts FAT file system

**Common Failure Reasons:**
| Problem | Solution |
|---------|----------|
| Wrong wiring | Check pin connections |
| No card inserted | Insert formatted SD card |
| Unsupported card | Use FAT32 formatted card |
| Bad CS pin | Explicitly set: `SD.begin(5)` |

---

#### Step 2: Check Card Type

```cpp
uint8_t cardType = SD.cardType();
if (cardType == CARD_NONE) {
    Serial.println("❌ No SD card attached");
    return;
}
```

**Card Types:**

| Constant | Type | Description |
|----------|------|-------------|
| `CARD_NONE` | None | No card detected |
| `CARD_MMC` | MMC | MultiMediaCard (older) |
| `CARD_SD` | SDSC | Standard Capacity (≤2GB) |
| `CARD_SDHC` | SDHC | High Capacity (2GB-32GB) |

**Why This Matters:**
- SDSC cards use byte addressing
- SDHC cards use block addressing
- Library handles differences automatically

---

#### Step 3: Get Card Information

```cpp
uint64_t cardSize = SD.cardSize() / (1024 * 1024);
Serial.printf("SD Card Size: %lluMB\n", cardSize);
```

**Available Info Methods:**

| Method | Returns | Description |
|--------|---------|-------------|
| `SD.cardSize()` | `uint64_t` | Total card capacity (bytes) |
| `SD.totalBytes()` | `uint64_t` | Total filesystem space |
| `SD.usedBytes()` | `uint64_t` | Used space |

**Calculation:**
```cpp
cardSize / (1024 * 1024)        // Bytes → MB
cardSize / (1024 * 1024 * 1024) // Bytes → GB
```

---

#### Step 4: Run Comprehensive Test Sequence

```cpp
// Run test sequence
listDir(SD, "/", 0);                              // 1. List root
createDir(SD, "/mydir");                          // 2. Create directory
listDir(SD, "/", 0);                              // 3. Verify creation
removeDir(SD, "/mydir");                          // 4. Remove directory
listDir(SD, "/", 2);                              // 5. List recursively
writeFile(SD, "/hello.txt", "Hello ");            // 6. Create file
appendFile(SD, "/hello.txt", "World!\n");         // 7. Append data
readFile(SD, "/hello.txt");                       // 8. Read back
deleteFile(SD, "/foo.txt");                       // 9. Try delete (may not exist)
renameFile(SD, "/hello.txt", "/foo.txt");         // 10. Rename file
readFile(SD, "/foo.txt");                         // 11. Verify rename
testFileIO(SD, "/test.txt");                      // 12. Performance test
```

**Test Sequence Flow:**

```
┌─────────────────────────────────────────┐
│  1. List Root Directory                 │
│     (Show existing files)               │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  2. Create Directory "/mydir"           │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  3. List Root Again                     │
│     (Verify /mydir appears)             │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  4. Remove Directory "/mydir"           │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  5. List Recursively (2 levels)         │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  6. Write "Hello " to /hello.txt        │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  7. Append "World!\n" to /hello.txt     │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  8. Read /hello.txt                     │
│     (Should show "Hello World!")        │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│  9. Delete /foo.txt (cleanup from prev) │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│ 10. Rename /hello.txt → /foo.txt        │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│ 11. Read /foo.txt                       │
│     (Verify rename worked)              │
└──────────────┬──────────────────────────┘
               ▼
┌─────────────────────────────────────────┐
│ 12. Performance Test                    │
│     (Write 1MB, measure speed)          │
└─────────────────────────────────────────┘
```

---

#### Step 5: Report Storage Usage

```cpp
Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
```

**Calculation Example:**
```
Total space: 7580 MB  (8GB card)
Used space: 125 MB    (after test files)
Free space: 7455 MB   (calculated)
```

---

### Main Loop

```cpp
// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // All tests run once in setup()
}
```

**Why Empty?**
- This is a **diagnostic sketch** - runs once
- All tests execute in `setup()`
- For continuous logging, add code to `loop()`

---

## Sample Output

When you run this sketch, you should see output like this:

```
SD Card Type: SDHC
SD Card Size: 7580MB

Listing directory: /
  FILE: system~1  SIZE: 0
  FILE: indexe~1  SIZE: 0

Creating Dir: /mydir
✅ Dir created

Listing directory: /
  FILE: system~1  SIZE: 0
  FILE: indexe~1  SIZE: 0
  DIR : mydir

Removing Dir: /mydir
✅ Dir removed

Listing directory: /
  FILE: system~1  SIZE: 0
  FILE: indexe~1  SIZE: 0

Writing file: /hello.txt
✅ File written

Appending to file: /hello.txt
✅ Message appended

Reading file: /hello.txt
Read from file: Hello World!

Deleting file: /foo.txt
❌ Delete failed

Renaming file /hello.txt to /foo.txt
✅ File renamed

Reading file: /foo.txt
Read from file: Hello World!

📖 0 bytes read in 0 ms
💾 1048576 bytes written in 2504 ms

Total space: 7580MB
Used space: 1MB
```

---

## Understanding File System Basics

### File Paths

| Path | Description | Valid? |
|------|-------------|--------|
| `/` | Root directory | ✅ |
| `/data.txt` | File in root | ✅ |
| `/logs/` | Subdirectory | ✅ |
| `/logs/flight1.csv` | File in subdirectory | ✅ |
| `data.txt` | ❌ Missing leading slash | ❌ |
| `C:\data.txt` | ❌ Windows-style path | ❌ |

### File Naming Rules

**Allowed:**
- Letters: `A-Z`, `a-z`
- Numbers: `0-9`
- Special: `_`, `-`, `.`

**Restrictions:**
- Max 8 characters + 3 extension (8.3 format for FAT)
- No spaces (use underscore: `flight_data.txt`)
- Case-insensitive (FAT doesn't preserve case)

**Examples:**
```cpp
"/data.txt"        // ✅ Good
"/flight_001.csv"  // ✅ Good
"/log-2024.txt"    // ✅ Good
"/my data.txt"     // ❌ Space (may work but avoid)
"/verylongname.txt"// ❌ Too long for 8.3
```

---

## Common Issues and Solutions

### Issue 1: Card Mount Failed

**Symptoms:**
```
❌ Card Mount Failed
```

**Solutions:**

| Check | Fix |
|-------|-----|
| Wiring | Verify all 6 connections |
| Power | Ensure stable 3.3V supply |
| Card format | Format as FAT32 |
| CS pin | Try `SD.begin(5)` explicitly |
| Card insertion | Fully insert SD card |

---

### Issue 2: No SD Card Attached

**Symptoms:**
```
❌ No SD card attached
```

**Solutions:**
- Check if card is fully inserted
- Try different SD card
- Verify card is not locked (check switch on side)

---

### Issue 3: Write/Delete Failed

**Symptoms:**
```
❌ Write failed
❌ Delete failed
```

**Solutions:**

| Cause | Fix |
|-------|-----|
| Card write-protected | Check physical lock switch |
| Disk full | Delete old files |
| File open | Close file before deleting |
| Bad sectors | Try different SD card |

---

### Issue 4: Slow Performance

**Symptoms:**
```
💾 1048576 bytes written in 5000 ms  (Only 0.2 MB/s)
```

**Solutions:**
- Use Class 10 or higher SD card
- Enable SPI high-speed mode: `SD.begin(SS, SPI, 25000000)` (25 MHz)
- Write in larger chunks (512 bytes or more)
- Minimize `file.close()` / `file.open()` cycles

---

## Optimizations for Flight Data Logging

### 1. Keep File Open

**❌ Slow (opens/closes every write):**
```cpp
void loop() {
    File log = SD.open("/data.csv", FILE_APPEND);
    log.printf("%lu,%.2f\n", millis(), data);
    log.close();  // Slow!
}
```

**✅ Fast (keeps file open):**
```cpp
File logFile;

void setup() {
    logFile = SD.open("/data.csv", FILE_APPEND);
}

void loop() {
    logFile.printf("%lu,%.2f\n", millis(), data);
    logFile.flush();  // Force write to card
}
```

---

### 2. Use Buffered Writes

**❌ Slow (writes single lines):**
```cpp
void loop() {
    log.printf("%lu,%.2f\n", millis(), data);
}
```

**✅ Fast (accumulates data, writes in chunks):**
```cpp
char buffer[512];
int bufferPos = 0;

void loop() {
    bufferPos += sprintf(buffer + bufferPos, "%lu,%.2f\n", millis(), data);
    
    if (bufferPos > 400) {  // Write when nearly full
        log.write((uint8_t*)buffer, bufferPos);
        bufferPos = 0;
    }
}
```

---

### 3. Periodic Flush

```cpp
unsigned long lastFlush = 0;

void loop() {
    log.printf("%lu,%.2f\n", millis(), data);
    
    // Flush every second to prevent data loss
    if (millis() - lastFlush > 1000) {
        log.flush();
        lastFlush = millis();
    }
}
```

---

## Applications in Rocketry

This test validates your SD card for:

| Application | Why Important |
|-------------|---------------|
| **Flight Data Logging** | Verify 10+ MB/s write speed |
| **Configuration Files** | Test read/write/rename |
| **Post-Flight Analysis** | Ensure data integrity |
| **Multi-Flight Storage** | Directory organization |
| **Telemetry Backup** | Redundant storage system |

**Recommended File Structure:**
```
/
├── config.ini          (System configuration)
├── flights/
│   ├── flight_001.csv  (Flight 1 data)
│   ├── flight_002.csv  (Flight 2 data)
│   └── flight_003.csv  (Flight 3 data)
└── logs/
    ├── errors.log      (Error log)
    └── system.log      (System events)
```

---
