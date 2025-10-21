# Copilot Instructions for DMD32_HRCxxx Project

## Project Overview
This is a multi-variant ESP32 firmware project for industrial scale/display systems using various communication protocols and display technologies. The codebase supports 8 different hardware variants through conditional compilation flags.

## Hardware Variants & Compilation Flags
The project builds different firmware variants using `#define` flags in `src/main.cpp`:

**Display Types:**
- `HRCZAMAN` - Time display system with 2x4 panel configuration
- `HRCMESAJ` - Message display with relay outputs and serial commands  
- `HRCMAXI` - Maximum display variant
- `HRCMESAJ_RGB` - RGB matrix display using HUB75 panels
- `HRCNANO` - Compact display using ESP32_LED_64x16_Matrix
- `HRCMINI` - Mini display using MD_Parola/MD_MAX72XX

**Communication Protocols:**
- `ESPNOW` - ESP-NOW peer-to-peer communication
- `SERITOUSB` - Serial to USB bridge
- `MODBUS_RTU` - Modbus RTU communication for scale data
- `LEADER` - Thermal printer integration

## Key Architecture Patterns

### Display System Architecture
Each variant uses different display libraries with consistent interface patterns:
- **DMD32**: Uses hardware timer interrupts (`triggerScan()`) for LED multiplexing at 300µs intervals
- **HUB75**: Virtual matrix panel system for RGB displays with DMA optimization
- **MD_Parola**: Scrolling text displays with built-in animation support

### Timer-Driven Display Refresh
```cpp
// Standard pattern for DMD32 variants
timer = timerBegin(1, cpuClock, true);
timerAttachInterrupt(timer, &triggerScan, true);  
timerAlarmWrite(timer, 300, true); // 300µs refresh
```

### Multi-Protocol Communication Stack
- **ESP-NOW**: Peer discovery, pairing management with MAC address persistence
- **Web Server**: OTA updates + device management at 192.168.4.1
- **Serial Protocols**: Scale data parsing with format detection

## Critical Development Workflows

### Build System (PlatformIO)
```bash
# Build for specific environment
pio run --environment nodemcu-32s
pio run --environment esp32-s3

# Upload firmware  
pio run --target upload --environment nodemcu-32s
```

### Display Testing Pattern
Always test display variants with timer management:
```cpp
pauseDMD();    // Stop display refresh during WiFi operations
// ... WiFi operations
resumeDMD();   // Restart display refresh
```

### ESP-NOW Development
- Pairing uses broadcast discovery with "PAIR_REQUEST"/"PAIR_RESPONSE" 
- MAC addresses stored in Preferences with key "paired_mac"
- Peer management through web interface with real-time status

## Project-Specific Conventions

### Font Management
- Custom fonts in `lib/DMD32/fonts/` for segment displays
- Font selection: `dmd.selectFont(Segfont_Sayi_Harf)` before text rendering
- Consistent 15-character padding: `sprintf(msg_c, "%-15s", msg.c_str())`

### Serial Data Processing
Weight data extraction pattern used across variants:
```cpp
String ExtractNumericData(String data);  // Extract numbers from mixed data
void FormatNumericData(String &data, char *output);  // Format for display
```

### Error Handling & Connectivity
- 5-second timeout pattern: `if (millis() - hata_timer > 5000)` 
- LED_PIN (GPIO2) for connection status indication
- Display shows "HATA" messages for communication failures

### Web Interface Integration
- Hidden AP mode with configurable SSID storage in Preferences
- OTA upload endpoint `/update` with display feedback during upload
- Device management APIs for ESP-NOW peer control

## Key Files & Dependencies

**Core Implementation**: `src/main.cpp` (2111 lines, all variants in one file)
**Display Driver**: `lib/DMD32/` - Modified ESP32 SPI-optimized DMD library  
**Build Config**: `platformio.ini` - Multi-environment ESP32 configurations
**Fonts**: `lib/DMD32/fonts/` - Custom industrial display fonts

## Display Refresh Critical Path
DMD32 variants require uninterrupted 300µs timer calls to `triggerScan()`. Any blocking operations (WiFi, file I/O) must use `pauseDMD()`/`resumeDMD()` or run in separate FreeRTOS tasks on core 0.

## Communication Protocol Notes
- **Modbus RTU**: 19200 baud, 7E1 parity for scale communication
- **ESP-NOW**: Channel 0, no encryption, broadcast for discovery
- **Serial Commands**: Line-terminated commands like `str1TEXT`, `mes MESSAGE`
- **Web API**: RESTful endpoints for device management and OTA updates

Always consider the target hardware variant when modifying display, communication, or timing-critical code sections.