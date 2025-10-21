# DMD32_HRCxxx - Multi-Variant ESP32 Display Firmware

Advanced ESP32 firmware project for industrial scale/display systems supporting 8 different hardware variants with multiple communication protocols and display technologies.

## 🚀 Features

### Hardware Variants
- **HRCZAMAN** - Time display system with 2x4 panel configuration
- **HRCMESAJ** - Message display with relay outputs and serial commands
- **HRCMAXI** - Maximum display variant  
- **HRCMESAJ_RGB** - RGB matrix display using HUB75 panels
- **HRCNANO** - Compact display using ESP32_LED_64x16_Matrix
- **HRCMINI** - Mini display using MD_Parola/MD_MAX72XX

### Communication Protocols
- **ESP-NOW** - Peer-to-peer communication with pairing management
- **SERITOUSB** - Serial to USB bridge functionality
- **MODBUS_RTU** - Modbus RTU communication for scale data (19200 baud, 7E1)
- **LEADER** - Thermal printer integration

### Special Features
- **Custom Font System** - Modified Comic24.h font (height reduced from 29→20 pixels)
- **Handcrafted Characters** - Custom binary-format fonts in `src/fonts/`
- **Web Server** - OTA updates + device management at 192.168.4.1
- **Timer-Driven Display** - 300µs refresh intervals for smooth LED multiplexing
- **ESP32 Arduino Framework 2.0.10** - Stable framework for compatibility

## 🛠️ Development Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) (VS Code extension recommended)
- ESP32 development board (NodeMCU-32S, ESP32-S3)

### Build Commands
```bash
# Build for specific environment
pio run --environment nodemcu-32s
pio run --environment esp32-s3

# Upload firmware
pio run --target upload --environment nodemcu-32s

# Monitor serial output
pio device monitor --baud 115200
```

## 📁 Project Structure

```
├── src/
│   ├── main.cpp              # Main application (2111+ lines, all variants)
│   └── fonts/                # Custom handcrafted fonts
│       ├── newFont.h         # Custom MD_MAX72XX font
│       └── dotmatrix_5x8.h   # 5x8 dot matrix font
├── lib/
│   ├── DMD32/               # Modified ESP32 SPI-optimized DMD library
│   │   └── fonts/           # Industrial display fonts
│   │       ├── Comic24.h    # Modified font (height: 29→20)
│   │       └── Segfont_Sayi_Harf.h  # Custom segment font
│   ├── MD_Parola/           # Scrolling text display library
│   ├── ESPAsyncWebServer/   # Async web server for OTA
│   └── ...                  # Other dependencies
└── platformio.ini           # Multi-environment build configuration
```

## ⚙️ Configuration

### Hardware Variant Selection
Edit `src/main.cpp` and uncomment the desired variant:
```cpp
/*DISPLAY*/
//#define HRCZAMAN
//#define HRCMESAJ  
//#define HRCMAXI
//#define HRCMESAJ_RGB
//#define HRCNANO
#define HRCMINI        // Active variant
/*DISPLAY*/

/*COMMUNICATION*/
#define ESPNOW         // Active protocol
//#define SERITOUSB
//#define MODBUS_RTU
/*COMMUNICATION*/
```

### Display Refresh Critical Path
DMD32 variants require uninterrupted 300µs timer calls. Use `pauseDMD()`/`resumeDMD()` for blocking operations:
```cpp
pauseDMD();    // Stop display refresh
// WiFi operations or file I/O
resumeDMD();   // Restart display refresh
```

## 🔧 Custom Fonts

### Modified Comic24.h
- **Original height:** 29 pixels → **Modified:** 20 pixels  
- **Binary format:** Readable B11111100 notation
- **4-pixel shift:** Numbers shifted upward for better display
- **Custom comments:** /*0*/, /*1*/ annotations for each digit

### Custom Character Creation
See `src/fonts/newFont.h` for examples of handcrafted character definitions:
```cpp
5, 126, 255, 195, 255, 126,  // 48 - 0 (custom designed)
5, 0, 4, 6, 255, 255,        // 49 - 1 (custom designed)
```

## 🌐 Web Interface

### Access Points
- **Hidden AP Mode:** Configurable SSID stored in Preferences
- **Management Interface:** http://192.168.4.1
- **OTA Updates:** `/update` endpoint with display feedback

### ESP-NOW Features
- **Peer Discovery:** Broadcast "PAIR_REQUEST"/"PAIR_RESPONSE"
- **MAC Persistence:** Stored in Preferences with key "paired_mac"
- **Real-time Status:** Web interface shows connection status

## 📡 Communication Protocols

### Serial Commands
```
str1TEXT      - Set string 1 to TEXT
mes MESSAGE   - Set message to MESSAGE  
```

### Modbus RTU
- **Baud Rate:** 19200
- **Parity:** 7E1
- **Function:** Scale data communication

### ESP-NOW
- **Channel:** 0 (no encryption)
- **Discovery:** Broadcast mode
- **Range:** Peer-to-peer communication

## 🔍 Troubleshooting

### Display Issues
1. Check hardware connections (SPI pins)
2. Verify timer interrupt settings (300µs)
3. Ensure proper font selection
4. Monitor serial output for "HATA" error messages

### Build Issues
1. Verify ESP32 Arduino Framework 2.0.10
2. Check library dependencies in `lib/` folder
3. Ensure correct environment in `platformio.ini`

### Communication Problems
1. Verify baud rates (115200 for debug, 19200 for Modbus)
2. Check ESP-NOW pairing status
3. Monitor 5-second timeout patterns

## 📄 License

This project contains multiple libraries with different licenses. See individual library folders for specific license information.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Test with relevant hardware variant
4. Submit a pull request with detailed description

## 📞 Support

For hardware-specific issues, ensure you specify:
- Hardware variant (HRCZAMAN, HRCMINI, etc.)
- Communication protocol (ESP-NOW, Modbus, etc.)
- ESP32 board type (NodeMCU-32S, ESP32-S3)
- Serial monitor output (if applicable)

---

**Built with ESP32 Arduino Framework 2.0.10 for maximum stability and compatibility.**