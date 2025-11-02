/*DİSPLAY*/
//#define HRCZAMAN
//#define HRCMESAJ
//#define HRCMAXI
//#define HRCMESAJ_RGB
//#define HRCNANO
#define HRCMINI
/*DİSPLAY*/

/*COMMUNICATION*/
#define ESPNOW
//#define SERITOUSB
//#define MODBUS_RTU
/*COMMUNICATION*/

/*MODEL*/
#define MODEL "HRCMINI"
/*MODEL*/

//#define LEADER

// Debug sistemi - Global olarak en üstte tanımlanmalı
bool debugEnabled = true; // Başlangıçta açık

// Debug makroları
#define DEBUG_PRINT(x) if(debugEnabled) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if(debugEnabled) { Serial.println(x); }
#define DEBUG_PRINTF(...) if(debugEnabled) { Serial.printf(__VA_ARGS__); }

#include <Arduino.h>

#ifdef HRCMESAJ_RGB

#include <ESP32-HUB75-VirtualMatrixPanel_T.hpp>

#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMono24pt7b.h>

#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <Fonts/FreeSans7pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>

#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#include <Fonts/FreeMonoBoldOblique12pt7b.h>

// 2 Panel - ICN2037 1/8 scan
#define PANEL_RES_X 64
#define PANEL_RES_Y 32

#define VDISP_NUM_ROWS 1
#define VDISP_NUM_COLS 2
#define PANEL_SCAN_TYPE FOUR_SCAN_32PX_HIGH
#define PANEL_CHAIN_TYPE CHAIN_TOP_RIGHT_DOWN

#define PANEL_CHAIN_LEN (VDISP_NUM_ROWS * VDISP_NUM_COLS)

MatrixPanel_I2S_DMA *dma_display = nullptr;
using MyScanTypeMapping = ScanTypeMapping<PANEL_SCAN_TYPE>;
VirtualMatrixPanel_T<PANEL_CHAIN_TYPE, MyScanTypeMapping>* virtualDisp = nullptr;

int16_t x_pos; // Yazının X konumu
int16_t textWidth; // Yazının genişliği (piksel)

#endif

// --- ESP-NOW message structure and parser -------------------------------------------------
// Expected message format (pipe-separated):
// NAME|MODEL|RSSI|CMD|DATA
// Example: "HRCMINI|DMD32_HRC| -42 |DISPLAY|Merhaba"

typedef struct {
  String name;   // device name (same as SSID)
  String model;  // model name
  int rssi;      // signal strength (optional, -128 if unknown)
  String cmd;    // command
  String data;   // payload to display
} EspNowMessage;

// Parse a pipe-separated payload into EspNowMessage. Returns true on success.
bool parseEspNowMessage(const char *payload, int len, EspNowMessage &msg) {
  if (payload == nullptr || len <= 0) return false;
  String s = String(payload).substring(0, len);
  
  // Yeni format kontrolü: NAME|MODEL|RSSI|CMD|DATA (5 parça olmalı)
  // Find first four separators. DATA may contain pipes, so extract first 4 splits only.
  int p1 = s.indexOf('|');
  if (p1 < 0) return false; // Hiç | yoksa false döndür
  
  int p2 = s.indexOf('|', p1 + 1);
  int p3 = s.indexOf('|', p2 + 1);
  int p4 = s.indexOf('|', p3 + 1);
  
  // Eğer 4 tane | varsa yeni format
  if (p2 > 0 && p3 > 0 && p4 > 0) {
    msg.name = s.substring(0, p1);
    msg.model = s.substring(p1 + 1, p2);
    String rssiStr = s.substring(p2 + 1, p3);
    rssiStr.trim();
    if (rssiStr.length() == 0) msg.rssi = -128;
    else msg.rssi = rssiStr.toInt();
    msg.cmd = s.substring(p3 + 1, p4);
    msg.data = s.substring(p4 + 1);
    return true;
  }
  
  return false; // Yeni format değilse false döndür
}


#ifdef HRCNANO
#include <Arduino.h>
#include "ESP32_LED_64x16_Matrix.h"

// Display objesi
ESP32_LED_64x16_Matrix display;

gpio_num_t pins[8] = {
  GPIO_NUM_16,  // latch
  GPIO_NUM_17,  // clock
  GPIO_NUM_18,  // data_R1
  GPIO_NUM_5,   // en_74138
  GPIO_NUM_23,  // la
  GPIO_NUM_22,  // lb
  GPIO_NUM_21,  // lc
  GPIO_NUM_19   // ld
};
#endif

#ifdef MODBUS_RTU

#include <ModbusMaster.h>

ModbusMaster node;

/*#include <ModbusRTU.h>

#define SLAVE_ID 1
#define FIRST_REG 0
#define REG_COUNT 2

ModbusRTU mb;

uint16_t res[2];

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Callback to monitor errors
  if (event != Modbus::EX_SUCCESS) {
    DEBUG_PRINT("Request result: 0x");
    DEBUG_PRINT(event, HEX);
  }
  return true;
}*/
#endif

String incomingBuffer = "";
char incomingChar = ' ';
int dataIndex = 0;             // Buffer icin indeks
char receivedData[24]; // Gelen veriyi tutacak buffer

bool isPaired = false; // Eslesme durumu

/************************************************ */
#ifdef MODBUS_RTU
#include <HardwareSerial.h>
HardwareSerial modbusSerial(1);
/****************************************** */
#endif

#include <Preferences.h>
#include <ctype.h> // isdigit fonksiyonu icin
#include <ArduinoJson.h>
Preferences preferences;

#ifdef ESPNOW
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>


bool Serial1_mod = false;
// Degiskenler
#define MAX_PAIRED_DEVICES 6


uint8_t pairedMacList[MAX_PAIRED_DEVICES][6];  // Eşleşmiş cihazların MAC adresleri
int pairedDeviceCount = 0;

struct PeerStatus {
  uint8_t mac[6];
  bool active;
};

PeerStatus peerStatusList[MAX_PAIRED_DEVICES];
int peerStatusCount = 0;

// Tarama sonucu geçici cihaz listesi
uint8_t discoveredMacList[MAX_PAIRED_DEVICES][6];
int discoveredCount = 0;

// Broadcast adresini belirle
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

unsigned long buttonPressStartTime = 0; // Butona basılma baslangıc zamanı
bool buttonPressed = false;

// Last received RSSI value
int lastReceivedRSSI = -50; // Default RSSI değeri

#endif

#if defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
#include <DMD32.h>
#include "fonts/Segfont_7x16.h"
#include "fonts/Segfont_Sayi_Harf.h"
#include "fonts/Arial_38b.h"
#include "fonts/Comic24.h"
#include "fonts/Arial_Black_16.h"
#include "fonts/SystemFont5x7.h"
#include "fonts/SystemFont5x7_ENDUTEK.h"

// DMD ayarları
#if defined (HRCMESAJ)
#define DISPLAYS_ACROSS 2
#define DISPLAYS_DOWN 1
//#define DISPLAYS_ACROSS 3
//#define DISPLAYS_DOWN 3
#elif defined (HRCZAMAN)
#define DISPLAYS_ACROSS 2
#define DISPLAYS_DOWN 4
#elif defined (HRCMAXI)
#define DISPLAYS_ACROSS 2
#define DISPLAYS_DOWN 1
#endif
DMD dmd(DISPLAYS_ACROSS, DISPLAYS_DOWN);

// Timer ayarları
hw_timer_t *timer = NULL;

// Ekranda metin gosterme fonksiyonu
void ekran_goster(int imlec_x, int imlec_y, String msg_s) {
  char msg_c[16]; 
  msg_s = msg_s.substring(0, 15); // 14 karaktere kes
  msg_s.toUpperCase();
  sprintf(msg_c, "%-15s", msg_s.c_str());  // Boslukları doldur
  dmd.drawString(imlec_x, imlec_y, msg_c, 15, GRAPHICS_NORMAL);
}

// DMD tarama islevi
void IRAM_ATTR triggerScan() {
  digitalWrite(PIN_DMD_nOE, LOW); // LED'leri aç
  dmd.scanDisplayBySPI();
  digitalWrite(PIN_DMD_nOE, HIGH); // LED'leri kapat
}

// WiFi islemleri sırasında DMD taramayı durdurup tekrar baslatma
void pauseDMD() {
  timerAlarmDisable(timer);  // DMD tarama zamanlayıcısını durdurun
  DEBUG_PRINTLN("DMD tarama durduruldu");
}

void resumeDMD() {
  timerAlarmEnable(timer);  // DMD tarama zamanlayıcısını tekrar baslatın
  DEBUG_PRINTLN("DMD tarama yeniden baslatıldı");
}

#if defined(HRCZAMAN)
void handleSerialData() {
  while (Serial.available() > 0) {
    incomingChar = Serial.read();
    incomingBuffer += incomingChar;

    if (incomingChar == '\n') {
      incomingBuffer.trim();  // Basındaki ve sonundaki boslukları temizle

      if (incomingBuffer.startsWith("str1")) {
        dmd.selectFont(System5x7_ENDUTEK);
        ekran_goster(0, 0, incomingBuffer.substring(4,20));
        ekran_goster(0, 33, incomingBuffer.substring(4,20));       
      }
      if (incomingBuffer.startsWith("str2")) {
        dmd.selectFont(Comic24);
        ekran_goster(0, 8, incomingBuffer.substring(4,20));
        ekran_goster(0, 41, incomingBuffer.substring(4,20));
      }
      incomingBuffer = "";  // Tamponu sıfırla
    }
  }
}
#endif

#ifdef HRCMESAJ
int out1_pin = 12;
int out2_pin = 14;

// Seri porttan gelen verileri isleme ve ekran kontrolu
void handleSerialData() {
  while (Serial.available() > 0) {
    incomingChar = Serial.read();
    incomingBuffer += incomingChar;
    if (incomingChar == 255)  // binary data...
    {
      Serial.flush();
      return;
    }
    if (incomingChar == '\n') {
      incomingBuffer.trim();  // Basındaki ve sonundaki boslukları temizle

      if (incomingBuffer.startsWith("satir1sil")) {
        ekran_goster(0, 0, "");
      } else if (incomingBuffer.startsWith("satir2sil")) {
        ekran_goster(0, 16, "");
      } else if (incomingBuffer.startsWith("satir3sil")) {
        ekran_goster(0, 32, "");
      } else if (incomingBuffer.startsWith("satir1yaz")) {
        ekran_goster(0, 0, incomingBuffer.substring(9));
      } else if (incomingBuffer.startsWith("satir2yaz")) {
        ekran_goster(0, 16, incomingBuffer.substring(9));
      } else if (incomingBuffer.startsWith("satir3yaz")) {
        ekran_goster(0, 32, incomingBuffer.substring(9));
      } else if (incomingBuffer.startsWith("cikis1acik")) {
        digitalWrite(out1_pin, LOW);
      } else if (incomingBuffer.startsWith("cikis2acik")) {
        digitalWrite(out2_pin, LOW);
      } else if (incomingBuffer.startsWith("cikis1kapali")) {
        digitalWrite(out1_pin, HIGH);
      } else if (incomingBuffer.startsWith("cikis2kapali")) {
        digitalWrite(out2_pin, HIGH);
      }
      else if (incomingBuffer.startsWith("sil")) {
        dmd.selectFont(Segfont_Sayi_Harf);
        ekran_goster(0, 0, ""); 
      }
      else if (incomingBuffer.startsWith("mes")) {
        dmd.selectFont(Segfont_Sayi_Harf);
        String mesaj1 = incomingBuffer.substring(3,18);
        mesaj1.toUpperCase();
        ekran_goster(0, 0, mesaj1);  
      }
      else if (incomingBuffer.startsWith("str1")) {
        dmd.selectFont(System5x7_ENDUTEK);
        String mesaj1 = incomingBuffer.substring(4,24);
        mesaj1.toUpperCase();
        ekran_goster(0, 0, mesaj1);        
      }
      else if (incomingBuffer.startsWith("str2")) {
        dmd.selectFont(System5x7_ENDUTEK);
        String mesaj2 = incomingBuffer.substring(4,24);
        mesaj2.toUpperCase();
        ekran_goster(0, 8, mesaj2);        
      }
      else if (incomingBuffer.startsWith("kay")) {
        dmd.selectFont(Segfont_Sayi_Harf);
        String mesaj3 = incomingBuffer.substring(3,64);
        mesaj3.toUpperCase();
        dmd.drawMarquee(mesaj3.c_str(),mesaj3.length(),(32*DISPLAYS_ACROSS)-1,0);
        long start=millis();
        long timer=start;
        boolean ret=false;
        while(!ret){
          if ((timer+80) < millis()) {
            ret=dmd.stepMarquee(-1,0);
            timer=millis();
          }
        }      
      }
      else if (incomingBuffer.startsWith("kystr1")) {
        dmd.selectFont(System5x7_ENDUTEK);
        String mesaj3 = incomingBuffer.substring(3,64);
        mesaj3.toUpperCase();
        dmd.drawMarquee(mesaj3.c_str(),mesaj3.length(),(32*DISPLAYS_ACROSS)-1,0);
        long start=millis();
        long timer=start;
        boolean ret=false;
        while(!ret){
          if ((timer+80) < millis()) {
            ret=dmd.stepMarquee(-1,0);
            timer=millis();
          }
        }      
      }
      else if (incomingBuffer.startsWith("kystr2")) {
        dmd.selectFont(System5x7_ENDUTEK);
        String mesaj3 = incomingBuffer.substring(3,64);
        mesaj3.toUpperCase();
        dmd.drawMarquee(mesaj3.c_str(),mesaj3.length(),(32*DISPLAYS_ACROSS)-1,0);
        long start=millis();
        long timer=start;
        boolean ret=false;
        while(!ret){
          if ((timer+80) < millis()) {
            ret=dmd.stepMarquee(-1,8);
            timer=millis();
          }
        }      
      }
      incomingBuffer = "";  // Tamponu sıfırla
    }
  }
}
#endif
#endif

#if defined(HRCMESAJ_RGB) 
void handleSerialData() {
  while (Serial.available() > 0) {
    incomingChar = Serial.read();
    incomingBuffer += incomingChar;

    if (incomingChar == '\n') {
      incomingBuffer.trim();  // Basındaki ve sonundaki boslukları temizle

      if (incomingBuffer.startsWith("mes")) {
        virtualDisp->clearScreen();
        //virtualDisp->setFont();
        //virtualDisp->setTextSize(1);
        //virtualDisp->setTextWrap(true);
        //virtualDisp->setCursor(0, 1);
        //virtualDisp->setTextColor(virtualDisp->color565(255, 100, 100));
        //virtualDisp->print("TANK 1");
        //virtualDisp->setCursor(43, 1);
        //virtualDisp->setTextColor(virtualDisp->color565(100, 255, 100));
        //virtualDisp->print(" TARTIM");
        //virtualDisp->setCursor(86, 1);
        //virtualDisp->setTextColor(virtualDisp->color565(100, 100, 255));
        //virtualDisp->print(" ONAYLANDI");
        virtualDisp->setFont(&FreeSansBold9pt7b); 
        virtualDisp->setTextColor(virtualDisp->color565(255,0,0));
        virtualDisp->setCursor(0,13);
        virtualDisp->print(incomingBuffer.substring(3,15));

// Kırmızı çizgi
uint16_t redColor = virtualDisp->color565(255,255,255);
// Y1 ve Y2 koordinatlarını panel yüksekliğine göre ayarlayabilirsin
// Örnek: Yatay çizgiyi 18. piksele koyuyorum
virtualDisp->drawFastHLine(
    0,                  // X başlangıcı
    15,                 // Y yüksekliği
    virtualDisp->width(), // Çizginin uzunluğu (panelin genişliği kadar)
    redColor            // Renk
);

        virtualDisp->setCursor(0,30);
        virtualDisp->setTextColor(virtualDisp->color565(0,255,0));
        virtualDisp->print(incomingBuffer.substring(15,27));
      }
      incomingBuffer = "";  // Tamponu sıfırla
    }
  }
}
#endif

unsigned long hata_timer = 0;

// Pin tanimlari
#define PAIR_BUTTON 0  // Dugme icin GPIO pini
#define LED_PIN 2      // Durum gostergesi icin LED pini
boolean hata = false;
String sonformattedText;

#ifdef HRCMINI
#include "MD_Parola.h"
#include "MD_MAX72xx.h"
#include <SPI.h>

// Font dosyalarını include et
#include "fonts/dotmatrix_5x8.h"
#include "fonts/newFont.h"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

// MD_MAX72XX ve MD_Parola ayarlari
#define MAX_DEVICES 5

#define CLK_PIN   14  // SCK
#define DATA_PIN  13  // MOSI
#define CS_PIN    27  // CS

// Software SPI ile MD_Parola (orijinal pin konfigürasyonu)
MD_Parola display = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Current queued display message (non-blocking display handling)
String currentDisplayMessage = "";
bool displayMessageQueued = false;

// Startup message sequence
unsigned long startupMessageTimer = 0;
int startupMessageIndex = -1;
String startupMessages[] = {"HRCMINI", " BILTER"};
int startupMessagesCount = 2;

// Connection status tracking
bool lastConnectionStatus = false;
bool connectionStatusChanged = false;
unsigned long connectionCheckTimer = 0;
bool startupComplete = false;

void ShowOnDisplay(String message) {
    
    // ESP-NOW mesajları için özel işlem (her zaman static)
    bool isESPNOWMessage = (message.indexOf("(") != -1);
    
    // ENDUTEK ve uzun mesajlar için kayan yazı kontrolü
    bool needsScrolling = false;
    if (message == " BILTER" || (!isESPNOWMessage && message != "HATA" && message != "HRCMINI")) {
        //  BILTER her zaman kayan yazı
        if (message == " BILTER") {
            needsScrolling = true;
            //DEBUG_PRINTLN(" BILTER - forced scroll animation");
        } else {
            // Diğer mesajlar için genişlik kontrolü
            int textWidth = message.length() * 6;  // Her karakter ~6 pixel
            int displayWidth = 1 * 40; // 5 modül, her modül 32 pixel genişlik = 160 pixel
            
            if (textWidth > displayWidth) {
                needsScrolling = true;
                //Serial.println("Text too wide (" + String(textWidth) + "px > " + String(displayWidth) + "px), using scroll");
            } else {
                DEBUG_PRINTLN("Text fits (" + String(textWidth) + "px <= " + String(displayWidth) + "px), using static");
            }
        }
    }
    
    // BLOCKING APPROACH - Hemen göster ve animasyonu bekle
    if (message == "HATA" || message == "HRCMINI" || isESPNOWMessage || !needsScrolling) {
        // Static göster (sola yaslanmış)
        display.displayText(message.c_str(), PA_LEFT, 0, 0, PA_PRINT, PA_NO_EFFECT);
        //Serial.println("Static message set (LEFT aligned) - blocking animate...");
        // BLOCKING animate
        unsigned long timeout = millis() + 2000;
        while (!display.displayAnimate() && millis() < timeout) {
            delay(50);
        }
        //DEBUG_PRINTLN("Static animation completed");
    } else {
          // Ekranı temizle
        display.displayClear();
        display.displayReset(); 
        // Uzun mesajlar scroll göster
        display.displayText(message.c_str(), PA_CENTER, 60, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
        //DEBUG_PRINTLN("Scroll message set - blocking animate...");
        // BLOCKING animate - max 5 saniye
        unsigned long timeout = millis() + 5000;
        int animCount = 0;
        while (!display.displayAnimate() && millis() < timeout) {
            delay(30);
            animCount++;
            if (animCount % 20 == 0) {
                //Serial.println("Animation step: " + String(animCount));
            }
        }
        //Serial.println("Scroll animation completed after " + String(animCount) + " steps");
    }
    
    currentDisplayMessage = message;
    displayMessageQueued = false; // Animasyon tamamlandı
}

// Caller (main loop) should call display.displayAnimate() repeatedly to progress animations.

#endif

#include <ESPAsyncWebServer.h>
#include <Update.h>

const char* ssid = "HRC";  // WiFi SSID
const char* password = "teraziwifi";  // WiFi Şifresi

AsyncWebServer server(80);
TaskHandle_t wifiTaskHandle = NULL;

// Veri monitör için buffer'lar
struct DataEntry {
  String message;
  unsigned long timestamp;
  String type; // "in" veya "out" ESP-NOW için
};

const int MAX_DATA_ENTRIES = 20;
DataEntry serialDataBuffer[MAX_DATA_ENTRIES];
DataEntry espnowDataBuffer[MAX_DATA_ENTRIES];
int serialDataIndex = 0;
int espnowDataIndex = 0;

// Veri ekleme fonksiyonları
void addSerialData(String message) {
  serialDataBuffer[serialDataIndex].message = message;
  serialDataBuffer[serialDataIndex].timestamp = millis();
  serialDataIndex = (serialDataIndex + 1) % MAX_DATA_ENTRIES;
}

void addEspnowData(String message, String type) {
  espnowDataBuffer[espnowDataIndex].message = message;
  espnowDataBuffer[espnowDataIndex].type = type;
  espnowDataBuffer[espnowDataIndex].timestamp = millis();
  espnowDataIndex = (espnowDataIndex + 1) % MAX_DATA_ENTRIES;
}

// Web sayfası ile firmware yuklemek icin HTML sayfası
#if defined(HRCMINI) || defined(HRCMESAJ_RGB) || defined(HRCNANO)
const char* upload_html = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>HRCMINI Yönetim Paneli</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh; padding: 20px;
    }
    .container {
      max-width: 1200px; margin: 0 auto;
      background: rgba(255,255,255,0.95); border-radius: 20px;
      box-shadow: 0 20px 40px rgba(0,0,0,0.1); padding: 30px;
      backdrop-filter: blur(10px);
    }
    .header {
      text-align: center; margin-bottom: 40px;
      color: #2c3e50; border-bottom: 2px solid #3498db; padding-bottom: 20px;
    }
    .header h1 { font-size: 2.5em; margin-bottom: 10px; }
    .header p { color: #7f8c8d; font-size: 1.1em; }
    
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(350px, 1fr)); gap: 25px; }
    .card {
      background: white; border-radius: 15px; padding: 25px;
      box-shadow: 0 8px 25px rgba(0,0,0,0.1); border: 1px solid #e8f4f8;
      transition: all 0.3s ease; position: relative; overflow: hidden;
    }
    .card::before {
      content: ''; position: absolute; top: 0; left: 0; right: 0; height: 4px;
      background: linear-gradient(90deg, #3498db, #2ecc71); border-radius: 15px 15px 0 0;
    }
    .card:hover { transform: translateY(-5px); box-shadow: 0 15px 35px rgba(0,0,0,0.15); }
    
    .card-title {
      font-size: 1.4em; color: #2c3e50; margin-bottom: 20px;
      display: flex; align-items: center; gap: 10px;
    }
    .card-title .icon { font-size: 1.5em; }
    
    .form-group { margin-bottom: 20px; }
    .form-label { display: block; margin-bottom: 8px; color: #34495e; font-weight: 600; }
    .form-input {
      width: 100%; padding: 12px 15px; border: 2px solid #e8f4f8;
      border-radius: 10px; font-size: 16px; transition: all 0.3s ease;
      background: #f8fbff;
    }
    .form-input:focus {
      outline: none; border-color: #3498db; background: white;
      box-shadow: 0 0 0 3px rgba(52, 152, 219, 0.1);
    }
    
    .btn {
      padding: 12px 25px; border: none; border-radius: 10px; font-size: 16px;
      font-weight: 600; cursor: pointer; transition: all 0.3s ease;
      display: inline-flex; align-items: center; gap: 8px; text-decoration: none;
    }
    .btn-primary { background: #3498db; color: white; }
    .btn-primary:hover { background: #2980b9; transform: translateY(-2px); }
    .btn-success { background: #27ae60; color: white; }
    .btn-success:hover { background: #219a52; transform: translateY(-2px); }
    .btn-warning { background: #f39c12; color: white; }
    .btn-warning:hover { background: #e67e22; transform: translateY(-2px); }
    .btn-danger { background: #e74c3c; color: white; }
    .btn-danger:hover { background: #c0392b; transform: translateY(-2px); }
    .btn-small { padding: 8px 15px; font-size: 14px; }
    
    .table-container {
      overflow-x: auto; border-radius: 10px; 
      box-shadow: inset 0 2px 8px rgba(0,0,0,0.05);
    }
    table {
      width: 100%; border-collapse: collapse; background: white;
    }
    th, td {
      padding: 15px; text-align: left; border-bottom: 1px solid #e8f4f8;
    }
    th {
      background: #f8fbff; color: #2c3e50; font-weight: 600;
      border-bottom: 2px solid #3498db;
    }
    tr:hover { background: #f8fbff; }
    
    .status { padding: 5px 12px; border-radius: 20px; font-size: 12px; font-weight: 600; }
    .status-online { background: #d4edda; color: #155724; }
    .status-offline { background: #f8d7da; color: #721c24; }
    .status-discovered { background: #fff3cd; color: #856404; }
    
    .progress-bar {
      width: 100%; height: 8px; background: #e8f4f8; border-radius: 10px; overflow: hidden;
      margin: 15px 0;
    }
    .progress-fill {
      height: 100%; background: linear-gradient(90deg, #3498db, #2ecc71);
      transition: width 0.3s ease; border-radius: 10px;
    }
    
    .notification {
      position: fixed; top: 20px; right: 20px; padding: 15px 20px;
      border-radius: 10px; color: white; font-weight: 600; z-index: 1000;
      transform: translateX(400px); transition: transform 0.3s ease;
    }
    .notification.show { transform: translateX(0); }
    .notification.success { background: #27ae60; }
    .notification.error { background: #e74c3c; }
    .notification.warning { background: #f39c12; }
    
    .modal {
      display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0;
      background: rgba(0,0,0,0.5); z-index: 2000; backdrop-filter: blur(5px);
    }
    .modal-content {
      position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
      background: white; padding: 30px; border-radius: 15px; max-width: 400px; width: 90%;
      box-shadow: 0 20px 40px rgba(0,0,0,0.2);
    }
    .modal-header { margin-bottom: 20px; }
    .modal-title { font-size: 1.3em; color: #2c3e50; }
    .modal-buttons { display: flex; gap: 15px; justify-content: flex-end; margin-top: 25px; }
    
    .data-monitor {
      background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 10px;
      font-family: 'Courier New', monospace; height: 200px; overflow-y: auto;
      border: 2px solid #34495e;
    }
    .data-line { margin-bottom: 5px; }
    .data-serial { color: #3498db; }
    .data-espnow-in { color: #27ae60; }
    .data-espnow-out { color: #f39c12; }
    
    @media (max-width: 768px) {
      .container { padding: 15px; margin: 10px; }
      .grid { grid-template-columns: 1fr; }
      .header h1 { font-size: 2em; }
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>� HRCMINI Yönetim Paneli</h1>
      <p>Cihaz ayarları, bağlantı yönetimi ve sistem kontrolü</p>
    </div>

    <div class="grid">
      <!-- Wi-Fi Ayarları -->
      <div class="card">
        <div class="card-title">
          <span class="icon">📶</span>
          Wi-Fi Ayarları
        </div>
        
        <form onsubmit="updateSettings(); return false;">
          <div class="form-group">
            <label class="form-label">SSID (Ağ Adı)</label>
            <input type="text" id="ssidInput" class="form-input" placeholder="Yeni SSID adını girin" required>
          </div>
          <div class="form-group">
            <label class="form-label">Şifre</label>
            <input type="password" id="passwordInput" class="form-input" placeholder="Wi-Fi şifresini girin">
          </div>
          <button type="submit" class="btn btn-primary">
            <span>💾</span> Kaydet
          </button>
        </form>
      </div>

      <!-- ESP-NOW Eşleştirme -->
      <div class="card">
        <div class="card-title">
          <span class="icon">🤝</span>
          ESP-NOW Eşleştirme
        </div>
        <div class="form-group">
          <button onclick="startPairing()" class="btn btn-success" style="width: 100%;">
            <span>�</span> Cihaz Tarama Başlat
          </button>
        </div>
        <div class="form-group">
          <label class="form-label">Manuel Cihaz Ekle</label>
          <div style="display: flex; gap: 10px;">
            <input type="text" id="macInput" class="form-input" placeholder="AA:BB:CC:DD:EE:FF" style="flex: 1;">
            <button onclick="addMac()" class="btn btn-primary">
              <span>➕</span> Ekle
            </button>
          </div>
        </div>
      </div>

      <!-- Parlaklık Ayarı -->
      <div class="card">
        <div class="card-title">
          <span class="icon">🔆</span>
          Ekran Parlaklığı
        </div>
        <div class="form-group">
          <label class="form-label">Parlaklık Seviyesi: <span id="brightnessValue">50</span>%</label>
          <input type="range" id="brightnessSlider" min="0" max="100" value="50" class="form-input" 
                 style="width: 100%;" onchange="updateBrightness(this.value)">
        </div>
        <button onclick="saveBrightness()" class="btn btn-warning">
          <span>💡</span> Parlaklığı Kaydet
        </button>
      </div>

      <!-- Debug Ayarları -->
      <div class="card">
        <div class="card-title">
          <span class="icon">🐛</span>
          Debug Sistemi
        </div>
        <div class="form-group">
          <label class="form-label">Serial Debug Mesajları</label>
          <div style="display: flex; align-items: center; gap: 10px; margin-top: 8px;">
            <button id="debugToggle" onclick="toggleDebug()" class="btn">
              <span id="debugIcon">🟢</span> <span id="debugText">Açık</span>
            </button>
            <span id="debugStatus" style="color: #27ae60; font-weight: bold;">Debug mesajları aktif</span>
          </div>
        </div>
      </div>

      <!-- Firmware Güncelleme -->
      <div class="card">
        <div class="card-title">
          <span class="icon">📦</span>
          Firmware Güncelleme
        </div>
        <form method="POST" action="/update" enctype="multipart/form-data" onsubmit="return confirmFirmwareUpdate()">
          <div class="form-group">
            <label class="form-label">Firmware Dosyası (.bin)</label>
            <input type="file" name="update" class="form-input" accept=".bin" required>
          </div>
          <div class="progress-bar" id="uploadProgress" style="display: none;">
            <div class="progress-fill" id="progressFill"></div>
          </div>
          <button type="submit" class="btn btn-warning">
            <span>⬆️</span> Firmware Yükle
          </button>
        </form>
      </div>

      <!-- Sistem İşlemleri -->
      <div class="card">
        <div class="card-title">
          <span class="icon">🗑️</span>
          Sistem İşlemleri
        </div>
        <div class="form-group">
          <button onclick="clearPrefs()" class="btn btn-danger" style="width: 100%; margin-bottom: 10px;">
            <span>�</span> Hafızayı Temizle
          </button>
          <button onclick="restartDevice()" class="btn btn-warning" style="width: 100%;">
            <span>🔄</span> Cihazı Yeniden Başlat
          </button>
        </div>
      </div>

      <!-- Serial Veri İzleme -->
      <div class="card">
        <div class="card-title">
          <span class="icon">�</span>
          Serial Veri İzleme
        </div>
        <div class="data-monitor" id="serialMonitor">
          <div class="data-line data-serial">Serial veri bekleniyor...</div>
        </div>
        <button onclick="clearSerialMonitor()" class="btn btn-small btn-primary">
          <span>🧹</span> Temizle
        </button>
      </div>

      <!-- ESP-NOW Veri İzleme -->
      <div class="card">
        <div class="card-title">
          <span class="icon">📶</span>
          ESP-NOW Veri İzleme
        </div>
        <div class="data-monitor" id="espnowMonitor">
          <div class="data-line data-espnow-in">ESP-NOW veri bekleniyor...</div>
        </div>
        <button onclick="clearEspnowMonitor()" class="btn btn-small btn-success">
          <span>🧹</span> Temizle
        </button>
      </div>

      <!-- Ekran Test -->
      <div class="card">
        <div class="card-title">
          <span class="icon">📺</span>
          Ekran Test
        </div>
        <div class="form-group">
          <label class="form-label">Test Mesajı (max 23 karakter)</label>
          <input type="text" id="testMessage" class="form-input" placeholder="Test mesajını girin..." maxlength="23">
        </div>
        <div class="form-group" style="display: flex; gap: 10px; flex-wrap: wrap;">
          <button onclick="sendTestMessage('normal')" class="btn btn-primary">
            <span>📝</span> Normal Göster
          </button>
          <button onclick="sendTestMessage('scroll')" class="btn btn-success">
            <span>➡️</span> Kayan Göster
          </button>
          <button onclick="clearDisplay()" class="btn btn-warning">
            <span>🧹</span> Ekranı Sil
          </button>
        </div>
      </div>

      <!-- ESP-NOW Mesaj Gönder -->
      <div class="card">
        <div class="card-title">
          <span class="icon">📡</span>
          ESP-NOW Mesaj Gönder
        </div>
        <div class="form-group">
          <label class="form-label">Pair'lere Gönderilecek Mesaj</label>
          <input type="text" id="espnowMessage" class="form-input" placeholder="ESP-NOW mesajını girin..." maxlength="50">
        </div>
        <div class="form-group">
          <button onclick="sendEspnowMessage()" class="btn btn-success" style="width: 100%;">
            <span>📡</span> Eşleşmiş Cihazlara Gönder
          </button>
        </div>
        <div class="form-group">
          <small style="color: #7f8c8d;">
            ℹ️ Mesaj tüm eşleşmiş cihazlara broadcast edilecek
          </small>
        </div>
      </div>
    </div>

    <!-- Eşleşmiş Cihazlar Tablosu -->
    <div class="card" style="margin-top: 25px;">
      <div class="card-title">
        <span class="icon">📡</span>
        Eşleşmiş Cihazlar
      </div>
      <div class="table-container">
        <table>
          <thead>
            <tr><th>MAC Adresi</th><th>Name</th><th>Model</th><th>RSSI</th><th>Durum</th><th>İşlemler</th></tr>
          </thead>
          <tbody id="pairedDevices"></tbody>
        </table>
      </div>
    </div>

    <!-- Çevredeki Cihazlar Tablosu -->
    <div class="card" style="margin-top: 25px;">
      <div class="card-title">
        <span class="icon">🔍</span>
        Çevredeki Cihazlar
      </div>
      <div class="table-container">
        <table>
          <thead>
            <tr><th>MAC Adresi</th><th>Name</th><th>Model</th><th>RSSI</th><th>Durum</th><th>İşlemler</th></tr>
          </thead>
          <tbody id="discoveredDevices"></tbody>
        </table>
      </div>
    </div>
  </div>

  <!-- Bildirimler -->
  <div id="notification" class="notification"></div>

  <!-- Onay Modalı -->
  <div id="confirmModal" class="modal">
    <div class="modal-content">
      <div class="modal-header">
        <h3 class="modal-title" id="modalTitle">İşlemi Onayla</h3>
      </div>
      <p id="modalMessage">Bu işlemi gerçekleştirmek istediğinizden emin misiniz?</p>
      <div class="modal-buttons">
        <button onclick="closeModal()" class="btn btn-primary">İptal</button>
        <button onclick="confirmAction()" class="btn btn-danger" id="confirmBtn">Onayla</button>
      </div>
    </div>
  </div>


<script>
// Global değişkenler
let currentAction = null;
let currentActionData = null;
let dataUpdateInterval;
let deviceInfo = {}; // MAC -> {name, model, rssi} mapping

// Sayfa yüklendiğinde başlat
document.addEventListener('DOMContentLoaded', function() {
  fetchDevices();
  loadBrightness(); // Parlaklık ayarını yükle
  loadDebugStatus(); // Debug durumunu yükle
  loadSSID(); // SSID ayarını yükle
  startDataMonitoring();
  setInterval(fetchDevices, 3000);
});

// Bildirim sistemi
function showNotification(message, type = 'success') {
  const notification = document.getElementById('notification');
  notification.textContent = message;
  notification.className = `notification ${type}`;
  notification.classList.add('show');
  
  setTimeout(() => {
    notification.classList.remove('show');
  }, 4000);
}

// Modal işlemleri
function showModal(title, message, action, data = null) {
  document.getElementById('modalTitle').textContent = title;
  document.getElementById('modalMessage').textContent = message;
  document.getElementById('confirmModal').style.display = 'block';
  currentAction = action;
  currentActionData = data;
}

function closeModal() {
  document.getElementById('confirmModal').style.display = 'none';
  currentAction = null;
  currentActionData = null;
}

function confirmAction() {
  if (currentAction) {
    currentAction(currentActionData);
  }
  closeModal();
}

// Wi-Fi ayarlarını güncelle
function updateSettings() {
  const ssid = document.getElementById('ssidInput').value;
  const password = document.getElementById('passwordInput').value;
  
  if (!ssid.trim()) {
    showNotification('SSID boş olamaz!', 'error');
    return;
  }
  
  showModal(
    '⚠️ Wi-Fi Ayarlarını Kaydet',
    `SSID: "${ssid}" olarak güncellenecek. Cihaz yeniden başlayacak. Devam etmek istiyor musunuz?`,
    function() {
      showNotification('Wi-Fi ayarları kaydediliyor...', 'warning');
      
      fetch('/update_ssid', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `ssid=${encodeURIComponent(ssid)}&password=${encodeURIComponent(password)}`
      })
      .then(response => response.text())
      .then(() => {
        showNotification('Ayarlar kaydedildi! Cihaz yeniden başlatılıyor...', 'success');
        setTimeout(() => location.reload(), 4000);
      })
      .catch(() => {
        showNotification('Ayar kaydetme hatası!', 'error');
      });
    }
  );
}

// Parlaklık güncelleme
function updateBrightness(value) {
  document.getElementById('brightnessValue').textContent = value;
}

function saveBrightness() {
  const brightness = document.getElementById('brightnessSlider').value;
  showNotification(`Parlaklık %${brightness} olarak kaydediliyor...`, 'warning');
  
  fetch('/set_brightness', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `brightness=${brightness}`
  })
  .then(() => showNotification(`Parlaklık %${brightness} olarak kaydedildi!`, 'success'))
  .catch(() => showNotification('Parlaklık ayarlama hatası!', 'error'));
}

// Debug sistemi
function toggleDebug() {
  const currentState = document.getElementById('debugText').textContent === 'Açık';
  const newState = !currentState;
  
  showNotification(`Debug ${newState ? 'açılıyor' : 'kapatılıyor'}...`, 'warning');
  
  fetch('/set_debug', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `enabled=${newState}`
  })
  .then(() => {
    updateDebugUI(newState);
    showNotification(`Debug ${newState ? 'açıldı' : 'kapatıldı'}!`, 'success');
  })
  .catch(() => showNotification('Debug ayarlama hatası!', 'error'));
}

function updateDebugUI(enabled) {
  const icon = document.getElementById('debugIcon');
  const text = document.getElementById('debugText');
  const status = document.getElementById('debugStatus');
  const button = document.getElementById('debugToggle');
  
  if (enabled) {
    icon.textContent = '🟢';
    text.textContent = 'Açık';
    status.textContent = 'Debug mesajları aktif';
    status.style.color = '#27ae60';
    button.className = 'btn btn-success';
  } else {
    icon.textContent = '🔴';
    text.textContent = 'Kapalı';
    status.textContent = 'Debug mesajları kapalı';
    status.style.color = '#e74c3c';
    button.className = 'btn btn-danger';
  }
}

async function loadDebugStatus() {
  try {
    const response = await fetch('/get_debug');
    const enabled = (await response.text()) === 'true';
    updateDebugUI(enabled);
    console.log('✅ Debug durumu yüklendi:', enabled);
  } catch (error) {
    console.error('❌ Debug durumu yüklenemedi:', error);
    updateDebugUI(true); // Hata durumunda varsayılan açık
  }
}

// SSID ayarını yükle
async function loadSSID() {
  try {
    const response = await fetch('/get_ssid');
    const ssid = await response.text();
    
    document.getElementById('ssidInput').value = ssid;
    
    console.log('✅ SSID yüklendi:', ssid);
  } catch (error) {
    console.error('❌ SSID yüklenemedi:', error);
    document.getElementById('ssidInput').placeholder = "SSID yüklenemedi";
  }
}

// ESP-NOW işlemleri
function startPairing() {
  showModal(
    '� Cihaz Tarama Başlat',
    'Çevredeki ESP-NOW cihazları taransın mı? Bu işlem birkaç saniye sürer.',
    function() {
      showNotification('📡 Çevredeki cihazlar taranıyor...', 'warning');
      
      fetch('/start_pairing', { method: 'POST' })
        .then(r => r.text())
        .then(response => {
          showNotification('✅ Cihaz tarama tamamlandı', 'success');
          setTimeout(fetchDevices, 2000); // 2 saniye sonra listeyi güncelle
        })
        .catch(() => showNotification('❌ Tarama başlatma hatası!', 'error'));
    }
  );
}

function addMac() {
  const mac = document.getElementById('macInput').value.trim();
  if (!mac) {
    showNotification('MAC adresi boş olamaz!', 'error');
    return;
  }
  
  // MAC format kontrolü
  const macRegex = /^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$/;
  if (!macRegex.test(mac)) {
    showNotification('Geçersiz MAC adresi formatı! (AA:BB:CC:DD:EE:FF)', 'error');
    return;
  }
  
  showModal(
    '➕ Cihaz Ekle',
    `MAC adresi "${mac}" hafızaya eklenecek. Onaylıyor musunuz?`,
    function() {
      fetch('/add_mac', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `mac=${mac}`
      })
      .then(() => {
        document.getElementById('macInput').value = '';
        showNotification('Cihaz başarıyla eklendi!', 'success');
        fetchDevices();
      })
      .catch(() => showNotification('Cihaz ekleme hatası!', 'error'));
    }
  );
}

function deleteMac(mac) {
  showModal(
    '🗑️ Cihaz Sil',
    `MAC adresi "${mac}" hafızadan silinecek. Bu işlem geri alınamaz!`,
    function() {
      showNotification('Cihaz siliniyor...', 'warning');
      
      fetch('/delete_mac', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `mac=${encodeURIComponent(mac)}`
      })
      .then(response => response.text())
      .then(responseText => {
        console.log('Delete response:', responseText);
        showNotification('✅ Cihaz hafızadan silindi: ' + mac, 'success');
        setTimeout(fetchDevices, 1000); // 1 saniye bekle, sonra listeyi güncelle
      })
      .catch(error => {
        console.error('Delete error:', error);
        showNotification('❌ Cihaz silme hatası!', 'error');
      });
    }
  );
}

function pairMac(mac) {
  console.log('🎯 pairMac çağrıldı, MAC:', mac);
  
  showModal(
    '🤝 Cihaz Eşleştir',
    `MAC adresi "${mac}" ile eşleştirme yapılacak. Devam edilsin mi?`,
    function() {
      console.log('📤 Eşleştirme isteği gönderiliyor...');
      showNotification('Eşleştirme isteği gönderiliyor...', 'warning');
      
      fetch('/pair_request', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `mac=${encodeURIComponent(mac)}`
      })
      .then(response => {
        console.log('✅ Pair request yanıt:', response.status);
        return response.text();
      })
      .then(responseText => {
        console.log('📝 Yanıt metni:', responseText);
        showNotification('✅ Eşleştirme isteği gönderildi!', 'success');
        setTimeout(fetchDevices, 1000);
      })
      .catch(error => {
        console.error('❌ Eşleştirme hatası:', error);
        showNotification('❌ Eşleştirme hatası!', 'error');
      });
    }
  );
}

// ESP-NOW mesajını parse et ve device info güncelle
function parseEspNowMessage(message, mac) {
  // SCAN_RESPONSE format: SCAN_RESPONSE|NODE:HRC_1|MODEL:HRCMINI|VER:v1.0.0
  if (message.startsWith('SCAN_RESPONSE')) {
    const parts = message.split('|');
    let name = 'Unknown', model = 'Unknown', version = 'Unknown';
    
    parts.forEach(part => {
      if (part.startsWith('NODE:')) name = part.substring(5);
      if (part.startsWith('MODEL:')) model = part.substring(6);
      if (part.startsWith('VER:')) version = part.substring(4);
    });
    
    deviceInfo[mac] = {
      name: name,
      model: model,
      rssi: '-',
      version: version,
      lastSeen: new Date().toLocaleTimeString()
    };
    return;
  }
  
  // Format: NAME|MODEL|RSSI|CMD|DATA veya sadece data  
  const parts = message.split('|');
  if (parts.length >= 4) {
    // Yeni format: NAME|MODEL|RSSI|CMD|DATA
    deviceInfo[mac] = {
      name: parts[0],
      model: parts[1], 
      rssi: parts[2],
      lastSeen: new Date().toLocaleTimeString()
    };
  } else {
    // Eski format: sadece data
    if (!deviceInfo[mac]) {
      deviceInfo[mac] = {
        name: 'Unknown',
        model: 'Legacy',
        rssi: '-',
        lastSeen: new Date().toLocaleTimeString()
      };
    }
  }
}

// Cihaz listesini güncelle
async function fetchDevices() {
  try {
    const response = await fetch('/mac_list');
    const { paired, discovered } = await response.json();

    // Eşleşmiş cihazlar tablosu
    const pairedTable = document.getElementById('pairedDevices');
    pairedTable.innerHTML = '';
    
    if (paired.length === 0) {
      pairedTable.innerHTML = '<tr><td colspan="6" style="text-align: center; color: #7f8c8d;">Henüz eşleşmiş cihaz yok</td></tr>';
    } else {
      paired.forEach(device => {
        const row = document.createElement('tr');
        const statusClass = device.active ? 'status-online' : 'status-offline';
        const statusText = device.active ? '🟢 Çevrimiçi' : '🔴 Çevrimdışı';
        const lastSeen = device.lastSeen || 'Bilinmiyor';
        
        // Device info'yu al
        const info = deviceInfo[device.mac] || { name: '-', model: '-', rssi: '-' };
        
        row.innerHTML = `
          <td><code>${device.mac}</code></td>
          <td><strong>${info.name}</strong></td>
          <td><span style="background: #3498db; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px;">${info.model}</span></td>
          <td><span style="color: ${info.rssi !== '-' ? (parseInt(info.rssi) > -50 ? 'green' : parseInt(info.rssi) > -70 ? 'orange' : 'red') : '#7f8c8d'}">${info.rssi !== '-' ? info.rssi + ' dBm' : '-'}</span></td>
          <td><span class="status ${statusClass}">${statusText}</span></td>
          <td>
            <button onclick="deleteMac('${device.mac}')" class="btn btn-small btn-danger">
              <span>🗑️</span> Sil
            </button>
          </td>
        `;
        pairedTable.appendChild(row);
      });
    }

    // Keşfedilen cihazlar tablosu
    const discoveredTable = document.getElementById('discoveredDevices');
    discoveredTable.innerHTML = '';
    
    if (discovered.length === 0) {
      discoveredTable.innerHTML = '<tr><td colspan="6" style="text-align: center; color: #7f8c8d;">Çevrede cihaz bulunamadı</td></tr>';
    } else {
      discovered.forEach(device => {
        const row = document.createElement('tr');
        const signalStrength = device.rssi ? `${device.rssi} dBm` : 'Bilinmiyor';
        
        // Device info'yu al
        const info = deviceInfo[device.mac] || { name: '-', model: '-', rssi: '-' };
        
        // Bu cihaz zaten eşleşmiş mi kontrol et
        const isAlreadyPaired = paired.some(pairedDevice => pairedDevice.mac === device.mac);
        
        const statusHtml = isAlreadyPaired 
          ? '<span class="status status-online">✅ Zaten Eşleşmiş</span>'
          : '<span class="status status-discovered">🆕 Keşfedildi</span>';
          
        const buttonHtml = isAlreadyPaired
          ? '<span style="color: #7f8c8d; font-size: 12px;">Zaten eşleşmiş</span>'
          : `<button onclick="pairMac('${device.mac}')" class="btn btn-small btn-success">
               <span>🤝</span> Eşleştir
             </button>`;
        
        row.innerHTML = `
          <td><code>${device.mac}</code></td>
          <td><strong>${info.name}</strong></td>
          <td><span style="background: #3498db; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px;">${info.model}</span></td>
          <td><span style="color: ${info.rssi !== '-' ? (parseInt(info.rssi) > -50 ? 'green' : parseInt(info.rssi) > -70 ? 'orange' : 'red') : '#7f8c8d'}">${info.rssi !== '-' ? info.rssi + ' dBm' : signalStrength}</span></td>
          <td>${statusHtml}</td>
          <td>${buttonHtml}</td>
        `;
        discoveredTable.appendChild(row);
      });
    }
  } catch (error) {
    console.error('Cihaz listesi alınamadı:', error);
  }
}

// Parlaklık ayarını yükle
async function loadBrightness() {
  try {
    const response = await fetch('/get_brightness');
    const brightness = parseInt(await response.text());
    
    document.getElementById('brightnessSlider').value = brightness;
    document.getElementById('brightnessValue').textContent = brightness;
    
    console.log('✅ Parlaklık yüklendi:', brightness + '%');
  } catch (error) {
    console.error('❌ Parlaklık yüklenemedi:', error);
    // Hata durumunda varsayılan değeri ayarla
    document.getElementById('brightnessSlider').value = 50;
    document.getElementById('brightnessValue').textContent = 50;
  }
}

// Sistem işlemleri
function clearPrefs() {
  showModal(
    '🧹 Hafızayı Temizle',
    'Tüm preferences hafızası silinecek! Bu işlem geri alınamaz. Cihaz yeniden başlayacak.',
    function() {
      showNotification('Hafıza temizleniyor...', 'warning');
      
      fetch('/clear_preferences', { method: 'POST' })
        .then(response => response.text())
        .then(result => {
          showNotification(`${result} Cihaz yeniden başlatılıyor...`, 'success');
          setTimeout(() => location.reload(), 3000);
        })
        .catch(() => showNotification('Hafıza temizleme hatası!', 'error'));
    }
  );
}

function restartDevice() {
  showModal(
    '🔄 Yeniden Başlat',
    'Cihaz yeniden başlatılacak. Bu işlem birkaç saniye sürer.',
    function() {
      showNotification('Cihaz yeniden başlatılıyor...', 'warning');
      
      fetch('/restart', { method: 'POST' })
        .then(() => {
          showNotification('Cihaz yeniden başlatıldı!', 'success');
          setTimeout(() => location.reload(), 5000);
        })
        .catch(() => showNotification('Yeniden başlatma hatası!', 'error'));
    }
  );
}

// Firmware güncelleme onayı
function confirmFirmwareUpdate() {
  const fileInput = document.querySelector('input[type="file"]');
  if (!fileInput.files[0]) {
    showNotification('Lütfen bir firmware dosyası seçin!', 'error');
    return false;
  }
  
  return confirm('⚠️ Firmware güncellemesi başlatılacak! Bu işlem sırasında cihazı kapatmayın. Devam edilsin mi?');
}

// Veri izleme
function startDataMonitoring() {
  dataUpdateInterval = setInterval(fetchDataLogs, 1000);
}

function fetchDataLogs() {
  // Serial veri loglarını al
  fetch('/serial_data')
    .then(response => response.json())
    .then(data => {
      if (data.data && data.data.length > 0) {
        data.data.forEach(entry => addSerialDataLine(entry.message, entry.timestamp));
      }
    })
    .catch(() => {
      // Sessizce hata yakala
    });
  
  // ESP-NOW veri loglarını al
  fetch('/espnow_data')
    .then(response => response.json())
    .then(data => {
      if (data.data && data.data.length > 0) {
        data.data.forEach(entry => {
          addEspnowDataLine(entry.type, entry.message, entry.timestamp);
          
          // Gelen mesajları parse et ve device info güncelle
          if (entry.type === 'in') {
            // "From MAC: message" formatından MAC ve mesajı ayır
            const match = entry.message.match(/From ([A-F0-9:]+): (.+)/);
            if (match) {
              const mac = match[1];
              const message = match[2];
              parseEspNowMessage(message, mac);
            }
          }
        });
      }
    })
    .catch(() => {
      // Sessizce hata yakala
    });
}

function addSerialDataLine(message, timestamp = null) {
  const monitor = document.getElementById('serialMonitor');
  const line = document.createElement('div');
  line.className = 'data-line data-serial';
  
  const time = timestamp || new Date().toLocaleTimeString();
  line.innerHTML = `<span style="opacity:0.7">[${time}]</span> 📥 ${message}`;
  
  monitor.appendChild(line);
  monitor.scrollTop = monitor.scrollHeight;
  
  // Maksimum 30 satır tut
  while (monitor.children.length > 30) {
    monitor.removeChild(monitor.firstChild);
  }
}

function addEspnowDataLine(type, message, timestamp = null) {
  const monitor = document.getElementById('espnowMonitor');
  const line = document.createElement('div');
  
  const cssClass = type === 'in' ? 'data-espnow-in' : 'data-espnow-out';
  const icon = type === 'in' ? '📥' : '📤';
  line.className = `data-line ${cssClass}`;
  
  const time = timestamp || new Date().toLocaleTimeString();
  line.innerHTML = `<span style="opacity:0.7">[${time}]</span> ${icon} ${message}`;
  
  monitor.appendChild(line);
  monitor.scrollTop = monitor.scrollHeight;
  
  // Maksimum 30 satır tut
  while (monitor.children.length > 30) {
    monitor.removeChild(monitor.firstChild);
  }
}

function clearSerialMonitor() {
  document.getElementById('serialMonitor').innerHTML = '<div class="data-line data-serial">Serial monitor temizlendi...</div>';
}

function clearEspnowMonitor() {
  document.getElementById('espnowMonitor').innerHTML = '<div class="data-line data-espnow-in">ESP-NOW monitor temizlendi...</div>';
}

// Ekran test fonksiyonları
function sendTestMessage(type) {
  const message = document.getElementById('testMessage').value.trim();
  if (!message) {
    showNotification('Test mesajı boş olamaz!', 'error');
    return;
  }
  
  if (message.length > 23) {
    showNotification('Mesaj 23 karakterden uzun olamaz!', 'error');
    return;
  }
  
  showNotification(`"${message}" ekrana gönderiliyor...`, 'warning');
  
  fetch('/display_test', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `message=${encodeURIComponent(message)}&type=${type}`
  })
  .then(response => response.text())
  .then(result => {
    showNotification(`Mesaj ekrana gönderildi: ${result}`, 'success');
  })
  .catch(() => {
    showNotification('Mesaj gönderim hatası!', 'error');
  });
}

function clearDisplay() {
  showModal(
    '🧹 Ekranı Temizle',
    'Ekran tamamen temizlenecek. Devam edilsin mi?',
    function() {
      showNotification('Ekran temizleniyor...', 'warning');
      
      fetch('/display_clear', { method: 'POST' })
        .then(response => response.text())
        .then(result => {
          showNotification('Ekran temizlendi!', 'success');
        })
        .catch(() => {
          showNotification('Ekran temizleme hatası!', 'error');
        });
    }
  );
}

// ESP-NOW mesaj gönder
function sendEspnowMessage() {
  const message = document.getElementById('espnowMessage').value.trim();
  if (!message) {
    showNotification('ESP-NOW mesajı boş olamaz!', 'error');
    return;
  }
  
  if (message.length > 50) {
    showNotification('Mesaj 50 karakterden uzun olamaz!', 'error');
    return;
  }
  
  showModal(
    '📡 ESP-NOW Mesaj Gönder',
    `"${message}" mesajı tüm eşleşmiş cihazlara gönderilecek. Onaylıyor musunuz?`,
    function() {
      showNotification('ESP-NOW mesajı gönderiliyor...', 'warning');
      
      fetch('/espnow_send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `message=${encodeURIComponent(message)}`
      })
      .then(response => response.text())
      .then(result => {
        document.getElementById('espnowMessage').value = '';
        showNotification(`ESP-NOW mesajı gönderildi: ${result}`, 'success');
      })
      .catch(() => {
        showNotification('ESP-NOW mesaj gönderim hatası!', 'error');
      });
    }
  );
}

// Modal pencere dışına tıklanınca kapat
document.getElementById('confirmModal').addEventListener('click', function(e) {
  if (e.target === this) closeModal();
});

</script>
</body>
</html>
)rawliteral";

#elif defined(MODBUS_RTU)
const char* upload_html = R"(
	<!DOCTYPE html>
	<html>
	<meta charset='UTF-8'>
	<body>
	  <h2>HRCMINI Firmware Guncelleme</h2>
	  <form method="POST" action="/update" enctype="multipart/form-data">
		<input type="file" name="update">
		<input type="submit" value="Firmware Yukle">
	  </form>
  </body>
  </html>
  )";

#elif defined(HRCMAXI)
const char* upload_html = R"(
	<!DOCTYPE html>
	<html>
	<meta charset='UTF-8'>
	<body>
	  <h2>HRCMAXI Firmware Guncelleme</h2>
	  <form method="POST" action="/update" enctype="multipart/form-data">
		<input type="file" name="update">
		<input type="submit" value="Firmware Yukle">
	  </form>
  </body>
  </html>
  )";

#elif defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
const char* upload_html = R"(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <title>HRCMESAJ Yönetim</title>
  <style>
    body { font-family:sans-serif; background:#f5f5f5; padding:20px; }
    h2 { color:#333; }
    table { width:100%; border-collapse:collapse; margin-top:20px; background:white; box-shadow:0 2px 5px rgba(0,0,0,0.1); }
    th,td { padding:12px; border-bottom:1px solid #ddd; text-align:left; }
    .green { color:green; } .red { color:red; } .gray { color:gray; }
    button { padding:6px 12px; margin-left:10px; }
    .restart-btn { background-color:#dc3545; color:white; border:none; padding:10px 20px; border-radius:5px; cursor:pointer; }
    .restart-btn:hover { background-color:#c82333; }
    input[type=text] { padding:6px; margin-right:10px; width:200px; }
    .fade-out { animation: fadeOut 1s ease forwards; }
    @keyframes fadeOut {
      to { opacity: 0; height: 0; padding: 0; margin: 0; overflow: hidden; }
    }
  </style>
</head>
<body>
<h2>📶 Wi-Fi Adı (SSID) Değiştir</h2>
<form onsubmit="updateSSID(); return false;">
  <input type="text" id="ssidInput" placeholder="Yeni SSID" />
  <button type="submit">Kaydet</button>
</form>

<h2>Yeniden Başlat</h2>
<button onclick='restartESP()' style="background-color:red;color:white;padding:10px;border:none;border-radius:5px">Yeniden Başlat</button>
<div id="restartMessage" style="margin-top:10px;color:green;font-weight:bold;display:none">HRCZAMAN yeniden başlatılıyor...</div>

  <h2>📦 Firmware Güncelle</h2>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="update">
    <input type="submit" value="Yükle">
  </form>
	<h2>HRCMESAJ KOMUTLARI</h2>
	<table border="1" cellpadding="4">
	  <tr><th>Komut</th><th>Gorev</th></tr>
	  <!--<tr><td style="color:black;">satir1yazGÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 1 DE MESAJ GÖSTERİLİR.</td></tr>
	  <tr><td style="color:black;">satir2yazGÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 2 DE MESAJ GÖSTERİLİR.</td></tr>
	  <tr><td style="color:black;">satir3yazGÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 3 DE MESAJ GÖSTERİLİR.</td></tr>
	  <tr><td style="color:black;">satir1sil</td><td style="color:darkred;">SATIR 1 İ SİLER.</td></tr>
	  <tr><td style="color:black;">satir2sil</td><td style="color:darkred;">SATIR 2 Yİ SİLER.</td></tr>
	  <tr><td style="color:black;">satir3sil</td><td style="color:darkred;">SATIR 3 Ü SİLER.</td></tr>
	  <tr><td style="color:black;">cikis1acik</td><td style="color:darkred;">RÖLE 1 AKTİF OLUR.</td></tr>
	  <tr><td style="color:black;">cikis1kapat</td><td style="color:darkred;">RÖLE 1 PASİF OLUR.</td></tr>
	  <tr><td style="color:black;">cikis2acik</td><td style="color:darkred;">RÖLE 2 AKTİF OLUR.</td></tr>
	  <tr><td style="color:black;">cikis2kapat</td><td style="color:darkred;">RÖLE 2 PASİF OLUR.</td></tr>-->
    <tr><td style="color:black;">mesGÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">MESAJ EKRANDA GÖSTERİLİR.</td></tr>
    <tr><td style="color:black;">str1GÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 1 DE MESAJ GÖSTERİLİR.</td></tr>
    <tr><td style="color:black;">str2GÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 2 DE MESAJ GÖSTERİLİR.</td></tr>
    <tr><td style="color:black;">kystr1GÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 1 DE KAYAN MESAJ GÖSTERİLİR.</td></tr>
    <tr><td style="color:black;">kystr2GÖSTERİLECEK MESAJI YAZINIZ</td><td style="color:darkred;">SATIR 2 DE KAYAN MESAJ GÖSTERİLİR.</td></tr>
    <tr><td style="color:black;">sil</td><td style="color:darkred;">TÜM EKRANI SİLER</td></tr>
	</table>
<script>
function updateSSID() {
  const ssid = document.getElementById("ssidInput").value;
  fetch('/update_ssid', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: 'ssid=' + encodeURIComponent(ssid)
  }).then(() => {
    alert('SSID saved, restarting...');
    setTimeout(() => location.reload(), 3000);
  });
}

function restartESP() {
  document.getElementById('restartMessage').style.display = 'block';
  fetch('/restart', {
    method: 'POST'
  }).then(() => {
    alert('HRCZAMAN Yeniden Başlatılıyor...');
    setTimeout(() => location.reload(), 5000);
  }).catch(() => {
    alert('HRCZAMAN Yeniden Başlatılıyor...');
    setTimeout(() => location.reload(), 5000);
  });
}
</script>
</body>
</html>

  )";
#elif defined(SERITOUSB)

const char* upload_html = R"(
	<!DOCTYPE html>
	<html>
	<meta charset='UTF-8'>
	<body>
	  <h2>SERiALTOWiFi Firmware Guncelleme</h2>
	  <form method="POST" action="/update" enctype="multipart/form-data">
		<input type="file" name="update">
		<input type="submit" value="Firmware Yukle">
	  </form>
  </body>
  </html>
  )";

#elif defined(LEADER)

const char* upload_html = R"(
	<!DOCTYPE html>
	<html>
	<meta charset='UTF-8'>
	<body>
	  <h2>LEADER PRİNTER Firmware Guncelleme</h2>
	  <form method="POST" action="/update" enctype="multipart/form-data">
		<input type="file" name="update">
		<input type="submit" value="Firmware Yukle">
	  </form>
  </body>
  </html>
  )";
#elif defined(ESPNOW)

const char* upload_html = R"(
	<!DOCTYPE html>
	<html>
	<meta charset='UTF-8'>
	<body>
	  <h2>LEADER PRİNTER Firmware Guncelleme</h2>
	  <form method="POST" action="/update" enctype="multipart/form-data">
		<input type="file" name="update">
		<input type="submit" value="Firmware Yukle">
	  </form>
  </body>
  </html>
  )";
#endif

void SaveSSID(const String& ssid) {
  DEBUG_PRINTF("💾 SSID kaydediliyor: '%s'\n", ssid.c_str());
  
  preferences.begin("wifi", false);
  size_t written = preferences.putString("ssid", ssid);
  preferences.end();
  
  DEBUG_PRINTF("✅ SSID kaydedildi, %zu bytes yazıldı\n", written);
  
  // Kayıt kontrolü
  preferences.begin("wifi", true);
  String verify = preferences.getString("ssid", "KAYIT_BASARISIZ");
  preferences.end();
  
  DEBUG_PRINTF("🔍 Kayıt kontrolü: '%s'\n", verify.c_str());
}

void SavePassword(const String& password) {
  preferences.begin("wifi", false);
  preferences.putString("password", password);
  preferences.end();
}

// Gizli AP modu baslat ve IP adresini 192.168.4.1 olarak ayarla
void startHiddenAP() {

  DEBUG_PRINTLN("🔍 WiFi preferences kontrol ediliyor...");
  
  preferences.begin("wifi", true);
  
  // Preferences debug
  size_t ssidLen = preferences.getBytesLength("ssid");
  size_t passLen = preferences.getBytesLength("password");
  DEBUG_PRINTF("📊 Preferences boyutları - SSID: %zu bytes, Password: %zu bytes\n", ssidLen, passLen);
  
  String savedSSID = preferences.getString("ssid", "HRC_DEFAULT");
  String savedPassword = preferences.getString("password", "teraziwifi");
  preferences.end();
  
  DEBUG_PRINTF("🌐 AP Başlatılıyor - SSID: '%s' (len: %d), Şifre: '%s' (len: %d)\n", 
                savedSSID.c_str(), savedSSID.length(),
                savedPassword.c_str(), savedPassword.length());
  
  // Eğer varsayılan değer dönüyorsa manuel kontrol
  if (savedSSID == "HRC_DEFAULT") {
    DEBUG_PRINTLN("⚠️ Varsayılan SSID kullanılıyor, preferences boş olabilir!");
  }
  // Gizli (SSID yayınlamayan) bir Access Point baslatma
  //WiFi.softAP(ssid, password, 1, 1);  // Gizli mod aktif

  IPAddress local_IP(192, 168, 4, 1);  // Sabit IP adresi
  IPAddress gateway(192, 168, 4, 1);   // Ag gecidi (gateway)
  IPAddress subnet(255, 255, 255, 0);  // Alt ag maskesi

  while (!WiFi.mode(WIFI_AP_STA)) {
	Serial.println("AP-STA MOD BASARISIZ!");
  }
  
  while (!WiFi.softAP(savedSSID.c_str(), savedPassword.c_str())) {
    Serial.println("AP-STA SSID BASARISIZ!");
    return;
  }
  
  DEBUG_PRINTF("✅ AP başarıyla oluşturuldu - SSID: '%s'\n", savedSSID.c_str());

  // AP-STA IP yapılandırmasını ayarla
  while (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP-STA KOFIGURASYON BASARISIZ!");
    return;
  }
  
  DEBUG_PRINTF("✅ AP IP yapılandırması tamamlandı: %s\n", WiFi.softAPIP().toString().c_str());

  // Sabit IP adresini seri monitore yazdır
  //DEBUG_PRINTLN("Gizli AP baslatıldı!");
  //DEBUG_PRINT("AP IP Adresi: ");
  //Serial.println(WiFi.softAPIP());
  
  // TX Power ayarla - WiFi AP için de maksimum güç
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Maksimum güç
  //DEBUG_PRINTLN("📡 WiFi AP TX Power ayarlandi: 19.5 dBm");
}

#ifdef ESPNOW

void updatePeerStatus(const uint8_t *mac_addr, bool active) {
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  for (int i = 0; i < peerStatusCount; i++) {
    if (memcmp(peerStatusList[i].mac, mac_addr, 6) == 0) {
      if (peerStatusList[i].active != active) {
        DEBUG_PRINTF("📊 Peer status değişti: %s -> %s\n", macStr, active ? "ACTIVE" : "INACTIVE");
      }
      peerStatusList[i].active = active;
      return;
    }
  }

  if (peerStatusCount < MAX_PAIRED_DEVICES) {
    memcpy(peerStatusList[peerStatusCount].mac, mac_addr, 6);
    peerStatusList[peerStatusCount].active = active;
    peerStatusCount++;
    DEBUG_PRINTF("📊 Yeni peer eklendi: %s -> %s\n", macStr, active ? "ACTIVE" : "INACTIVE");
  }
}

// Aktif cihaz sayısını hesapla
int getActiveDeviceCount() {
  int activeCount = 0;
  for (int i = 0; i < peerStatusCount; i++) {
    if (peerStatusList[i].active) {
      activeCount++;
    }
  }
  return activeCount;
}

void LoadPairedMac() {
  preferences.begin("espnow", true);
  int macDataLength = preferences.getBytesLength("paired_mac");
  pairedDeviceCount = macDataLength / 6;
  if (pairedDeviceCount > MAX_PAIRED_DEVICES) pairedDeviceCount = MAX_PAIRED_DEVICES;
  if (pairedDeviceCount > 0) {
    preferences.getBytes("paired_mac", pairedMacList, macDataLength);
    isPaired = true;
    for (int i = 0; i < pairedDeviceCount; i++) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, pairedMacList[i], 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      esp_now_add_peer(&peerInfo);
      updatePeerStatus(pairedMacList[i], false);
    }
  } else {
    isPaired = false;
  }
  preferences.end();
}

void RemovePairedMac(const char* macStr) {
  DEBUG_PRINTF("🔍 MAC silme işlemi başlıyor: %s\n", macStr);
  
  // Önce preferences'tan güncel verileri oku
  preferences.begin("espnow", false);
  pairedDeviceCount = preferences.getBytesLength("paired_mac") / 6;
  if (pairedDeviceCount > MAX_PAIRED_DEVICES) pairedDeviceCount = MAX_PAIRED_DEVICES;
  preferences.getBytes("paired_mac", pairedMacList, pairedDeviceCount * 6);
  preferences.end();
  
  DEBUG_PRINTF("🔢 Preferences'tan okunan cihaz sayısı: %d\r\n", pairedDeviceCount);
  
  // Mevcut MAC listesini göster
  DEBUG_PRINTLN("📋 Mevcut eşleşmiş MAC'ler:");
  for (int i = 0; i < pairedDeviceCount; i++) {
    DEBUG_PRINTF("  %d: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  i, pairedMacList[i][0], pairedMacList[i][1], pairedMacList[i][2],
                  pairedMacList[i][3], pairedMacList[i][4], pairedMacList[i][5]);
  }
  
  uint8_t mac[6];
  String macString = String(macStr);
  DEBUG_PRINTF("📝 Gelen MAC string: %s\r\n", macStr);
  
  // Manuel parsing - daha güvenilir
  String macParts[6];
  int partIndex = 0;
  int startPos = 0;
  
  // ':' ile ayır
  for (int i = 0; i <= (int)macString.length() && partIndex < 6; i++) {
    if (i == (int)macString.length() || macString.charAt(i) == ':') {
      if (i > startPos) {
        macParts[partIndex] = macString.substring(startPos, i);
        partIndex++;
      }
      startPos = i + 1;
    }
  }
  
  if (partIndex != 6) {
    Serial.printf("❌ MAC formatı hatalı: %s (parts: %d)\r\n", macStr, partIndex);
    return;
  }
  
  // Her parçayı hex'e çevir
  for (int i = 0; i < 6; i++) {
    mac[i] = (uint8_t)strtol(macParts[i].c_str(), NULL, 16);
  }
         
  DEBUG_PRINTF("🎯 Parse edilmiş MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  int index = -1;
  for (int i = 0; i < pairedDeviceCount; i++) {
    if (memcmp(pairedMacList[i], mac, 6) == 0) {
      index = i;
      break;
    }
  }

  if (index >= 0) {
    DEBUG_PRINTF("✅ MAC bulundu, index: %d\r\n", index);
    // listedeki öğeyi kaydırarak sil
    for (int i = index; i < pairedDeviceCount - 1; i++) {
      memcpy(pairedMacList[i], pairedMacList[i + 1], 6);
    }
    pairedDeviceCount--;

    // preferences’a gerçekten yaz
    preferences.begin("espnow", false);
    if (pairedDeviceCount > 0) {
      preferences.putBytes("paired_mac", pairedMacList, pairedDeviceCount * 6);
    } else {
      preferences.remove("paired_mac");  // hepsi silindiyse key'i kaldır
    }
    preferences.end();

    // RAM'den ve esp-now'dan da çıkar
    esp_err_t result = esp_now_del_peer(mac);
    DEBUG_PRINTF("🗑️ ESP-NOW peer silme sonucu: %s\r\n", result == ESP_OK ? "Başarılı" : "Hata");
    DEBUG_PRINTF("✅ MAC başarıyla silindi. Yeni sayı: %d\r\n", pairedDeviceCount);
  } else {
    DEBUG_PRINTF("⚠️ MAC adresi listede bulunamadı: %s\r\n", macStr);
  }
}

void addToDiscoveredList(const uint8_t *mac_addr) {
  // MAC adresini debug için yazdır
  DEBUG_PRINT("🔍 Discovered listesine eklenmeye çalışılan MAC: ");
  for (int i = 0; i < 6; i++) {
    DEBUG_PRINTF("%02X", mac_addr[i]);
    if (i < 5) DEBUG_PRINT(":");
  }
  Serial.println();
  
  // Sadece zaten discovered listesinde var mı kontrol et (paired olup olmaması önemli değil)
  for (int i = 0; i < discoveredCount; i++) {
    if (memcmp(discoveredMacList[i], mac_addr, 6) == 0) {
      DEBUG_PRINTLN("ℹ️ MAC zaten discovered listesinde");
      return;
    }
  }
  
  if (discoveredCount < MAX_PAIRED_DEVICES) {
    memcpy(discoveredMacList[discoveredCount], mac_addr, 6);
    discoveredCount++;
    DEBUG_PRINTF("✅ Yeni cihaz discovered listesine eklendi. Toplam: %d\r\n", discoveredCount);
  } else {
    DEBUG_PRINTLN("⚠️ Discovered listesi dolu!");
  }
}

void removeFromDiscoveredList(const uint8_t *mac_addr) {
  int index = -1;
  for (int i = 0; i < discoveredCount; i++) {
    if (memcmp(discoveredMacList[i], mac_addr, 6) == 0) {
      index = i;
      break;
    }
  }
  
  if (index >= 0) {
    // Listedeki öğeyi kaydırarak sil
    for (int i = index; i < discoveredCount - 1; i++) {
      memcpy(discoveredMacList[i], discoveredMacList[i + 1], 6);
    }
    discoveredCount--;
    DEBUG_PRINTF("🗑️ MAC discovered listesinden çıkarıldı. Yeni toplam: %d\r\n", discoveredCount);
  }
}

void StartPairing() {
  //DEBUG_PRINTLN("Cihaz Tarama Baslatildi...");
  const char *scanMessage = "DEVICE_SCAN";

  // Broadcast adresine tarama mesajı gönder
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)scanMessage, strlen(scanMessage));
  if (result == ESP_OK) {
    DEBUG_PRINTLN("📡 Cihaz tarama başlatıldı.");
    // Discovered listesini temizle
    discoveredCount = 0;
  } else {
    DEBUG_PRINTLN("⚠️ Cihaz tarama başlatılamadı.");
#ifdef HRCMINI
    ShowOnDisplay("TARAMA HATASI");
#endif
#ifdef HRCNANO
    display.message = "TARAMA HATASI";
	display.scrollTextHorizontal(200);
#endif
  }
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void SavePairedMac(const uint8_t *newMac) {
  // MAC adresini debug için yazdır
  DEBUG_PRINT("💾 Kaydetmeye çalışılan MAC: ");
  for (int i = 0; i < 6; i++) {
    DEBUG_PRINTF("%02X", newMac[i]);
    if (i < 5) DEBUG_PRINT(":");
  }
  Serial.println();
  
  preferences.begin("espnow", false);
  pairedDeviceCount = preferences.getBytesLength("paired_mac") / 6;
  if (pairedDeviceCount > MAX_PAIRED_DEVICES) pairedDeviceCount = MAX_PAIRED_DEVICES;
  preferences.getBytes("paired_mac", pairedMacList, pairedDeviceCount * 6);
  
  Serial.printf("📊 Mevcut eşleşmiş cihaz sayısı (kaydetmeden önce): %d\r\n", pairedDeviceCount);

  bool alreadyExists = false;
  for (int i = 0; i < pairedDeviceCount; i++) {
    if (memcmp(pairedMacList[i], newMac, 6) == 0) {
      alreadyExists = true;
      break;
    }
  }

  if (!alreadyExists) {
    if (pairedDeviceCount < MAX_PAIRED_DEVICES) {
      memcpy(pairedMacList[pairedDeviceCount], newMac, 6);
      pairedDeviceCount++;
      DEBUG_PRINTF("✅ Yeni MAC eklendi. Yeni toplam: %d\r\n", pairedDeviceCount);
    } else {
      for (int i = 0; i < MAX_PAIRED_DEVICES - 1; i++) {
        memcpy(pairedMacList[i], pairedMacList[i + 1], 6);
      }
      memcpy(pairedMacList[MAX_PAIRED_DEVICES - 1], newMac, 6);
      DEBUG_PRINTLN("🔄 Liste dolu, en eskisi silindi, yenisi eklendi");
    }
  } else {
    DEBUG_PRINTLN("ℹ️ MAC zaten kayıtlı");
  }

  preferences.putBytes("paired_mac", pairedMacList, pairedDeviceCount * 6);
  preferences.end();
  
  // Kaydedilen MAC'leri listele
  DEBUG_PRINTLN("📋 Kaydedilmiş MAC listesi:");
  for (int i = 0; i < pairedDeviceCount; i++) {
    DEBUG_PRINTF("  %d: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  i, pairedMacList[i][0], pairedMacList[i][1], pairedMacList[i][2],
                  pairedMacList[i][3], pairedMacList[i][4], pairedMacList[i][5]);
  }

  updatePeerStatus(newMac, true);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, newMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(newMac)) {
    esp_now_add_peer(&peerInfo);
  }

  //DEBUG_PRINTLN("✅ Eşleşmiş MAC adresi kaydedildi.");
}

void PrintMacAddress(const uint8_t *mac) {
  DEBUG_PRINT("MAC Adresi: ");
  for (int i = 0; i < 6; i++) {
    DEBUG_PRINTF("%02X", mac[i]);
    if (i < 5) DEBUG_PRINT(":");
  }
  Serial.println();
}

String macToStr(const uint8_t *mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void SendPairRequest(const uint8_t *mac_addr) {
  DEBUG_PRINT("🎯 PAIR_REQUEST gönderilecek MAC: ");
  PrintMacAddress(mac_addr);
  Serial.println();
  
  // Hedef cihazın peer olarak ekli olup olmadığını kontrol et
  if (!esp_now_is_peer_exist(mac_addr)) {
    DEBUG_PRINTLN("⚠️ Hedef cihaz peer listesinde yok, ekleniyor...");
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    esp_err_t addResult = esp_now_add_peer(&peerInfo);
    if (addResult == ESP_OK) {
      DEBUG_PRINTLN("✅ Hedef cihaz peer olarak eklendi");
    } else {
      DEBUG_PRINTF("❌ Peer ekleme hatası: %d\r\n", addResult);
      return;
    }
  } else {
    DEBUG_PRINTLN("ℹ️ Hedef cihaz zaten peer listesinde");
  }
  
  const char *pairMessage = "PAIR_REQUEST";
  esp_err_t result = esp_now_send(mac_addr, (uint8_t *)pairMessage, strlen(pairMessage));
  
  if (result == ESP_OK) {
    DEBUG_PRINTLN("✅ PAIR_REQUEST başarıyla gönderildi");
  } else {
    DEBUG_PRINTF("❌ PAIR_REQUEST gönderilemedi, hata kodu: %d\r\n", result);
  }
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

// Otomatik eşleştirme (buton ile)
void StartAutoPairing() {
  DEBUG_PRINTLN("🤖 Otomatik eşleştirme başlatıldı");
  
#ifdef HRCMINI
  ShowOnDisplay("ESLESME...");
#endif

  // Önce keşfedilen listeyi temizle
  discoveredCount = 0;
  
  // Cihaz taraması başlat
  const char *scanMessage = "DEVICE_SCAN";
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)scanMessage, strlen(scanMessage));
  
  if (result == ESP_OK) {
    DEBUG_PRINTLN("📡 Otomatik tarama başlatıldı");
    
    // LED yanıp sönsün (tarama göstergesi)
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
    
    // 3 saniye bekle (cihazların yanıt vermesi için)
    delay(3000);
    
    // İlk bulunan cihazla eşleştirme yap
    if (discoveredCount > 0) {
      DEBUG_PRINTF("✅ %d cihaz bulundu, ilki ile eşleştiriliyor\n", discoveredCount);
      
      // İlk cihazın MAC adresini al
      uint8_t targetMAC[6];
      memcpy(targetMAC, discoveredMacList[0], 6);
      
      // Eşleştirme isteği gönder
      SendPairRequest(targetMAC);
      
      // MAC adresini kaydet
      SavePairedMac(targetMAC);
      
#ifdef HRCMINI
      ShowOnDisplay("ESLESTI!");
#endif
      
      // Başarı LED'i
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      
    } else {
      DEBUG_PRINTLN("❌ Hiç cihaz bulunamadı");
      
#ifdef HRCMINI
      ShowOnDisplay("CIHAZ YOK");
#endif
      
      // Hata LED'i
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
      }
    }
  } else {
    DEBUG_PRINTLN("❌ Otomatik tarama başlatılamadı");
    
#ifdef HRCMINI
    ShowOnDisplay("HATA");
#endif
  }
}

void AddPeer(const uint8_t *mac_addr) {
  if (esp_now_is_peer_exist(mac_addr)) {
    //DEBUG_PRINTLN("ℹ️ Peer zaten kayıtlı.");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac_addr, 6);
  peerInfo.channel = 0;         // aynı kanal
  peerInfo.encrypt = false;     // şifreleme kullanmıyoruz

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK) {
    DEBUG_PRINT("✅ Peer eklendi: ");
    PrintMacAddress(mac_addr);
  } else {
    DEBUG_PRINTF("❌ Peer eklenemedi! Hata kodu: %d\n", result);
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  hata_timer = millis();
  char receivedData[len + 1];
  memcpy(receivedData, data, len);
  receivedData[len] = '\0';
  
  // Tüm gelen mesajları debug için yazdır
  /*DEBUG_PRINT("📨 ESP-NOW mesaj alındı: [");
  for (int i = 0; i < 6; i++) {
    DEBUG_PRINTF("%02X", mac_addr[i]);
    if (i < 5) DEBUG_PRINT(":");
  }
  DEBUG_PRINTF("] -> \"%s\"\n", receivedData);
  
  // Debug: Gelen mesajı logla
  DEBUG_PRINTF("📥 OnDataRecv: MAC: %02X:%02X:%02X:%02X:%02X:%02X, Data: %s\n",
                mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                receivedData);*/
  
  // Web monitörü için ESP-NOW gelen veriyi kaydet
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  addEspnowData("From " + String(macStr) + ": " + String(receivedData), "in");

  if (strcmp(receivedData, "DEVICE_SCAN") == 0) {
    // Cihaz tarama mesajı alındı - kendini broadcast ile tanıt
    preferences.begin("wifi", true);
    String deviceName = preferences.getString("ssid", "HRCMINI");
    preferences.end();
    
    String response = "SCAN_RESPONSE|NODE:" + deviceName + "|MODEL:" + String(MODEL) + "|VER:v1.0.0";
    
    // Broadcast adresine gönder (tüm cihazlar alsın)
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)response.c_str(), response.length());
    
    if (result == ESP_OK) {
      DEBUG_PRINTLN("📤 Tarama yanıtı broadcast gönderildi: " + deviceName);
    } else {
      Serial.printf("❌ Tarama yanıtı gönderilemedi: %s (error: %d)\n", deviceName.c_str(), result);
    }
  }
  else if (strcmp(receivedData, "PAIR_REQUEST") == 0) {
    //DEBUG_PRINTLN("Eslesme Istegi Alindi!");
    
    // SSID'yi doğru namespace'den oku
    preferences.begin("wifi", true);
    String deviceName = preferences.getString("ssid", "HRCMINI");
    preferences.end();
    
    String response = "PAIR_RESPONSE|NODE:" + deviceName + "|MODEL:" + String(MODEL) + "|VER:v1.0.0";
	esp_now_send(mac_addr, (uint8_t *)response.c_str(), response.length());
  isPaired = true;
	SavePairedMac(mac_addr);
	AddPeer(mac_addr);
#ifdef HRCMINI
	ShowOnDisplay("BAGLANDI!");
	sonformattedText = "";
#endif
    return;
  }

  else if (strcmp(receivedData, "PAIR_DEL") == 0) {
  //DEBUG_PRINTLN("🧹 Pair silme komutu alındı!");
  String macStr = macToStr(mac_addr);
  RemovePairedMac(macStr.c_str());
}
  else if (strncmp(receivedData, "SCAN_RESPONSE", 13) == 0) {
    Serial.println("📡 Tarama yanıtı alındı: " + String(receivedData));
    DEBUG_PRINTF("🎯 SCAN_RESPONSE için addToDiscoveredList çağrılıyor: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    // Sadece SCAN_RESPONSE mesajları için discovered listesine ekle
    addToDiscoveredList(mac_addr);
    return;
  }
  else if (strncmp(receivedData, "PAIR_RESPONSE", 13) == 0) {
    Serial.println("🤝 Eşleştirme yanıtı alındı: " + String(receivedData));
    isPaired = true;
	  SavePairedMac(mac_addr);
	  PrintMacAddress(mac_addr);
#ifdef HRCMINI
	  ShowOnDisplay("BAGLANDI!");
	  sonformattedText = "";
#endif
    return;
  }

  else if (strcmp(receivedData, "inddara") == 0) {
    digitalWrite(4,LOW);
	  delay(1000);
	  digitalWrite(4,HIGH);
  }

#ifdef MODBUS_RTU
  else if (strcmp(receivedData, "sifir") == 0) {
    //DEBUG_PRINTLN("SIFIR!");
	  //digitalWrite(15,LOW);
	  //delay(1000);
	  //digitalWrite(15,HIGH);
	  long signedValue = 0;
	  uint32_t combined = (uint32_t)signedValue; // long -> unsigned 32 bit
	  uint16_t hiWord = (combined >> 16) & 0xFFFF;
	  uint16_t loWord = (combined & 0xFFFF);
  
	  // Transmit buffer'a yazmak istediğimiz veriyi koy
	  node.setTransmitBuffer(0, hiWord);
	  node.setTransmitBuffer(1, loWord);
  
	  // 0x00 adresinden başla, 2 adet register yaz
	  uint8_t result = node.writeMultipleRegisters(0x00, 2);  
	  if (result == node.ku8MBSuccess) {
	    DEBUG_PRINTLN("Yazma basarili!");
	  } else {
	    DEBUG_PRINT("Yazma Hatasi, Kod = ");
	    DEBUG_PRINTLN(result, HEX);
	  }
  }
#endif

  else {
    // Normal veri mesajları için sadece eşleşmiş cihazlardan kabul et
    bool isPairedDevice = false;
    for (int i = 0; i < pairedDeviceCount; i++) {
      if (memcmp(pairedMacList[i], mac_addr, 6) == 0) {
        isPairedDevice = true;
        break;
      }
    }
    
    if (!isPairedDevice) {
      Serial.println("⚠️ Eşleşmemiş cihazdan veri, reddedildi: " + String(receivedData));
      return;
    }
    
    // Yeni format denemesi: NAME|MODEL|RSSI|CMD|DATA
    EspNowMessage parsedMsg;
    
    if (parseEspNowMessage(receivedData, len, parsedMsg)) {
      // Yeni format kullanılıyor
      Serial.println("✅ ESP-NOW: " + parsedMsg.name + " (" + parsedMsg.model + ") RSSI:" + String(parsedMsg.rssi) + " -> " + parsedMsg.data);
      
      // Ekranda sadece DATA kısmını göster
      String displayText = parsedMsg.data;
      
#ifdef HRCMINI
      String formattedText = displayText + "(" + ") ";
#else
      String formattedText = displayText;
#endif
      
#ifdef HRCMINI
      if (formattedText != sonformattedText) {
        ShowOnDisplay(formattedText);
        sonformattedText = formattedText;
      }
#endif

#if defined(HRCMESAJ) || defined(HRCMAXI)
      String ekran_goster;
      if (formattedText != sonformattedText) {
        ekran_goster = formattedText + "(" + "> ";
        sonformattedText = formattedText;
      } else {
        ekran_goster = formattedText + "(" + ") ";
      }
      // HRCMESAJ/HRCMAXI için display kodu
      dmd.drawString(0, 0, ekran_goster.c_str(), ekran_goster.length(), GRAPHICS_NORMAL);
#endif

#ifdef HRCMESAJ_RGB
      virtualDisp->clearScreen();
      virtualDisp->setFont();
      virtualDisp->setTextSize(1);
      virtualDisp->setTextWrap(true);
      virtualDisp->setCursor(0, 1);
      virtualDisp->setTextColor(virtualDisp->color565(255, 100, 100));
      virtualDisp->print("TANK 1");
      virtualDisp->setCursor(43, 1);
      virtualDisp->setTextColor(virtualDisp->color565(100, 255, 100));
      virtualDisp->print(" TARTIM");
      virtualDisp->setCursor(86, 1);
      virtualDisp->setTextColor(virtualDisp->color565(100, 100, 255));
      virtualDisp->print(" ONAYLANDI");
      virtualDisp->setFont(&FreeSansBold12pt7b); 
      virtualDisp->setTextColor(virtualDisp->color565(255,255,255));
      virtualDisp->setCursor(0,31);
      virtualDisp->print(formattedText);
#endif

#ifdef HRCNANO
      String ekran_goster;
      if (formattedText != sonformattedText) {
        ekran_goster = formattedText + "(" + "> ";
        sonformattedText = formattedText;
      } else {
        ekran_goster = formattedText + "(" + ") ";
      }
      display.message = ekran_goster;
      display.BreakTextInFrames(20);
#endif
      
    } else {
      // Eski format (sadece data) - | olmayan basit string
      Serial.println("📊 ESP-NOW Eski Format: " + String(receivedData));
      
#ifdef HRCMINI
      String formattedText = String(receivedData) + "(" + ") ";
#else
      String formattedText = String(receivedData);
#endif
#ifdef HRCMINI
      if (formattedText != sonformattedText) {
        if (formattedText != sonformattedText) {
          ShowOnDisplay(formattedText);
          sonformattedText = formattedText;
        }
      }
#endif
#if defined(HRCMESAJ) || defined(HRCMAXI)
      String ekran_goster;
      if (formattedText != sonformattedText) {
        ekran_goster = formattedText + "(" + "> ";
        sonformattedText = formattedText;
      } else {
        ekran_goster = formattedText + "(" + ") ";
      }
      dmd.drawString(0, 0, ekran_goster.c_str(), ekran_goster.length(), GRAPHICS_NORMAL);
#endif
#ifdef HRCMESAJ_RGB
      virtualDisp->clearScreen();
      virtualDisp->setFont();
      virtualDisp->setTextSize(1);
      virtualDisp->setTextWrap(true);
      virtualDisp->setCursor(0, 1);
      virtualDisp->setTextColor(virtualDisp->color565(255, 100, 100));
      virtualDisp->print("TANK 1");
      virtualDisp->setCursor(43, 1);
      virtualDisp->setTextColor(virtualDisp->color565(100, 255, 100));
      virtualDisp->print(" TARTIM");
      virtualDisp->setCursor(86, 1);
      virtualDisp->setTextColor(virtualDisp->color565(100, 100, 255));
      virtualDisp->print(" ONAYLANDI");
      virtualDisp->setFont(&FreeSansBold12pt7b); 
      virtualDisp->setTextColor(virtualDisp->color565(255,255,255));
      virtualDisp->setCursor(0,31);
      virtualDisp->print(formattedText);
#endif
#ifdef HRCNANO
      String ekran_goster;
      if (formattedText != sonformattedText) {
        ekran_goster = formattedText + "(" + "> ";
        sonformattedText = formattedText;
      } else {
        ekran_goster = formattedText + "(" + ") ";
      }
      display.message = ekran_goster;
      display.BreakTextInFrames(20);
#endif
    }
    // Gelen veriyi sabit olarak LED ekrana yazdır
    // display.displayText(receivedData.c_str(), PA_CENTER, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT); // Sabit pozisyon belirle
    hata = false;
    digitalWrite(LED_PIN, HIGH);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
updatePeerStatus(mac_addr, status == ESP_NOW_SEND_SUCCESS);
}

void SendData(String data) {
  if (!isPaired || pairedDeviceCount == 0) return;

  for (int i = 0; i < pairedDeviceCount; i++) {
    // Web monitörü için ESP-NOW giden veriyi kaydet
    String macStr = macToStr(pairedMacList[i]);
    addEspnowData("To " + macStr + ": " + data, "out");
    
    // Yeni format: NAME|MODEL|RSSI|CMD|DATA
    int rssi = WiFi.RSSI(); // WiFi RSSI al
    if (rssi == 0) rssi = lastReceivedRSSI; // ESP-NOW modunda last received RSSI kullan
    
    // SSID'yi doğru namespace'den oku
    preferences.begin("wifi", true);
    String deviceName = preferences.getString("ssid", "HRCMINI"); // SSID'yi NAME olarak kullan
    preferences.end();
    
    //DEBUG_PRINTLN("🔍 Debug - SSID: " + deviceName); // Debug için
    String formattedData = deviceName + "|" + MODEL + "|" + String(rssi) + "|CMD|" + data;
    
    esp_err_t result = esp_now_send(pairedMacList[i], (uint8_t *)formattedData.c_str(), formattedData.length());
    updatePeerStatus(pairedMacList[i], result == ESP_OK);
    
    DEBUG_PRINTLN("📤 ESP-NOW Sent: " + formattedData);
  }
}
#endif

// Web sunucusunu baslat
void startWebServer() {
  server.on("/update_ssid", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("ssid", true)) {
      String newSSID = request->getParam("ssid", true)->value();
      String password = "";
      if (request->hasParam("password", true)) {
        password = request->getParam("password", true)->value();
      }
      SaveSSID(newSSID);
      if (password.length() > 0) {
        SavePassword(password);
      }
      request->send(200, "text/plain", "Wi-Fi ayarları kaydedildi");
      ESP.restart();
    } else {
      request->send(400, "text/plain", "SSID parametresi eksik");
    }
  });

  // Parlaklık ayarı endpoint'i
  server.on("/set_brightness", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("brightness", true)) {
      int brightness = request->getParam("brightness", true)->value().toInt();
      if (brightness >= 0 && brightness <= 100) {
        #ifdef HRCMINI
        // MD_Parola için parlaklık ayarı (0-15 arası)
        uint8_t intensity = map(brightness, 0, 100, 0, 15);
        display.setIntensity(intensity);
        #endif
        
        // Parlaklığı preferences'a kaydet
        preferences.begin("settings", false);
        preferences.putInt("brightness", brightness);
        preferences.end();
        DEBUG_PRINTF("💾 Parlaklık kaydedildi: %d%%\n", brightness);
        
        request->send(200, "text/plain", "Parlaklık ayarlandı");
      } else {
        request->send(400, "text/plain", "Geçersiz parlaklık değeri (0-100)");
      }
    } else {
      request->send(400, "text/plain", "Parlaklık parametresi eksik");
    }
  });

  // Parlaklık okuma endpoint'i
  server.on("/get_brightness", HTTP_GET, [](AsyncWebServerRequest *request){
    preferences.begin("settings", true);
    int savedBrightness = preferences.getInt("brightness", 50); // Varsayılan %50
    preferences.end();
    DEBUG_PRINTF("📖 Parlaklık okundu: %d%%\n", savedBrightness);
    request->send(200, "text/plain", String(savedBrightness));
  });

  // SSID okuma endpoint'i
  server.on("/get_ssid", HTTP_GET, [](AsyncWebServerRequest *request){
    preferences.begin("wifi", true);
    String savedSSID = preferences.getString("ssid", "HRC_DEFAULT");
    preferences.end();
    DEBUG_PRINTF("📖 SSID okundu: '%s'\n", savedSSID.c_str());
    request->send(200, "text/plain", savedSSID);
  });

  // Debug ayarları endpoint'leri
  server.on("/set_debug", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("enabled", true)) {
      String enabledStr = request->getParam("enabled", true)->value();
      debugEnabled = (enabledStr == "true");
      
      // Debug durumunu preferences'a kaydet
      preferences.begin("settings", false);
      preferences.putBool("debug", debugEnabled);
      preferences.end();
      
      DEBUG_PRINTF("🐛 Debug %s\n", debugEnabled ? "açıldı" : "kapatıldı");
      request->send(200, "text/plain", debugEnabled ? "Debug açık" : "Debug kapalı");
    } else {
      request->send(400, "text/plain", "Enabled parametresi eksik");
    }
  });

  server.on("/get_debug", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", debugEnabled ? "true" : "false");
  });

  // Serial veri logları endpoint
  server.on("/serial_data", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(2048);
    JsonArray data = doc.createNestedArray("data");
    
    // Serial data buffer'ını JSON'a çevir
    for(int i = 0; i < MAX_DATA_ENTRIES; i++) {
      int index = (serialDataIndex + i) % MAX_DATA_ENTRIES;
      if(serialDataBuffer[index].message.length() > 0) {
        JsonObject entry = data.createNestedObject();
        entry["message"] = serialDataBuffer[index].message;
        entry["timestamp"] = String(serialDataBuffer[index].timestamp);
      }
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // ESP-NOW veri logları endpoint
  server.on("/espnow_data", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(2048);
    JsonArray data = doc.createNestedArray("data");
    
    // ESP-NOW data buffer'ını JSON'a çevir
    for(int i = 0; i < MAX_DATA_ENTRIES; i++) {
      int index = (espnowDataIndex + i) % MAX_DATA_ENTRIES;
      if(espnowDataBuffer[index].message.length() > 0) {
        JsonObject entry = data.createNestedObject();
        entry["message"] = espnowDataBuffer[index].message;
        entry["type"] = espnowDataBuffer[index].type;
        entry["timestamp"] = String(espnowDataBuffer[index].timestamp);
      }
    }
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Ekran test endpoint'leri
  server.on("/display_test", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("message", true) || !request->hasParam("type", true)) {
      request->send(400, "text/plain", "Parametreler eksik");
      return;
    }
    
    String message = request->getParam("message", true)->value();
    String type = request->getParam("type", true)->value();
    
    if (message.length() > 23) {
      request->send(400, "text/plain", "Mesaj çok uzun (max 23)");
      return;
    }
    
    #ifdef HRCMINI
    if (type == "scroll") {
      ShowOnDisplay(message.c_str()); // Kayan yazı için ShowOnDisplay kullan
      request->send(200, "text/plain", "Kayan mesaj gönderildi");
    } else {
      // Normal statik gösterim için direkt display kullan
      display.displayClear();
      display.print(message.c_str());
      request->send(200, "text/plain", "Normal mesaj gönderildi");
    }
    #else
    request->send(200, "text/plain", "Test mesajı gönderildi (display yok)");
    #endif
  });

  server.on("/display_clear", HTTP_POST, [](AsyncWebServerRequest *request){
    #ifdef HRCMINI
    display.displayClear();
    request->send(200, "text/plain", "Ekran temizlendi");
    #else
    request->send(200, "text/plain", "Ekran temizlendi (display yok)");
    #endif
  });

  server.on("/espnow_send", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("message", true)) {
      request->send(400, "text/plain", "Mesaj parametresi eksik");
      return;
    }
    
    String message = request->getParam("message", true)->value();
    if (message.length() > 50) {
      request->send(400, "text/plain", "Mesaj çok uzun (max 50)");
      return;
    }
    
    #ifdef ESPNOW
    int sentCount = 0;
    for (int i = 0; i < pairedDeviceCount; i++) {
      esp_err_t result = esp_now_send(pairedMacList[i], (uint8_t*)message.c_str(), message.length());
      if (result == ESP_OK) {
        sentCount++;
      }
    }
    
    // ESP-NOW giden veri loguna ekle
    char logMessage[100];
    snprintf(logMessage, sizeof(logMessage), "[OUT] %s (%d cihaz)", message.c_str(), sentCount);
    addEspnowData(logMessage, "OUT");
    
    request->send(200, "text/plain", String("Mesaj ") + sentCount + " cihaza gönderildi");
    #else
    request->send(200, "text/plain", "ESP-NOW aktif değil");
    #endif
  });

  server.on("/restart", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "HRCZAMAN yeniden başlatılıyor...");
    delay(1000);
    ESP.restart();
  });

  server.on("/clear_preferences", HTTP_POST, [](AsyncWebServerRequest *request){
    Preferences preferences;
    preferences.begin("espnow", false);
    preferences.clear();  // tüm key'leri sil
    preferences.end();
    request->send(200, "text/plain", "✅ Preferences temizlendi");
    delay(1000);
    ESP.restart();  // İsteğe bağlı: cihazı yeniden başlat
  });

  server.on("/start_pairing", HTTP_POST, [](AsyncWebServerRequest *request){
    #ifdef ESPNOW
    StartPairing();  // Broadcast ile eşleşme başlat
    #endif
    request->send(200, "text/plain", "Eşleşme başlatıldı!");
  });

  #ifdef ESPNOW
  server.on("/mac_list", HTTP_GET, [](AsyncWebServerRequest *request){
    //DEBUG_PRINTF("🌐 /mac_list çağrıldı. Paired: %d, Discovered: %d\n", pairedDeviceCount, discoveredCount);
    
    DynamicJsonDocument doc(2048);

    // ✅ Güncel eşleşen cihazlar
    JsonArray paired = doc.createNestedArray("paired");
    for (int i = 0; i < pairedDeviceCount; i++) {
      JsonObject obj = paired.createNestedObject();
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            pairedMacList[i][0], pairedMacList[i][1], pairedMacList[i][2],
            pairedMacList[i][3], pairedMacList[i][4], pairedMacList[i][5]);
      obj["mac"] = macStr;

      // ❗️peerStatusList boyut kontrolü ile eşleşme durumu
      bool aktif = false;
      for (int j = 0; j < peerStatusCount; j++) {
        if (memcmp(peerStatusList[j].mac, pairedMacList[i], 6) == 0) {
          aktif = peerStatusList[j].active;
          break;
        }
      }
      obj["active"] = aktif;
    }

    // ✅ Çevrede keşfedilen cihazlar
    JsonArray discovered = doc.createNestedArray("discovered");
    for (int i = 0; i < discoveredCount; i++) {
      JsonObject obj = discovered.createNestedObject();
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            discoveredMacList[i][0], discoveredMacList[i][1], discoveredMacList[i][2],
            discoveredMacList[i][3], discoveredMacList[i][4], discoveredMacList[i][5]);
      obj["mac"] = macStr;
    }

    // ✅ JSON çıktıyı hazırla ve gönder
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/add_mac", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("mac", true)) {
      String macStr = request->getParam("mac", true)->value();
      macStr.toUpperCase(); // Büyük harfe çevir
      uint8_t mac[6];
      sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
             &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
      SavePairedMac(mac);
    }
    request->send(200);
  });

  server.on("/delete_mac", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("mac", true)) {
      String macStr = request->getParam("mac", true)->value();
      DEBUG_PRINTLN("🗑️ MAC silme isteği: " + macStr);
      RemovePairedMac(macStr.c_str());
      request->send(200, "text/plain", "MAC silindi: " + macStr);
    } else {
      DEBUG_PRINTLN("❌ MAC parametresi eksik");
      request->send(400, "text/plain", "MAC parametresi eksik");
    }
  });

  server.on("/pair_request", HTTP_POST, [](AsyncWebServerRequest *request){
    DEBUG_PRINTLN("🌐 /pair_request endpoint çağrıldı");
    
    if (!request->hasParam("mac", true)) {
      DEBUG_PRINTLN("❌ MAC parametresi eksik");
      request->send(400, "text/plain", "MAC eksik");
      return;
    }

    String macStr = request->getParam("mac", true)->value();
    DEBUG_PRINTLN("🎯 Eşleştirme isteği MAC: " + macStr);
    
    uint8_t mac[6];
    // Manuel parsing - daha güvenilir
    String macParts[6];
    int partIndex = 0;
    int startPos = 0;
    
    // ':' ile ayır
    for (int i = 0; i <= (int)macStr.length() && partIndex < 6; i++) {
      if (i == (int)macStr.length() || macStr.charAt(i) == ':') {
        if (i > startPos) {
          macParts[partIndex] = macStr.substring(startPos, i);
          partIndex++;
        }
        startPos = i + 1;
      }
    }
    
    if (partIndex != 6) {
      Serial.printf("❌ MAC formatı hatalı: %s (parts: %d)\n", macStr.c_str(), partIndex);
      request->send(400, "text/plain", "MAC formatı hatalı");
      return;
    }
    
    // Her parçayı hex'e çevir
    for (int i = 0; i < 6; i++) {
      mac[i] = (uint8_t)strtol(macParts[i].c_str(), NULL, 16);
    }
    
    DEBUG_PRINTF("✅ MAC başarıyla parse edildi: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    DEBUG_PRINTLN("📤 SendPairRequest çağrılıyor...");
    // PAIR_REQUEST gönder
    SendPairRequest(mac);
    
    // MAC adresini eşleşmiş cihazlar listesine ekle
    SavePairedMac(mac);
    
    // Discovered listesinden çıkar (artık eşleşmiş)
    removeFromDiscoveredList(mac);
    
    Serial.printf("🤝 Eşleştirme işlemi başlatıldı: %s\n", macStr.c_str());
    request->send(200, "text/plain", "Eşleştirme isteği gönderildi ve kaydedildi");
  });
  #endif
  
  // Firmware guncelleme sayfası
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", upload_html);
  });
  // Firmware yukleme islemi
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", (Update.hasError()) ? "Yukleme Hatasi!" : "Yukleme Basarili!");
    ESP.restart();
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
#if defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
      dmd.drawString(0,  0, "GUNCELLEME ", 11, GRAPHICS_NORMAL); 
      dmd.drawString(0, 16, "BASLADI. ", 9, GRAPHICS_NORMAL);
      delay(2000);
      pauseDMD();
#endif
      Serial.printf("Yukleme Basladı: %s\n", filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    }
    if (!Update.hasError()) {
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
      }
    }
    if (final) {
      if (Update.end(true)) {
#if defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
        resumeDMD();
        dmd.drawString(0,  0, "YUKLEME       ", 14, GRAPHICS_NORMAL);
        dmd.drawString(0, 16, "TAMAMLANDI.   ", 14, GRAPHICS_NORMAL);
#endif
        delay(2000);
        Serial.printf("Yukleme Tamamlandı: %s\n", filename.c_str());
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
  //DEBUG_PRINTLN("Web sunucusu baslatıldı.");
}

// WiFi ve Web sunucusunu farklı bir task'ta calıstırma
void wifiTask(void * parameter) {
#if defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
  pauseDMD();  // WiFi islemleri sırasında DMD taramasını durdur
#endif
  startHiddenAP();  // Gizli AP'yi baslat
#if defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
  resumeDMD();  // WiFi islemleri tamamlandı, DMD taramasını yeniden baslat
#endif  
  startWebServer();  // Web sunucusunu baslat
  vTaskDelete(NULL);  // Gorevi sonlandır
}

String terazi_data;
const uint8_t *data_son;

//#if defined(HRCMINI) || defined(SERITOUSB)
//#endif
void FormatNumericData(String &numericData, char *output) {
    // Boşlukları temizle
    numericData.trim();
    
    // Noktanın yerini ve verinin uzunlugunu hesapla
    int dotIndex = numericData.indexOf('.');
    int length = numericData.length();

    // Float sayıyı dogru bir sekilde ayıkla
    float value = numericData.toFloat();

    // Formatlanmıs cıktıyı olustur
    if (dotIndex == -1) {
        // Nokta yoksa tam sayı olarak formatla
        snprintf(output, 6, "%5.0f", value);
    } else {
        // Nokta varsa ondalık basamaga gore formatla
        int decimalPlaces = length - dotIndex - 1;
        if (decimalPlaces == 0) {
            snprintf(output, 7, "%6.0f", value);
        } else if (decimalPlaces == 1) {
            snprintf(output, 7, "%6.1f", value);
        } else if (decimalPlaces == 2) {
            snprintf(output, 7, "%6.2f", value);
		    } else if (decimalPlaces == 3) {
			  // numericData string uzunluğunu kontrol et
			    if ((value > 99.999) || (value < -99.999)) { // 6. karakter varsa (5'ten büyükse)
              snprintf(output, 9, "%7.3f ", value);
			    }
			    else
            snprintf(output, 7, "%6.3f", value);
        } else if (decimalPlaces == 4) {
            snprintf(output, 7, "%6.4f", value);
		}
  }
}

String ExtractNumericData(String data) {
	String numericData = "";
	bool decimalPointSeen = false; // Ondalık nokta kontrolu
	for (int i = 0; i < data.length(); i++) {
	  if (isdigit(data[i]) || (data[i] == '-' && numericData.length() == 0)) {
		numericData += data[i];
	  } else if (data[i] == '.' && !decimalPointSeen) {
		numericData += data[i];
		decimalPointSeen = true;
	  }
	}
	return numericData;
}

// Serial port veri işleme fonksiyonu (optimize edilmiş)
void processSerialData(Stream& serialPort, const String& portName, bool enableESPNOW = true) {
  if (!serialPort.available()) return;
  
  char output[9]; // 6 karakter + null terminator için
  String data = serialPort.readStringUntil('\n');
  
  // Web monitörü için veriyi kaydet
  if (data.length() > 0) {
    addSerialData(portName + ": " + data.substring(0, min((int)data.length(), 50)));
  }
  
  String numericData = ExtractNumericData(data);
  FormatNumericData(numericData, output);
  
  // Minimum veri uzunluğu kontrolü
  if (data.length() > 5) {
    hata_timer = millis(); // HATA timer'ını sıfırla
    
    #ifdef ESPNOW
    if (enableESPNOW) {
      SendData(output);
    }
    #endif
    
    //Serial.println("Terazi Datası : " + data + " Ekran Datası : " + String(output));
    addSerialData("Processed: " + String(output));
    
    #ifdef HRCMINI
    String formattedText = numericData + "(" + ")";
    if (formattedText != sonformattedText) {
      ShowOnDisplay(formattedText);
      sonformattedText = formattedText;
    }
    #endif
    
    hata = false;
  }
  
  serialPort.flush();
}

#ifdef HRCMESAJ
void processModbusData() {
  //DEBUG_PRINT("Gelen Veri: ");
  DEBUG_PRINTLN(receivedData); // Gelen veriyi ASCII olarak yazdır

  // Tartım verisini ayıkla
  if (receivedData[0] == ':' && strlen(receivedData) >= 13) {
    // Tartım verisi 7. ve 8. indeksten baslıyor
    char tartimHex[5] = {receivedData[7], receivedData[8], receivedData[9], receivedData[10], '\0'};
    int tartimDecimal = strtol(tartimHex, NULL, 16); // Hex'ten Decimal'e cevir
    //Serial.print("Tartım Bilgisi (Decimal): ");
	char formattedTartim[10]; // Formatlanmıs veri icin buffer
	snprintf(formattedTartim, sizeof(formattedTartim), "%6d()  ", tartimDecimal);
	DEBUG_PRINTLN(formattedTartim);
	// Formatlanmıs veriyi ekrana yazdır
	dmd.drawString(0, 0, formattedTartim, (strlen(formattedTartim)+2), GRAPHICS_NORMAL);
  } else {
    DEBUG_PRINTLN("Gecersiz Veri");
  }
}
#endif

#ifdef LEADER

// Küçük Font – sola hizalı
const uint8_t sol[] = {
  27, 'a', 0,     // sola hizala
};

// Orta Font – ortalanmış
const uint8_t orta[] = {
  27, 'a', 1,     // ortala
};

// Büyük Font – sağa hizalı
const uint8_t sag[] = {
  27, 'a', 2,     // sağa hizala
};

// Küçük font – normal genişlik, 2x yükseklik
const uint8_t kucuk_font[] = {
  27, 77, 49,     // ESC M 1 → Font B (küçük font)
  29, 33, 0x01    // GS ! 0x01 → 2x yükseklik (bit 4)
};

// Orta font – normal font, 2x yükseklik
const uint8_t orta_font[] = {
  27, 77, 48,     // ESC M 0 → Font A (standart)
  29, 33, 0x01    // GS ! 0x01 → 2x yükseklik
};

// Büyük font – küçük font, 2x genişlik + 2x yükseklik
const uint8_t buyuk_font[] = {
  27, 77, 49,     // ESC M 1 → Font B (küçük font)
  29, 33, 0x11    // GS ! 0x11 → genişlik 2x + yükseklik 2x
};

// En büyük font – standart font, genişlik 4x + yükseklik 4x
const uint8_t enbuyuk_font[] = {
  27, 77, 48,     // ESC M 0 → Font A (standart)
  29, 33, 0x33    // GS ! 0x33 → genişlik 4x + yükseklik 4x
};

const uint8_t beep[] = {
  0x1B, 0x42, 8, 1, 0x0D
};

void yazicikomut(const uint8_t* font, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial1.write(font[i]);
  }
}

void printToEscPos(String data) {
  // ESC/POS başlangıcı
  //Serial1.write(0x1B); Serial1.write('@');         // initialize
  //Serial1.write(0x1B); Serial1.write('a'); Serial1.write(1); // center

  // 🔍 Örnek substring kullanımı:
  //int start = data.indexOf(0x0D,0x0A);
  //int end = data.indexOf("\r", start);
  //if (start != -1 && end != -1) {
    //String pi = data.substring(9, 34);
	yazicikomut(orta_font, sizeof(orta_font));
	yazicikomut(orta, sizeof(orta));
  Serial1.println(data.substring(9, 36));
	Serial1.println(data.substring(36, 61));
	Serial1.println(data.substring(61, 86));
	Serial1.println(data.substring(86, 111));
	Serial1.println(data.substring(111, 136));
	yazicikomut(sol, sizeof(sol));
	Serial1.println();
	Serial1.println("TARiH : "+ data.substring(160, 168));
	Serial1.println("SAAT  : "+ data.substring(151, 159));
	Serial1.println("FiS NO: "+ data.substring(182, 188));
	yazicikomut(buyuk_font, sizeof(buyuk_font));
	Serial1.println("--------------------");
	Serial1.println("\nTOPLAM KG "+ data.substring(228, 238));
	yazicikomut(orta_font, sizeof(orta_font));
	Serial1.println();
	yazicikomut(sol, sizeof(sol));
	Serial1.println("\nDARA KG : "+ data.substring(241, 247));
	yazicikomut(buyuk_font, sizeof(buyuk_font));
	Serial1.println("--------------------");
	Serial1.println("\n\n\n");
  //}

  // Tüm veri istersen direkt bastır:
  //Serial1.println(data);

  Serial1.write(0x0A); // boşluk satırı
  Serial1.write(0x1D); Serial1.write('V'); Serial1.write(1); // cut
}
#endif

/*--------------------------------------------------------------------------------------
  setup
  Called by the Arduino architecture before the main loop begins
--------------------------------------------------------------------------------------*/

void setup(void) {
#ifdef HRCMESAJ_RGB
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X *2,  // 128
    PANEL_RES_Y/2,     // 32
    2                // 2 panel
  );

  mxconfig.gpio.r1  = 25;
  mxconfig.gpio.g1  = 26;
  mxconfig.gpio.b1  = 27;
  mxconfig.gpio.r2  = 14;
  mxconfig.gpio.g2  = 12;
  mxconfig.gpio.b2  = 13;
  mxconfig.gpio.a   = 23;
  mxconfig.gpio.b   = 22;
  mxconfig.gpio.c   = 21;
  mxconfig.gpio.lat = 5;
  mxconfig.gpio.oe  = 15;
  mxconfig.gpio.clk = 18;

  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_10M;
  mxconfig.clkphase = false;
  mxconfig.latch_blanking = 1;
  mxconfig.min_refresh_rate = 500;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(80);
  dma_display->clearScreen();

  virtualDisp = new VirtualMatrixPanel_T<PANEL_CHAIN_TYPE, MyScanTypeMapping>(
    VDISP_NUM_ROWS,
    VDISP_NUM_COLS,
    PANEL_RES_X,
    PANEL_RES_Y
  );
  virtualDisp->setDisplay(*dma_display);

  // Temiz ekran
  virtualDisp->fillScreen(virtualDisp->color565(0,0,0));
  delay(500);
#endif
#ifdef HRCNANO
  // Donanım pinlerini ayarla
  display.setPins(pins);
    // Ekranı yapılandır: tip 0 = 64x16, 4 panel varsa (8x4 = 32 kolon)
  display.setDisplay(0, 1);  // 8 panel = 64x16 ekran

  // Timer başlat (ESP32 için turnOn içinde yapılmalı)
  display.turnOn();
  display.message = "HRCNANO "; // Sabit pozisyon belirle
  display.BreakTextInFrames(2000);
  display.message = "  BILTER"; // Sabit pozisyon belirle
  display.BreakTextInFrames(2000);
  display.clear_buffer();
#endif
#ifdef MODBUS_RTU
Serial1.begin(9600, SERIAL_8N1, 17, 16);
node.begin(1, Serial1);  // SlaveID=1, Serial=Hardware Serial

//Serial1.begin(9600, SERIAL_8N1, 17, 16);
//mb.begin(&Serial1);
//mb.master();
#endif
#ifdef LEADER
Serial.begin(1200, SERIAL_8N1);
Serial1.begin(115200, SERIAL_8N1, 17, 16);
  //startHiddenAP();
  //startWebServer();
  xTaskCreatePinnedToCore(
	wifiTask,        // Gorevin adı
	"WiFi Task",     // Gorev acıklaması
	8192,            // Yıgın boyutu
	NULL,            // Parametre
	2,               // Öncelik
	NULL,            // Gorev tutucu
	0                // Çekirdek (0 veya 1)
);
#elif defined(SERITOUSB)
  Serial.begin(9600, SERIAL_7N1);
  Serial2.begin(9600, SERIAL_8N1, 14, 12);
  //startHiddenAP();
  //startWebServer();
  xTaskCreatePinnedToCore(
	wifiTask,        // Gorevin adı
	"WiFi Task",     // Gorev acıklaması
	8192,            // Yıgın boyutu
	NULL,            // Parametre
	2,               // Öncelik
	NULL,            // Gorev tutucu
	0                // Çekirdek (0 veya 1)
);
#else
  Serial.begin(9600);
#endif

#ifdef HRCMESAJ
  pinMode(out1_pin, OUTPUT);
  pinMode(out2_pin, OUTPUT);
  digitalWrite(out1_pin, HIGH);
  digitalWrite(out2_pin, HIGH);
#endif

#if defined(HRCMESAJ) || defined(HRCMAXI) || defined(HRCZAMAN)
  // Timer baslatma - Eski API (ESP32 Arduino Framework 2.x)
  timer = timerBegin(0, 80, true);  // Timer 0, 80 prescaler, count up
  timerAttachInterrupt(timer, &triggerScan, true);
  timerAlarmWrite(timer, 300, true);  // 300µs interrupt
  timerAlarmEnable(timer);

  // DMD ekranı temizle
  dmd.clearScreen(true);
  dmd.selectFont(Segfont_Sayi_Harf);
  xTaskCreatePinnedToCore(
    wifiTask,        // Gorevin adı
    "WiFi Task",     // Gorev acıklaması
    8192,            // Yıgın boyutu
    NULL,            // Parametre
    2,               // Öncelik
    NULL,            // Gorev tutucu
    0                // Çekirdek (0 veya 1)
  );
#if defined(HRCMESAJ)
  dmd.drawString(0, 0, "HRCMESAJ ", 8, GRAPHICS_NORMAL);
#elif defined(HRCMAXI)
  dmd.drawString(0, 0, " HRCMAXI ", 8, GRAPHICS_NORMAL);
  //delay(2000);
  //dmd.clearScreen(true);
  //dmd.selectFont(Segfont_7x16);
  //dmd.selectFont(Arial_Black_16);
      // Watchdog islemlerini bir task icinde calıstır

	//dmd.drawString(0, 0, "   CAS   ", 8, GRAPHICS_NORMAL);
#elif defined(HRCZAMAN)  
	//dmd.drawString(0, 0, " ENDUTEK ", 8, GRAPHICS_NORMAL);
#endif
	//dmd.drawString(0, 0, "   MEGA  ", 8, GRAPHICS_NORMAL);
	//delay(2000);
  dmd.clearScreen(true);
#ifdef MODBUS_RTU
	modbusSerial.begin(19200, SERIAL_7E1, 5, 16);
#endif
#endif
#ifdef HRCMINI
  Serial.begin(9600); // Debug için normal Serial
  Serial1.begin(9600, SERIAL_8N1, 17, 16); // Harici cihaz için Serial1
  display.begin();
  
  // Buton pinini ayarla
  pinMode(PAIR_BUTTON, INPUT_PULLUP);
  
  // Kaydedilmiş ayarları yükle
  preferences.begin("settings", false);
  int savedBrightness = preferences.getInt("brightness", 50); // Varsayılan %50
  debugEnabled = preferences.getBool("debug", true); // Varsayılan açık
  preferences.end(); // ÖNEMLİ: Preferences'ı kapat
  
  uint8_t intensity = map(savedBrightness, 0, 100, 0, 15);
  display.setIntensity(intensity); // Parlaklık ayarı (0 - 15 arası)
  display.displayClear();  // Ekranı temizleme
  display.setFont(dotmatrix_5x8);
  
  DEBUG_PRINTF("🐛 Debug sistemi %s\n", debugEnabled ? "AÇIK" : "KAPALI");
  DEBUG_PRINTF("✅ Parlaklık yüklendi: %d%% (intensity: %d)\n", savedBrightness, intensity);
  
  startupMessageTimer = millis();
#endif
#ifdef ESPNOW
  pinMode(PAIR_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  startHiddenAP();
  startWebServer();
  // ESP-NOW baslat
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("ESP-NOW Baslatilamadi!");
#ifdef HRCMINI
#endif
#ifdef HRCNANO
	display.message = "BAGLANTI YOK"; // Sabit pozisyon belirle
	display.scrollTextHorizontal(200);
#endif
    return;
  } else {
    //DEBUG_PRINTLN("ESP-NOW Baslatildi!");
    
    // TX Power ayarla - maksimum güç (20 dBm = 100mW)
    WiFi.setTxPower(WIFI_POWER_19_5dBm); // Maksimum güç
    //DEBUG_PRINTLN("📡 ESP-NOW TX Power ayarlandi: 19.5 dBm");
    
    // Startup sequence loop'ta başlatılacak
  }
  // Daha once eslesmis cihaz var mı kontrol et
  LoadPairedMac();

if (!isPaired && !preferences.getBytesLength("paired_mac")) {
    DEBUG_PRINTLN("Eslesmis Cihaz Yok. Eslesme Bekleniyor.");
#ifdef HRCMINI
    ShowOnDisplay("ESLESME YOK..");
#endif
#ifdef HRCNANO
	display.message = "ESLESME YOK.."; // Sabit pozisyon belirle
	display.scrollTextHorizontal(200);
#endif
    Serial1_mod = true; 
} else {
    //DEBUG_PRINTLN("Eslesmis cihaz bulundu:");
    PrintMacAddress(pairedMacList[0]);
    
    // 'i' degiskenini 0 olarak baslat ve kayıtlı cihaz sayısına gore dongu yap
    int maxDeviceCount = sizeof(pairedMacList) / sizeof(pairedMacList[0]);
    for (int i = 0; i < pairedDeviceCount && i < maxDeviceCount; i++) {
        if (pairedMacList[i] != nullptr) { // Gecerli MAC adresi kontrolu
            //AddPeer(pairedMacList[i]); // Eslesmis cihazı peer olarak ekle
            DEBUG_PRINTF("Peer eklendi: Cihaz %d\n", i);
        } else {
            DEBUG_PRINTF("Gecersiz MAC adresi: Cihaz %d\n", i);
        }
    }
#ifdef HRCNANO
	display.message = "BAGLANTI KURULUYOR.."; // Sabit pozisyon belirle
	display.scrollTextHorizontal(200);
#endif
}

  // Gelen veri icin callback fonksiyonu
  esp_now_register_recv_cb(OnDataRecv);
  // Gonderim durumu icin callback fonksiyonu
  esp_now_register_send_cb(OnDataSent);

  // Add peer        
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //DEBUG_PRINTLN("Peer Eklenemedi!");
#ifdef HRCMINI
    ShowOnDisplay("PEER ERR");
#endif
  } else {
    DEBUG_PRINTLN("Peer Basariyla Eklendi.");
  }
#endif
#ifdef HRCMINI
ShowOnDisplay("HRCMINI");
delay(2000);
ShowOnDisplay(" BILTER");
delay(3000);

// Startup tamamlandı
startupComplete = true;
hata_timer = millis(); // HATA timer'ını başlat
hata = false;
#endif
#ifdef HRCZAMAN
  dmd.selectFont(System5x7_ENDUTEK);
  preferences.begin("wifi", true);
  dmd.drawString(0, 10, preferences.getString("ssid").c_str(), 14, GRAPHICS_NORMAL);
  dmd.drawString(0, 42, preferences.getString("ssid").c_str(), 14, GRAPHICS_NORMAL);
  preferences.end();  
#endif
#if defined(HRCMAXI) || defined(HRCZAMAN) || defined(HRCMESAJ)
  delay(2000);
  dmd.clearScreen(true);
#endif

}

/*--------------------------------------------------------------------------------------
  loop
  Arduino architecture main loop
--------------------------------------------------------------------------------------*/
char veri[25];
unsigned long startMillis = millis();  // Baslangıc zamanı

#define BUFFER_SIZE 256 // Gelen veri icin maksimum buffer boyutu

String serialData = "";
unsigned long lastCharTime = 0;
unsigned long timeoutDuration = 200; // 2 saniye
bool beepYapildi = false;

void loop(void) {
#ifdef LEADER
  // 1. Veri geldikçe string'e ekle
  while (Serial.available()) {
    if (!beepYapildi) {
      yazicikomut(beep, sizeof(beep));  // 🔔 İlk karakterde beep
      beepYapildi = true;
    }

    char c = Serial.read();
    serialData += c;
    lastCharTime = millis(); // veri geldi, zamanı güncelle
  }

  // 2. Zaman kontrolü (2 saniye boyunca veri gelmediyse)
  if (serialData.length() > 0 && millis() - lastCharTime > timeoutDuration) {
    printToEscPos(serialData);
    serialData = "";
    beepYapildi = false;  // ⏪ Sonraki paket için sıfırla
  }
#endif
#ifdef SERITOUSB
while (Serial2.available()) {
    String inByte = Serial2.readStringUntil('\n');
    float terazi_data = inByte.substring(0,7).toFloat() * 1000.0;
    char terazi_data_c[20];
    snprintf(terazi_data_c, sizeof(terazi_data_c), "%8.1f", terazi_data);
    String stabil_data = "ST,GS,+";
    Serial.println(stabil_data + String(terazi_data_c) + String(F(" g   "))); // String(terazi_data_c)  //ST,GS,+  236.6 g   CRLF
	//DEBUG_PRINT(inByte);
	Serial2.flush();
}
      
#endif 

#if defined(HRCMESAJ) || defined(HRCMAXI)
  // Gelen veriyi kontrol et
  if (millis() - hata_timer > 5000) {
	hata = true;
	digitalWrite(LED_PIN, LOW);
  } else {
	hata = false;
  }
#ifdef MODBUS_RTU
  // handleSerialInput();  // Seri porttan gelen verileri isle

  // Modbus ASCII sorgusunu byte array olarak tanımlıyoruz
  uint8_t modbusRequest[] = {
    0x3A,0x30,0x31,0x30,0x33,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x31,0x46,0x42,0x0D,0x0A
  };
  // Modbus sorgusunu gonder
   modbusSerial.write(modbusRequest, sizeof(modbusRequest));
  //modbusDEBUG_PRINT(":010300000001FB\r\n");
  while (modbusSerial.available()) {
	startMillis = millis() + 5000;
    char incomingChar = modbusSerial.read(); // Gelen karakteri oku
    if (incomingChar == ':') {
      dataIndex = 0; // Yeni veri basladıgında buffer sıfırla
    }

    // Buffer'a gelen veriyi ekle
    receivedData[dataIndex++] = incomingChar;

    // Sonlandırıcı karaktere ulasıldıgında
    if (incomingChar == '\n') {
      receivedData[dataIndex] = '\0'; // Buffer'a null karakter ekle
      processModbusData();           // Gelen veriyi isle
      dataIndex = 0;                 // İndeksi sıfırla
    }
  }
  if (millis() > startMillis) {
	dmd.drawString(0,0,"HATA RS  ",9,GRAPHICS_NORMAL);
  }
  delay(100);
#elif defined(HRCMAXI)
  // Gelen veriyi kontrol et
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    //String data = Serial.readStringUntil('\r');
    if (data.length() > 0) {
      hata_timer = millis();
      String numericData = ExtractNumericData(data); // Sayısal kısmı ayıkla
      char output[9]; // 6 karakter + null terminator icin
      FormatNumericData(numericData, output); // Formatla
      if (String(output).length() > 0) {
        dmd.drawString(0, 0, numericData.c_str(), numericData.length(), GRAPHICS_NORMAL);
      }
    }
    Serial.flush();
  }
if (Serial.available()) {
	char output[9]; // 6 karakter + null terminator icin
    String data = Serial.readStringUntil('\n');
	  //String data = Serial.readStringUntil('\r');
    String numericData = ExtractNumericData(data); // Sayısal kısmı ayıkla
	  FormatNumericData(numericData, output); // Formatla
    if (String(output).length() > 0) {
      hata_timer = millis();
	  String formattedText = numericData + "(" + ")";
	  dmd.drawString(0, 0, formattedText.c_str(), formattedText.length(), GRAPHICS_NORMAL);
    }
    Serial.flush();
  }
  if (millis() > hata_timer + 5000) {
	  dmd.drawString(0,0,"HATA WF  ",9,GRAPHICS_NORMAL);
  }
  //delay(100);
#endif
#endif
#if defined(HRCZAMAN) || defined(HRCMESAJ_RGB) || defined(HRCMESAJ)
  handleSerialData();
#endif
#ifdef ESPNOW
 if (digitalRead(PAIR_BUTTON) == LOW) { // Buton basılı mı?
    if (!buttonPressed) { // İlk defa mı algılandı?
      buttonPressStartTime = millis(); // Basılma baslangıc zamanını kaydet
      buttonPressed = true;
    } else if (millis() - buttonPressStartTime >= 3000) { // 3 saniye gecti mi?
      StartAutoPairing(); // 3 saniye boyunca basılı tutulduysa otomatik eslesme baslat
      buttonPressed = false; // İslem tamam, tekrar tetiklenmesini engelle
    }
  } else { // Buton bırakıldıgında
    buttonPressed = false; // Durumu sıfırla
  }
#endif

#if defined(HRCMINI) || defined(HRCNANO)
  // Optimize edilmiş serial port işleme
  #ifdef ESPNOW
  processSerialData(Serial, "Serial", isPaired); // Serial ESP-NOW ile çalışsın sadece eşleşmişse
  #else
  processSerialData(Serial, "Serial", false); // ESP-NOW devre dışı
  #endif
  
  processSerialData(Serial1, "Serial1", true); // Serial1 her zaman ESP-NOW ile çalışsın

  if (((millis() - hata_timer) > 8000) && (!hata)) {
    // 8 saniye veri gelmezse HATA göster
    #ifdef HRCMINI
    ShowOnDisplay("HATA");
    #endif
    hata = true;
	  sonformattedText = "";
    digitalWrite(LED_PIN, LOW);
  }
 
#endif

#ifdef MODBUS_RTU
long signedValue;

uint8_t result = node.readHoldingRegisters(0x00, 2);
if (result == node.ku8MBSuccess) {
    // Başarılı ise bufferdan verileri çek
    uint16_t val1 = node.getResponseBuffer(0);  // ilk register değeri
    uint16_t val2 = node.getResponseBuffer(1);  // ikinci register değeri

	uint32_t combined = ((uint32_t)val1 << 16) | val2;
	signedValue = (long)combined;

    DEBUG_PRINT("Register[0x10] = ");
    DEBUG_PRINT(val1);
    DEBUG_PRINT("   Register[0x11] = ");
    DEBUG_PRINTLN(val2);
  } else {
    // Hata kodu
    DEBUG_PRINT("Modbus read error, code: ");
    DEBUG_PRINTLN(result, HEX);
  }

/*if (!mb.slave()) {
  mb.readHreg(SLAVE_ID, FIRST_REG, res, 2, cb);
  while (mb.slave()) {
    mb.task();
    delay(10);
  }

  uint32_t combined = ((uint32_t)res[0] << 16) | res[1];
  long signedValue = (long)combined;

  Serial.print("Long (invert): ");
  DEBUG_PRINTLN(signedValue);*/

#ifdef ESPNOW
  SendData(String(signedValue));
#endif
 //}
 #endif
#ifdef HRCNANO
//display.BreakTextInFrames(20);  // Sabit gösterim
  if (((millis() - hata_timer) > 5000) && (!hata)) {
    // Gelen veriyi sabit olarak LED ekrana yazdır
    display.clear_buffer(); // Ekranı temizle
    display.message = " HATA WF"; // Sabit metni ekranda goster
	display.BreakTextInFrames(20);
    hata = true;
	sonformattedText = "";
    digitalWrite(LED_PIN, LOW);
  }
#endif



}
