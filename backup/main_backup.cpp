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

//#define LEADER

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
    Serial.print("Request result: 0x");
    Serial.print(event, HEX);
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
  Serial.println("DMD tarama durduruldu");
}

void resumeDMD() {
  timerAlarmEnable(timer);  // DMD tarama zamanlayıcısını tekrar baslatın
  Serial.println("DMD tarama yeniden baslatıldı");
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
String startupMessages[] = {"HRCMINI", "ENDUTEK", "CONNECTION_CHECK"};
int startupMessagesCount = 3;

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
    if (message == "ENDUTEK" || (!isESPNOWMessage && message != "HATA" && message != "HRCMINI")) {
        // ENDUTEK her zaman kayan yazı
        if (message == "ENDUTEK") {
            needsScrolling = true;
            Serial.println("ENDUTEK - forced scroll animation");
        } else {
            // Diğer mesajlar için genişlik kontrolü
            int textWidth = message.length() * 6;  // Her karakter ~6 pixel
            int displayWidth = 1 * 40; // 5 modül, her modül 32 pixel genişlik = 160 pixel
            
            if (textWidth > displayWidth) {
                needsScrolling = true;
                Serial.println("Text too wide (" + String(textWidth) + "px > " + String(displayWidth) + "px), using scroll");
            } else {
                Serial.println("Text fits (" + String(textWidth) + "px <= " + String(displayWidth) + "px), using static");
            }
        }
    }
    
    // BLOCKING APPROACH - Hemen göster ve animasyonu bekle
    if (message == "HATA" || message == "HRCMINI" || isESPNOWMessage || !needsScrolling) {
        // Static göster (sola yaslanmış)
        display.displayText(message.c_str(), PA_LEFT, 0, 0, PA_PRINT, PA_NO_EFFECT);
        Serial.println("Static message set (LEFT aligned) - blocking animate...");
        // BLOCKING animate
        unsigned long timeout = millis() + 2000;
        while (!display.displayAnimate() && millis() < timeout) {
            delay(50);
        }
        Serial.println("Static animation completed");
    } else {
          // Ekranı temizle
        display.displayClear();
        display.displayReset(); 
        // Uzun mesajlar scroll göster
        display.displayText(message.c_str(), PA_CENTER, 60, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
        Serial.println("Scroll message set - blocking animate...");
        // BLOCKING animate - max 5 saniye
        unsigned long timeout = millis() + 5000;
        int animCount = 0;
        while (!display.displayAnimate() && millis() < timeout) {
            delay(30);
            animCount++;
            if (animCount % 20 == 0) {
                Serial.println("Animation step: " + String(animCount));
            }
        }
        Serial.println("Scroll animation completed after " + String(animCount) + " steps");
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
            <span>🔄</span> Cihaz Tarama Başlat
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
            <tr><th>MAC Adresi</th><th>Durum</th><th>Son Görülme</th><th>İşlemler</th></tr>
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
            <tr><th>MAC Adresi</th><th>Sinyal Gücü</th><th>Durum</th><th>İşlemler</th></tr>
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

// Sayfa yüklendiğinde başlat
document.addEventListener('DOMContentLoaded', function() {
  fetchDevices();
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

// ESP-NOW işlemleri
function startPairing() {
  showModal(
    '🔄 Cihaz Tarama',
    'ESP-NOW cihaz tarama başlatılsın mı? Bu işlem birkaç saniye sürer.',
    function() {
      showNotification('Cihaz tarama başlatıldı...', 'warning');
      
      fetch('/start_pairing', { method: 'POST' })
        .then(r => r.text())
        .then(response => {
          showNotification(`Tarama tamamlandı: ${response}`, 'success');
          fetchDevices();
        })
        .catch(() => showNotification('Tarama başlatma hatası!', 'error'));
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
      fetch('/delete_mac', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `mac=${mac}`
      })
      .then(() => {
        showNotification('Cihaz hafızadan silindi!', 'success');
        fetchDevices();
      })
      .catch(() => showNotification('Cihaz silme hatası!', 'error'));
    }
  );
}

function pairMac(mac) {
  showModal(
    '🤝 Cihaz Eşleştir',
    `MAC adresi "${mac}" ile eşleştirme yapılacak. Devam edilsin mi?`,
    function() {
      showNotification('Eşleştirme isteği gönderiliyor...', 'warning');
      
      fetch('/pair_request', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: `mac=${mac}`
      })
      .then(() => {
        showNotification('Eşleştirme isteği gönderildi!', 'success');
        fetchDevices();
      })
      .catch(() => showNotification('Eşleştirme hatası!', 'error'));
    }
  );
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
      pairedTable.innerHTML = '<tr><td colspan="4" style="text-align: center; color: #7f8c8d;">Henüz eşleşmiş cihaz yok</td></tr>';
    } else {
      paired.forEach(device => {
        const row = document.createElement('tr');
        const statusClass = device.active ? 'status-online' : 'status-offline';
        const statusText = device.active ? '🟢 Çevrimiçi' : '🔴 Çevrimdışı';
        const lastSeen = device.lastSeen || 'Bilinmiyor';
        
        row.innerHTML = `
          <td><code>${device.mac}</code></td>
          <td><span class="status ${statusClass}">${statusText}</span></td>
          <td>${lastSeen}</td>
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
      discoveredTable.innerHTML = '<tr><td colspan="4" style="text-align: center; color: #7f8c8d;">Çevrede cihaz bulunamadı</td></tr>';
    } else {
      discovered.forEach(device => {
        const row = document.createElement('tr');
        const signalStrength = device.rssi ? `${device.rssi} dBm` : 'Bilinmiyor';
        
        row.innerHTML = `
          <td><code>${device.mac}</code></td>
          <td>${signalStrength}</td>
          <td><span class="status status-discovered">🆕 Keşfedildi</span></td>
          <td>
            <button onclick="pairMac('${device.mac}')" class="btn btn-small btn-success">
              <span>🤝</span> Eşleştir
            </button>
          </td>
        `;
        discoveredTable.appendChild(row);
      });
    }
  } catch (error) {
    console.error('Cihaz listesi alınamadı:', error);
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
        data.data.forEach(entry => addEspnowDataLine(entry.type, entry.message, entry.timestamp));
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
  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.end();
}

// Gizli AP modu baslat ve IP adresini 192.168.4.1 olarak ayarla
void startHiddenAP() {

  preferences.begin("wifi", true);
  String savedSSID = preferences.getString("ssid", "HRC_DEFAULT");
  preferences.end();
  // Gizli (SSID yayınlamayan) bir Access Point baslatma
  //WiFi.softAP(ssid, password, 1, 1);  // Gizli mod aktif

  IPAddress local_IP(192, 168, 4, 1);  // Sabit IP adresi
  IPAddress gateway(192, 168, 4, 1);   // Ag gecidi (gateway)
  IPAddress subnet(255, 255, 255, 0);  // Alt ag maskesi

  while (!WiFi.mode(WIFI_AP_STA)) {
	Serial.println("AP-STA MOD BASARISIZ!");
  }
  
  while (!WiFi.softAP(savedSSID.c_str(), password)) {
    Serial.println("AP-STA SSID BASARISIZ!");
    return;
  }

  // AP-STA IP yapılandırmasını ayarla
  while (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP-STA KOFIGURASYON BASARISIZ!");
    return;
  }

  // Sabit IP adresini seri monitore yazdır
  Serial.println("Gizli AP baslatıldı!");
  Serial.print("AP IP Adresi: ");
  Serial.println(WiFi.softAPIP());
}

#ifdef ESPNOW

void updatePeerStatus(const uint8_t *mac_addr, bool active) {
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  
  for (int i = 0; i < peerStatusCount; i++) {
    if (memcmp(peerStatusList[i].mac, mac_addr, 6) == 0) {
      if (peerStatusList[i].active != active) {
        Serial.printf("📊 Peer status değişti: %s -> %s\n", macStr, active ? "ACTIVE" : "INACTIVE");
      }
      peerStatusList[i].active = active;
      return;
    }
  }

  if (peerStatusCount < MAX_PAIRED_DEVICES) {
    memcpy(peerStatusList[peerStatusCount].mac, mac_addr, 6);
    peerStatusList[peerStatusCount].active = active;
    peerStatusCount++;
    Serial.printf("📊 Yeni peer eklendi: %s -> %s\n", macStr, active ? "ACTIVE" : "INACTIVE");
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
  uint8_t mac[6];
  sscanf(macStr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

  int index = -1;
  for (int i = 0; i < pairedDeviceCount; i++) {
    if (memcmp(pairedMacList[i], mac, 6) == 0) {
      index = i;
      break;
    }
  }

  if (index >= 0) {
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
    esp_now_del_peer(mac);
    Serial.println("🧹 MAC başarıyla silindi.");
  } else {
    Serial.println("⚠️ MAC adresi listede bulunamadı.");
  }
}

void addToDiscoveredList(const uint8_t *mac_addr) {
  for (int i = 0; i < pairedDeviceCount; i++) {
    if (memcmp(pairedMacList[i], mac_addr, 6) == 0) return;
  }
  for (int i = 0; i < discoveredCount; i++) {
    if (memcmp(discoveredMacList[i], mac_addr, 6) == 0) return;
  }
  if (discoveredCount < MAX_PAIRED_DEVICES) {
    memcpy(discoveredMacList[discoveredCount], mac_addr, 6);
    discoveredCount++;
  }
}

void StartPairing() {
  Serial.println("Eslesme Modu Baslatildi...");
  const char *pairMessage = "PAIR_REQUEST";

  // Broadcast adresine mesaj gonder
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)pairMessage, strlen(pairMessage));
  if (result == ESP_OK) {
    Serial.println("Eslesme Istegi Basariyla Gonderildi.");
  } else {
    Serial.println("Eslesme Istegi Gonderilemedi.");
#ifdef HRCMINI
    ShowOnDisplay("BAGLANTI YOK!!!");
#endif
#ifdef HRCNANO
    display.message = "BAGLANTI YOK!!!";
	display.scrollTextHorizontal(200);
#endif
  }
  digitalWrite(LED_PIN, HIGH);
  delay(3000);
  digitalWrite(LED_PIN, LOW);
#ifdef HRCMINI
  display.displayClear();
#endif
}

void SavePairedMac(const uint8_t *newMac) {
  preferences.begin("espnow", false);
  pairedDeviceCount = preferences.getBytesLength("paired_mac") / 6;
  if (pairedDeviceCount > MAX_PAIRED_DEVICES) pairedDeviceCount = MAX_PAIRED_DEVICES;
  preferences.getBytes("paired_mac", pairedMacList, pairedDeviceCount * 6);

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
    } else {
      for (int i = 0; i < MAX_PAIRED_DEVICES - 1; i++) {
        memcpy(pairedMacList[i], pairedMacList[i + 1], 6);
      }
      memcpy(pairedMacList[MAX_PAIRED_DEVICES - 1], newMac, 6);
    }
  }

  preferences.putBytes("paired_mac", pairedMacList, pairedDeviceCount * 6);
  preferences.end();

  updatePeerStatus(newMac, true);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, newMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(newMac)) {
    esp_now_add_peer(&peerInfo);
  }

  Serial.println("✅ Eşleşmiş MAC adresi kaydedildi.");
}

void PrintMacAddress(const uint8_t *mac) {
  Serial.print("MAC Adresi: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

String macToStr(const uint8_t *mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void AddPeer(const uint8_t *mac_addr) {
  if (esp_now_is_peer_exist(mac_addr)) {
    Serial.println("ℹ️ Peer zaten kayıtlı.");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac_addr, 6);
  peerInfo.channel = 0;         // aynı kanal
  peerInfo.encrypt = false;     // şifreleme kullanmıyoruz

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK) {
    Serial.print("✅ Peer eklendi: ");
    PrintMacAddress(mac_addr);
  } else {
    Serial.printf("❌ Peer eklenemedi! Hata kodu: %d\n", result);
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  addToDiscoveredList(mac_addr);  
  hata_timer = millis();
  char receivedData[len + 1];
  memcpy(receivedData, data, len);
  receivedData[len] = '\0';
  
  // Web monitörü için ESP-NOW gelen veriyi kaydet
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  addEspnowData("From " + String(macStr) + ": " + String(receivedData), "in");

  if (strcmp(receivedData, "PAIR_REQUEST") == 0) {
    Serial.println("Eslesme Istegi Alindi!");
    const char *response = "PAIR_RESPONSE";
	esp_now_send(mac_addr, (uint8_t *)response, strlen(response));
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
  Serial.println("🧹 Pair silme komutu alındı!");
  String macStr = macToStr(mac_addr);
  RemovePairedMac(macStr.c_str());
}
  else if (strcmp(receivedData, "PAIR_RESPONSE") == 0) {
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
    //Serial.println("SIFIR!");
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
	    Serial.println("Yazma basarili!");
	  } else {
	    Serial.print("Yazma Hatasi, Kod = ");
	    Serial.println(result, HEX);
	  }
  }
#endif

  else {
#ifdef HRCMINI
	  String formattedText = String(receivedData) + "(" + ") ";
#else
	  String formattedText = String(receivedData);
#endif
#ifdef HRCMINI
    if (formattedText != sonformattedText) {
	    //display.displayClear(); // Ekranı temizle
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
	}
	else {
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
	}
	else {
	  ekran_goster = formattedText + "(" + ") ";
	}
	display.message = ekran_goster; // Sabit pozisyon belirle
	display.BreakTextInFrames(20);
#endif
	// Gelen veriyi seri monitora yazdır
	Serial.print("Gelen veri: ");
	Serial.println(receivedData);
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
    
    esp_err_t result = esp_now_send(pairedMacList[i], (uint8_t *)data.c_str(), data.length());
    updatePeerStatus(pairedMacList[i], result == ESP_OK);
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
      // TODO: Şifre kaydetme işlemi eklenecek
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
        Preferences prefs;
        prefs.begin("display", false);
        prefs.putInt("brightness", brightness);
        prefs.end();
        
        request->send(200, "text/plain", "Parlaklık ayarlandı");
      } else {
        request->send(400, "text/plain", "Geçersiz parlaklık değeri (0-100)");
      }
    } else {
      request->send(400, "text/plain", "Parlaklık parametresi eksik");
    }
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
      RemovePairedMac(macStr.c_str());
    }
    request->send(200);
  });

  server.on("/pair_request", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("mac", true)) {
      request->send(400, "text/plain", "MAC eksik");
      return;
    }

    String macStr = request->getParam("mac", true)->value();
    uint8_t mac[6];
    if (sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
             &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
      request->send(400, "text/plain", "MAC formatı hatalı");
      return;
    }

    const char* msg = "PAIR_REQUEST";
    esp_err_t result = esp_now_send(mac, (uint8_t*)msg, strlen(msg));
    if (result == ESP_OK) {
      Serial.printf("📤 PAIR_REQUEST gönderildi: %s\n", macStr.c_str());
      request->send(200, "text/plain", "Gönderildi");
    } else {
      Serial.printf("❌ Gönderim hatası: %s\n", macStr.c_str());
      request->send(500, "text/plain", "Gönderilemedi");
    }
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
  Serial.println("Web sunucusu baslatıldı.");
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
			if (value > 99.999) {
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

#ifdef HRCMESAJ
void processModbusData() {
  //Serial.print("Gelen Veri: ");
  Serial.println(receivedData); // Gelen veriyi ASCII olarak yazdır

  // Tartım verisini ayıkla
  if (receivedData[0] == ':' && strlen(receivedData) >= 13) {
    // Tartım verisi 7. ve 8. indeksten baslıyor
    char tartimHex[5] = {receivedData[7], receivedData[8], receivedData[9], receivedData[10], '\0'};
    int tartimDecimal = strtol(tartimHex, NULL, 16); // Hex'ten Decimal'e cevir
    //Serial.print("Tartım Bilgisi (Decimal): ");
	char formattedTartim[10]; // Formatlanmıs veri icin buffer
	snprintf(formattedTartim, sizeof(formattedTartim), "%6d()  ", tartimDecimal);
	Serial.println(formattedTartim);
	// Formatlanmıs veriyi ekrana yazdır
	dmd.drawString(0, 0, formattedTartim, (strlen(formattedTartim)+2), GRAPHICS_NORMAL);
  } else {
    Serial.println("Gecersiz Veri");
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
  display.message = " ENDUTEK"; // Sabit pozisyon belirle
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
  
  // Kaydedilmiş parlaklık ayarını yükle
  Preferences prefs;
  prefs.begin("display", true);
  int savedBrightness = prefs.getInt("brightness", 50); // Varsayılan %50
  prefs.end();
  
  uint8_t intensity = map(savedBrightness, 0, 100, 0, 15);
  display.setIntensity(intensity); // Parlaklık ayarı (0 - 15 arası)
  display.displayClear();  // Ekranı temizleme
  display.setFont(dotmatrix_5x8);
  
  Serial.printf("Parlaklık yüklendi: %d%% (intensity: %d)\n", savedBrightness, intensity);
  
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
    Serial.println("ESP-NOW Baslatilamadi!");
#ifdef HRCMINI
#endif
#ifdef HRCNANO
	display.message = "BAGLANTI YOK"; // Sabit pozisyon belirle
	display.scrollTextHorizontal(200);
#endif
    return;
  } else {
    Serial.println("ESP-NOW Baslatildi!");
    // Startup sequence loop'ta başlatılacak
  }
  // Daha once eslesmis cihaz var mı kontrol et
  LoadPairedMac();

if (!isPaired && !preferences.getBytesLength("paired_mac")) {
    Serial.println("Eslesmis Cihaz Yok. Eslesme Bekleniyor.");
#ifdef HRCMINI
    ShowOnDisplay("ESLESME YOK..");
#endif
#ifdef HRCNANO
	display.message = "ESLESME YOK.."; // Sabit pozisyon belirle
	display.scrollTextHorizontal(200);
#endif
    Serial1_mod = true; 
} else {
    Serial.println("Eslesmis cihaz bulundu:");
    PrintMacAddress(pairedMacList[0]);
    
    // 'i' degiskenini 0 olarak baslat ve kayıtlı cihaz sayısına gore dongu yap
    int maxDeviceCount = sizeof(pairedMacList) / sizeof(pairedMacList[0]);
    for (int i = 0; i < pairedDeviceCount && i < maxDeviceCount; i++) {
        if (pairedMacList[i] != nullptr) { // Gecerli MAC adresi kontrolu
            //AddPeer(pairedMacList[i]); // Eslesmis cihazı peer olarak ekle
            Serial.printf("Peer eklendi: Cihaz %d\n", i);
        } else {
            Serial.printf("Gecersiz MAC adresi: Cihaz %d\n", i);
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
    Serial.println("Peer Eklenemedi!");
#ifdef HRCMINI
    ShowOnDisplay("PEER ERR");
#endif
  } else {
    Serial.println("Peer Basariyla Eklendi.");
  }
#endif
#ifdef HRCMINI
ShowOnDisplay("HRCMINI");
delay(2000);
ShowOnDisplay("ENDUTEK");
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
	//Serial.print(inByte);
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
  //modbusSerial.print(":010300000001FB\r\n");
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
      StartPairing(); // 3 saniye boyunca basılı tutulduysa eslesme baslat
      buttonPressed = false; // İslem tamam, tekrar tetiklenmesini engelle
    }
  } else { // Buton bırakıldıgında
    buttonPressed = false; // Durumu sıfırla
  }
#endif

#if defined(HRCMINI) || defined(HRCNANO)
 if (Serial.available()) {
	char output[9]; // 6 karakter + null terminator icin
    String data = Serial.readStringUntil('\n');
	//String data = Serial.readStringUntil('\r');
    
    // Web monitörü için veriyi kaydet
    if (data.length() > 0) {
      addSerialData("Serial: " + data.substring(0, min((int)data.length(), 50))); // İlk 50 karakter
    }
    
    String numericData = ExtractNumericData(data); // Sayısal kısmı ayıkla
	FormatNumericData(numericData, output); // Formatla
    if (String(output).length() > 0) {
      hata_timer = millis();
#ifdef ESPNOW
      if (isPaired) {
        SendData(output);
      }
#endif
    }
    Serial.flush();
  }

  if (Serial1.available()) {
	char output[9]; // 6 karakter + null terminator icin
    String data = Serial1.readStringUntil('\n');
    
    // Web monitörü için veriyi kaydet
    if (data.length() > 0) {
      addSerialData("Serial1: " + data.substring(0, min((int)data.length(), 50))); // İlk 50 karakter
    }
    
    //String numericData = ExtractNumericData(data.substring(10,18)); // Sayısal kısmı ayıkla
    String numericData = ExtractNumericData(data);
	FormatNumericData(numericData, output); // Formatla
	//if (data.length() == 17) {
	//if (data.length() == 16) {
	if (data.length() > 5) {
      hata_timer = millis();
#ifdef ESPNOW
        SendData(output);
#endif
	  Serial.println("Terazi Datası : " + data + " Ekran Datası : " + String(output));
      
      // Formatlanmış veriyi de monitöre ekle
      addSerialData("Processed: " + String(output));
      
	  Serial1.flush();
    }
    //if (Serial1_mod) {
    #ifdef HRCMINI
	  String formattedText = numericData + "(" + ")";
      if (formattedText != sonformattedText) {
	    ShowOnDisplay(formattedText);
	    sonformattedText = formattedText;
	  }
    #endif
    //}
    Serial1.flush();
  }
  
  else if (!isPaired && Serial1.available()) {
      char output[9]; // 6 karakter + null terminator icin
      String data = Serial1.readStringUntil('\n');
      
      // Web monitörü için veriyi kaydet
      if (data.length() > 0) {
        addSerialData("Raw: " + data.substring(0, min((int)data.length(), 50))); // İlk 50 karakter
      }
      
      String numericData = ExtractNumericData(data); // Sayısal kısmı ayıkla
	  FormatNumericData(numericData, output); // Formatla
	  //Serial.println(data.length());
      //if (data.length() == 17) { //seles
	  //if (data.length() == 16) {
	  if (data.length() > 5) {
        hata_timer = millis();
	    Serial.println("Terazi Datası : " + data + " Ekran Datası : " + String(output));
        
        // Formatlanmış veriyi de monitöre ekle
        addSerialData("Processed: " + String(output));
        //if (Serial1_mod) {
	      String formattedText = String(output) + "(" + ")";
      #ifdef HRCMINI
          if (formattedText != sonformattedText) {
	        ShowOnDisplay(formattedText);
	        sonformattedText = formattedText;
	      }
      #endif
      //}
    }
	  hata = false;
    Serial1.flush();
   }

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

    Serial.print("Register[0x10] = ");
    Serial.print(val1);
    Serial.print("   Register[0x11] = ");
    Serial.println(val2);
  } else {
    // Hata kodu
    Serial.print("Modbus read error, code: ");
    Serial.println(result, HEX);
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
  Serial.println(signedValue);*/

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