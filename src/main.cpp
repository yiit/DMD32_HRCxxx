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

MD_Parola display = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

void ShowOnDisplay(String message) {
	display.displayClear();
	display.displayText(message.c_str(), PA_CENTER, 50, 1000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
	while (!display.displayAnimate()) {
	  // Animasyon tamamlanana kadar bekle
	}
}

#endif

#include <ESPAsyncWebServer.h>
#include <Update.h>

const char* ssid = "HRC";  // WiFi SSID
const char* password = "teraziwifi";  // WiFi Şifresi

AsyncWebServer server(80);
TaskHandle_t wifiTaskHandle = NULL;

// Web sayfası ile firmware yuklemek icin HTML sayfası
#if defined(HRCMINI) || defined(HRCMESAJ_RGB) || defined(HRCNANO)
const char* upload_html = R"rawliteral(
<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <title>HRCMINI Yönetim</title>
  <style>
    body { font-family:sans-serif; background:#f5f5f5; padding:20px; }
    h2 { color:#333; }
    table { width:100%; border-collapse:collapse; margin-top:20px; background:white; box-shadow:0 2px 5px rgba(0,0,0,0.1); }
    th,td { padding:12px; border-bottom:1px solid #ddd; text-align:left; }
    .green { color:green; } .red { color:red; } .gray { color:gray; }
    button { padding:6px 12px; margin-left:10px; }
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
  <h2>📦 Firmware Güncelle</h2>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="update">
    <input type="submit" value="Yükle">
  </form>

  <h2>📡 Eşleşmiş Cihazlar</h2>
  <table>
    <thead><tr><th>MAC</th><th>Durum</th><th>İşlem</th></tr></thead>
    <tbody id="pairedDevices"></tbody>
  </table>

  <h2>🔍 Çevredeki Cihazlar</h2>
  <table>
    <thead><tr><th>MAC</th><th>Durum</th><th>İşlem</th></tr></thead>
    <tbody id="discoveredDevices"></tbody>
  </table>

  <h2>➕ Manuel Cihaz Ekle</h2>
  <input type="text" id="macInput" placeholder="AA:BB:CC:DD:EE:FF">
  <button onclick="addMac()">Ekle</button>

  <h2>🤝 Eşleşme Başlat</h2>
<button onclick="startPairing()">Cihazlarla Eşleş</button>

<script>
function startPairing() {
  fetch('/start_pairing', { method: 'POST' })
    .then(r => r.text())
    .then(t => alert(t));
}
</script>

  <h2>🗑️ Hafızayı Temizle</h2>
<button onclick="clearPrefs()">Preferences Temizle</button>

<script>
function clearPrefs() {
  if (confirm("Preferences hafızasını silmek istediğine emin misin?")) {
    fetch('/clear_preferences', { method: 'POST' })
      .then(res => res.text())
      .then(txt => alert(txt));
  }
}
</script>


<script>
async function fetchDevices() {
  const res = await fetch('/mac_list');
  const { paired, discovered } = await res.json();

  const pairedTable = document.getElementById("pairedDevices");
  pairedTable.innerHTML = "";
  paired.forEach(dev => {
    const row = document.createElement("tr");
    const color = dev.active ? 'green' : 'red';
    const status = dev.active ? '✅ Aktif' : '❌ Kapalı';
    row.innerHTML = `<td>${dev.mac}</td>
                     <td class="${color}">${status}</td>
                     <td><button onclick="deleteMac('${dev.mac}', this)">Sil</button></td>`;
    pairedTable.appendChild(row);
  });

  const discoveredTable = document.getElementById("discoveredDevices");
  discoveredTable.innerHTML = "";
  discovered.forEach(dev => {
    const row = document.createElement("tr");
    row.innerHTML = `<td>${dev.mac}</td>
                     <td class="gray">🆕 Etrafda</td>
                     <td><button onclick="pairMac('${dev.mac}')">Eşleştir</button></td>`;
    discoveredTable.appendChild(row);
  });
}

function updateSSID() {
  const ssid = document.getElementById("ssidInput").value;
  fetch('/update_ssid', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: 'ssid=' + encodeURIComponent(ssid)
  }).then(() => {
    alert("SSID kaydedildi, yeniden başlatılıyor...");
    setTimeout(() => location.reload(), 3000);
  });
}

function deleteMac(mac, btn) {
  fetch('/delete_mac', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:`mac=${mac}`
  }).then(() => {
    const row = btn.closest('tr');
    row.classList.add("fade-out");
    setTimeout(() => row.remove(), 1000);
  });
}

function addMac() {
  const mac = document.getElementById("macInput").value;
  fetch('/add_mac', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:`mac=${mac}`
  }).then(() => {
    document.getElementById("macInput").value = "";
    fetchDevices();
  });
}

function pairMac(mac) {
  fetch('/add_mac', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:`mac=${mac}`
  }).then(() => fetchDevices());
}

setInterval(fetchDevices, 3000);
fetchDevices();

function startPairing() {
  fetch('/start_pairing', { method: 'POST' })
    .then(r => r.text())
    .then(t => {
      alert(t);
      fetchDevices(); // eşleşme sonrası listeyi güncelle
    });
}

function pairMac(mac) {
  fetch('/pair_request', {
    method: 'POST',
    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
    body: `mac=${mac}`
  }).then(() => fetchDevices());
}


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
  for (int i = 0; i < peerStatusCount; i++) {
    if (memcmp(peerStatusList[i].mac, mac_addr, 6) == 0) {
      peerStatusList[i].active = active;
      return;
    }
  }

  if (peerStatusCount < MAX_PAIRED_DEVICES) {
    memcpy(peerStatusList[peerStatusCount].mac, mac_addr, 6);
    peerStatusList[peerStatusCount].active = active;
    peerStatusCount++;
  }
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

//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  addToDiscoveredList(mac_addr);  
  hata_timer = millis();
  char receivedData[len + 1];
  memcpy(receivedData, data, len);
  receivedData[len] = '\0';

  if (strcmp(receivedData, "PAIR_REQUEST") == 0) {
    Serial.println("Eslesme Istegi Alindi!");
    //ShowOnDisplay("BAGLANIYOR...");
    const char *response = "PAIR_RESPONSE";
	esp_now_send(mac_addr, (uint8_t *)response, strlen(response));
    //esp_now_send(info->src_addr, (uint8_t *)response, strlen(response));
    //memcpy(pairedMacList[0], info->src_addr, 6);
	//Serial.write(pairedMacList[0], 6);
    isPaired = true;
	SavePairedMac(mac_addr);
    //SavePairedMac(info->src_addr);
	AddPeer(mac_addr);
	//AddPeer(info->src_addr);
	//ShowOnDisplay("BAGLANDI!");
#ifdef HRCMINI
	display.displayClear();
	display.displayText("BAGLANDI!", PA_CENTER, 50, 200, PA_SCROLL_LEFT, PA_SCROLL_LEFT); // Sabit pozisyon belirle
    // Animasyonu tamamlamak icin dongu
    while (!display.displayAnimate()) {
    // Animasyonun tamamlanmasını bekle
    }
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
    Serial.println("Eslesme Yaniti Alindi!");
	//ShowOnDisplay("BAGLANIYOR...");
    //memcpy(pairedMacList[0], info->src_addr, 6);
	//Serial.write(pairedMacList[0], 6);
    isPaired = true;
	SavePairedMac(mac_addr);
    //SavePairedMac(info->src_addr);
	//AddPeer(mac_addr);
	//AddPeer(info->src_addr);
	PrintMacAddress(mac_addr);
    //PrintMacAddress(info->src_addr);
#ifdef HRCMINI
	display.displayClear();
	display.displayText("BAGLANDI!", PA_CENTER, 50, 200, PA_SCROLL_LEFT, PA_SCROLL_LEFT); // Sabit pozisyon belirle
    // Animasyonu tamamlamak icin dongu
    while (!display.displayAnimate()) {
    // Animasyonun tamamlanmasını bekle
    }
	sonformattedText = "";
#endif
    return;
  }

  else if (strcmp(receivedData, "inddara") == 0) {
    Serial.println("DARA!");
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
    //Serial.print("Gelen Veri: ");
    //Serial.println(receivedData);
    // Gelen veriyi sabit olarak LED ekrana yazdır
    // display.displayClear(); // Ekranı temizle
    // Gelen veriyi sabit olarak goster
#ifdef HRCMINI
    if (formattedText != sonformattedText) {
	  //formattedText += "(";
	  //formattedText += ") "; // Gelen veriyi formatla
	  display.displayClear(); // Ekranı temizle
	  //display.displayText(formattedText.c_str(), PA_CENTER, 50, 200, PA_SCROLL_LEFT, PA_SCROLL_LEFT); // Sabit pozisyon belirle
	  //display.displayText(formattedText.c_str(), PA_RIGHT, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT); // Sabit pozisyon belirle
	  // Animasyonu tamamlamak icin dongu
	  //while (!display.displayAnimate()) {
	  // Animasyonun tamamlanmasını bekle
	  //}
	  display.displayText(formattedText.c_str(), PA_RIGHT, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT); // Sabit pozisyon belirle
      // Animasyonu tamamlamak icin dongu
      while (!display.displayAnimate()) {
      // Animasyonun tamamlanmasını bekle
      }
	  sonformattedText = formattedText;
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
      SaveSSID(newSSID);
      ESP.restart();  // SSID'yi aktif hale getirmek için restart
    }
    request->send(200);
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
  Serial1.begin(9600, SERIAL_8N1, 17, 16);
  // Parola ekranını baslat
  display.begin();
  display.setIntensity(15); // Parlaklık ayarı (0 - 15 arası)
  display.displayClear();  // Ekranı temizleme
  display.setFont(dotmatrix_5x8);
  display.displayText("HRCMINI",PA_RIGHT,50,200,PA_PRINT,PA_NO_EFFECT);
  while (!display.displayAnimate()) {
    // Animasyon tamamlanana kadar bekle
  }
#endif
#ifdef ESPNOW
  pinMode(PAIR_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //pinMode(15, OUTPUT);
  //digitalWrite(15, HIGH);
  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH); 

  //WiFi.mode(WIFI_AP_STA);
  startHiddenAP();
  startWebServer();
  // ESP-NOW baslat
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Baslatilamadi!");
#ifdef HRCMINI
    ShowOnDisplay("BAGLANTI YOK");
#endif
#ifdef HRCNANO
	display.message = "BAGLANTI YOK"; // Sabit pozisyon belirle
	display.scrollTextHorizontal(200);
#endif
    return;
  } else {
    Serial.println("ESP-NOW Baslatildi!");
#ifdef HRCMINI
	ShowOnDisplay("ENDUTEK");
  //ShowOnDisplay("ATASAN");
	//ShowOnDisplay("ELMASLAR");
  //ShowOnDisplay("ENDER BASKUL");
	//ShowOnDisplay("ARI BASKUL");
	//ShowOnDisplay("CAS");
	//ShowOnDisplay("BILTER");
	//ShowOnDisplay("OLGUN BASKUL");
	//ShowOnDisplay("SELES");
	//ShowOnDisplay("LINEER TARTI");
  delay(1000);
  display.displayClear();
#endif
  }
  // Daha once eslesmis cihaz var mı kontrol et
  /*LoadPairedMac();
  if (!isPaired && !preferences.getBytesLength("paired_mac")) {
    Serial.println("Eslesmis Cihaz Yok. Eslesme Bekleniyor.");
    ShowOnDisplay("ESLESME YOK..");
	Serial1_mod = true; 
  } else {
    Serial.println("Eslesmis cihaz bulundu:");
    PrintMacAddress(pairedMacList[0]);
	for (int i; i < 3; i++)
      AddPeer(pairedMacList[i]); // Eslesmis cihazi peer olarak ekle
	ShowOnDisplay("BAGLANTI KURULUYOR..");
  }*/

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
#ifdef HRCMINI
    ShowOnDisplay("BAGLANTI KURULUYOR..");
#endif
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
 if (isPaired && Serial.available()) {
	char output[9]; // 6 karakter + null terminator icin
    String data = Serial.readStringUntil('\n');
	//String data = Serial.readStringUntil('\r');
    String numericData = ExtractNumericData(data); // Sayısal kısmı ayıkla
	FormatNumericData(numericData, output); // Formatla
    if (String(output).length() > 0) {
      hata_timer = millis();
#ifdef ESPNOW
      SendData(output);
#endif
    }
    Serial.flush();
  }

  if (isPaired && Serial1.available()) {
	char output[9]; // 6 karakter + null terminator icin
    String data = Serial1.readStringUntil('\n');
    //String numericData = ExtractNumericData(data.substring(10,18)); // Sayısal kısmı ayıkla
    String numericData = ExtractNumericData(data);
	FormatNumericData(numericData, output); // Formatla
	//if (data.length() == 17) {
	//if (data.length() == 16) {
	if (data.length() > 5) {
      hata_timer = millis();
      //SendData(output);
	  Serial.println("Terazi Datası : " + data + " Ekran Datası : " + String(output));
	  Serial1.flush();
    }
    //if (Serial1_mod) {
    #ifdef HRCMINI
	  String formattedText = numericData + "(" + ")";
      if (formattedText != sonformattedText) {
	    display.displayText(formattedText.c_str(), PA_RIGHT, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT); // Sabit pozisyon belirle
        // Animasyonu tamamlamak icin dongu
        while (!display.displayAnimate()) {
          // Animasyonun tamamlanmasını bekle
        }
	    sonformattedText = formattedText;
	  }
    #endif
    //}
    Serial1.flush();
  }
  
  else if (!isPaired && Serial1.available()) {
      char output[9]; // 6 karakter + null terminator icin
      String data = Serial1.readStringUntil('\n');
      String numericData = ExtractNumericData(data); // Sayısal kısmı ayıkla
	  FormatNumericData(numericData, output); // Formatla
	  //Serial.println(data.length());
      //if (data.length() == 17) { //seles
	  //if (data.length() == 16) {
	  if (data.length() > 5) {
        hata_timer = millis();
	    Serial.println("Terazi Datası : " + data + " Ekran Datası : " + String(output));
        //if (Serial1_mod) {
	      String formattedText = String(output) + "(" + ")";
      #ifdef HRCMINI
          if (formattedText != sonformattedText) {
	        display.displayText(formattedText.c_str(), PA_RIGHT, 0, 0, PA_NO_EFFECT, PA_NO_EFFECT); // Sabit pozisyon belirle
            // Animasyonu tamamlamak icin dongu
            while (!display.displayAnimate()) {
            // Animasyonun tamamlanmasını bekle
          }		
	    sonformattedText = formattedText;
	    }
      #endif
      //}
    }
	  hata = false;
    Serial1.flush();
   }

  if (((millis() - hata_timer) > 5000) && (!hata)) {
    // Gelen veriyi sabit olarak LED ekrana yazdır
    #ifdef HRCMINI
    display.displayClear(); // Ekranı temizle
    display.setTextAlignment(PA_RIGHT);
    display.print("HATA"); // Sabit metni ekranda goster
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