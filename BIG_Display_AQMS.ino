#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>   // <--- YOU MUST ADD THIS EXACT LINE
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include <RTClib.h>
RTC_DS3231 rtc;
#include <Wire.h>

String companyName = "";
String companyAddress = "";
unsigned long lastHeaderChange = 0;
uint8_t headerPage = 0;
unsigned long lastSensorRead = 0;

// --- CLASSIC ESP32 SAFE DISPLAY PINS ---
// CRITICAL: These match the classic ESP32 DevKit V1
#define PIN_OE    22
#define PIN_A     19
#define PIN_B     21
#define PIN_CLK   18
#define PIN_LAT   5
#define PIN_DATA  23

#define RXD1 17
#define TXD1 16
HardwareSerial RS485Serial(2);
const uint16_t MODBUS_MAX_FRAME = 64;
const uint16_t MODBUS_INTERBYTE_MS = 40;   // time in ms used as inter-byte timeout (~3.5 char @ 9600)
const int DE_PIN = -1;                    // set to DE pin number if your RS485 transceiver needs driver enable, otherwise -1
// buffer used by the frame reader
uint8_t frameBuffer[MODBUS_MAX_FRAME];
uint16_t frameLen = 0;

//  modbus varialbles .............................................
uint8_t requestBuffer[64];
uint8_t responseBuffer[64];
uint16_t registerAddress;
uint16_t registerValue;
const uint16_t registerCount = 14;
uint16_t slaveAddress = 01;
uint16_t modbusRegisters[registerCount];
float pm25,pm10,hum,temp,aqi;

// ================== System Reboot ===================================
#define AUTO_REBOOT_INTERVAL (3UL * 60UL * 60UL * 1000UL) // 6 hours
unsigned long bootTime;

#define WIDTH   128
#define HEIGHT   96

// --- DIRECT REGISTER ACCESS MACROS (100x Faster than digitalWrite) ---
#define FAST_CLK_HIGH()  REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_CLK)
#define FAST_CLK_LOW()   REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_CLK)
#define FAST_DATA_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_DATA)
#define FAST_DATA_LOW()  REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_DATA)
#define FAST_LAT_HIGH()  REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_LAT)
#define FAST_LAT_LOW()   REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_LAT)
#define FAST_OE_HIGH()   REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_OE)
#define FAST_OE_LOW()    REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_OE)

// --- PRE-CALCULATED LINEAR DMA BUFFERS ---
uint8_t renderBuffer[WIDTH][HEIGHT];

// 24 panels * 128 pixels per row per panel = 3072 pixels per scanline
uint8_t scanBuffer1[4][3072]; 
uint8_t scanBuffer2[4][3072];  
volatile uint8_t (*activeScanBuffer)[3072] = scanBuffer1; // ISR reads from here instantly
uint8_t (*drawScanBuffer)[3072] = scanBuffer2;            // Graphics compile here safely

class LEDMatrix : public Adafruit_GFX {
public:
  LEDMatrix(int16_t w, int16_t h) : Adafruit_GFX(w, h) {}
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
      renderBuffer[x][y] = (color > 0) ? 1 : 0;
    }
  }
};

LEDMatrix display(WIDTH, HEIGHT);
WebServer server(80);
Preferences preferences;

unsigned long wifiStartTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 15 * 60 * 1000; 
bool wifiActive = true;

String customFooter = "INSTRUMEX AQI SYSTEM";
int scrollX = WIDTH;
unsigned long lastScrollTime = 0;
int displayBrightness = 0; 
int scanDelay = 1500;  

// Parsed Data Variables
String pm25_val = "--"; 
String pm10_val = "--";
String temp_val = "--";
String hum_val  = "--";
String aqi_val = "--";
String time_val = "--:--";
String date_val = "--/--/--";
String rs485Buffer = "";

// Timer Global Variables
esp_timer_handle_t matrix_timer;
volatile uint8_t scanRow = 0;

// ==========================================
// --- ZERO-MATH HARDWARE ISR ENGINE ---
// ==========================================
void IRAM_ATTR matrix_scan_isr(void* arg) {
  // 1. Turn OFF the display
  FAST_OE_HIGH();
  
  // 2. Grab the pre-calculated memory line for this exact row
  volatile uint8_t *row_data = activeScanBuffer[scanRow];
  
  // 3. Blast the bits to the shift registers (Zero Math!)
  for (int i = 0; i < 3072; i++) {
    if (row_data[i]) {
      FAST_DATA_LOW();  // Inverted for HUB12
    } else {
      FAST_DATA_HIGH();
    }
    FAST_CLK_HIGH();
    FAST_CLK_LOW();
  }

  // 4. Latch the shifted data
  FAST_LAT_HIGH();
  FAST_LAT_LOW();

  // 5. Set the Row Address Pins (A and B)
  if (scanRow & 0x01) REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_A); else REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_A);
  if (scanRow & 0x02) REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_B); else REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_B);

  // 6. Turn ON the display
  FAST_OE_LOW();

  // 7. Move to next row
  scanRow++;
  if (scanRow > 3) scanRow = 0;
}

// ==========================================
// --- SIMPLE WEB PORTAL ---
// ==========================================
const char* htmlPage = R"rawliteral(
<!DOCTYPE html><html><head><title>Instrumex Display</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:Arial;text-align:center;background:#f4f4f4;}
  .card{background:white;padding:25px;border-radius:10px;max-width:400px;margin:auto;box-shadow: 0px 5px 15px rgba(0,0,0,0.1);}
  input[type="range"]{width:90%;padding:10px;margin:10px 0;}
  input[type="text"]{width:90%;padding:10px;margin:10px 0; border:1px solid #ccc; border-radius:5px;}
  .btn{color:white;border:none;padding:14px;cursor:pointer;border-radius:6px;width:100%;font-weight:bold;font-size:16px;}
  .btn-grn{background:#2e7d32;}
  .btn-org{background:#f57c00;margin-top:15px;}
  label{font-weight:bold;display:block;text-align:left;margin-left:5%;margin-top:10px;}
</style></head>
<body><div class="card"><h2 style="color:#0f4c81;margin-top:0;">Display Settings</h2>
<form action="/save" method="POST">
  <label>Dimming Level: <span id="bval">%BRIGHT%</span></label>
  <input type="range" name="bright" min="0" max="100" value="%BRIGHT%" oninput="document.getElementById('bval').innerText=this.value">
  <label>1st Row Custom Header:</label>
  <label>Company First Name:</label><input type="text" name="company" value="%COMPANY%">
  <label>Company Last Name:</label><input type="text" name="address" value="%ADDRESS%">
  <label>Date (DD/MM/YYYY)</label><input type="text" name="date" value="%DATE%">
  <label>Time (HH:MM:SS)</label><input type="text" name="time" value="%TIME%">
  <input type="button"
       value="Use Current PC Time"
       onclick="setCurrentTime()">
  <br><br><input type="submit" value="Save Settings" class="btn btn-grn">
</form>
<form action="/auto" method="POST">
  <input type="submit" value="Auto Tune (Fix Screen)" class="btn btn-org">
</form>
</div></body></html>

<script>
function setCurrentTime()
{
    let now = new Date();

    let date =
        String(now.getDate()).padStart(2,'0') + "/" +
        String(now.getMonth()+1).padStart(2,'0') + "/" +
        now.getFullYear();

    let time =
        String(now.getHours()).padStart(2,'0') + ":" +
        String(now.getMinutes()).padStart(2,'0') + ":" +
        String(now.getSeconds()).padStart(2,'0');

    document.getElementsByName("date")[0].value = date;
    document.getElementsByName("time")[0].value = time;
}
</script>

)rawliteral";

void handleRoot() {
  String html = htmlPage;
  html.replace("%BRIGHT%", String(displayBrightness));
  html.replace("%DELAY%", String(scanDelay));
  html.replace("%COMPANY%", companyName);
  html.replace("%ADDRESS%", companyAddress);
  html.replace("%DATE%", date_val);
  html.replace("%TIME%", time_val);
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("bright")) displayBrightness = server.arg("bright").toInt();
  if (server.hasArg("delay")) scanDelay = server.arg("delay").toInt();
  if (server.hasArg("company"))
    companyName = server.arg("company");

if (server.hasArg("address"))
    companyAddress = server.arg("address");

  preferences.putString("company", companyName);
  preferences.putString("address", companyAddress);
  
  preferences.putInt("bright", displayBrightness);
  preferences.putInt("delay", scanDelay);

  if(server.hasArg("date") && server.hasArg("time"))
{
    String newDate = server.arg("date");
    String newTime = server.arg("time");

    int day   = newDate.substring(0,2).toInt();
    int month = newDate.substring(3,5).toInt();
    int year  = newDate.substring(6,10).toInt();

    int hour   = newTime.substring(0,2).toInt();
    int minute = newTime.substring(3,5).toInt();
    int second = newTime.substring(6,8).toInt();

    rtc.adjust(DateTime(year, month, day,
                        hour, minute, second));
}

}

void handleAuto() {
  displayBrightness = 0; scanDelay = 1500;      
  preferences.putInt("bright", 0); preferences.putInt("delay", 1500);
  server.sendHeader("Location", "/"); server.send(303);
}

// ==========================================
// --- GRAPHICS & BUFFER COMPILER ---
// ==========================================
void updateBuffer() {
  // 1. Wipe the rendering board clean
  memset(renderBuffer, 0, sizeof(renderBuffer));
  
  // 2. Draw all Text and Lines using Adafruit_GFX
  display.setTextSize(1); 
  display.setTextColor(1);

  if (headerPage == 0)
{
    // Date + Time
    display.setCursor(4, 4);
    display.print(date_val);

    display.setCursor(76, 4);
    display.print(time_val);
}
else if (headerPage == 1)
{
    // Company Name
    display.setCursor(3, 4);
    display.print(companyName);
    display.setCursor(55, 4);
     display.print(companyAddress);  
           
        
}

  display.setCursor(4, 20);  display.print("PM2.5");
  display.setCursor(56, 20); display.print(pm25_val); 
  display.setCursor(96, 20); display.print("ug/m3");  

  display.setCursor(4, 36);  display.print("PM10");
  display.setCursor(56, 36); display.print(pm10_val);
  display.setCursor(96, 36); display.print("ug/m3");

  display.setCursor(4, 52);  display.print("TEMP");
  display.setCursor(56, 52); display.print(temp_val);
  display.setCursor(102, 52); display.print((char)247); display.print("C");    

  display.setCursor(4, 68);  display.print("HUM");
  display.setCursor(56, 68); display.print(hum_val);
  display.setCursor(108, 68); display.print("%");   

  display.setCursor(4,84);   display.print("AQI");
  display.setCursor(56,84);  display.print(aqi_val);
  display.setCursor(108,84); display.print("");

  display.drawRect(0, 0, 128, 96, 1);
  display.drawLine(0, 15, 127, 15, 1); 
  display.drawLine(0, 31, 127, 31, 1);
  display.drawLine(0, 47, 127, 47, 1); 
  display.drawLine(0, 63, 127, 63, 1);
  display.drawLine(0, 79, 127, 79, 1);

  // 3. Compile the 2D renderBuffer into the 1D high-speed hardware format
  for (uint8_t row = 0; row < 4; row++) {
    int bit_idx = 0;
    for (int p = 0; p < 24; p++) {
      int panel_x_grid = 3 - (p % 4);
      int panel_y_grid = p / 4;
      for (int chunk_x = 0; chunk_x < 4; chunk_x++) {
        for (int chunk_y = 0; chunk_y < 4; chunk_y++) {
          for (int x_pixel = 0; x_pixel < 8; x_pixel++) {
            int local_x = 31 - ((chunk_x * 8) + x_pixel);
            int local_y = 15 - ((chunk_y * 4) + (3 - row));
            int x = (panel_x_grid * 32) + local_x;
            int y = (panel_y_grid * 16) + local_y;
            int finalX = (WIDTH - 1) - x;

            if (finalX >= 0 && finalX < WIDTH && y >= 0 && y < HEIGHT) {
              drawScanBuffer[row][bit_idx] = renderBuffer[finalX][y];
            } else {
              drawScanBuffer[row][bit_idx] = 0;
            }
            bit_idx++;
          }
        }
      }
    }
  }

  // 4. Instantly flip the memory pointer so the ISR reads the new frame!
  if (activeScanBuffer == scanBuffer1) {
    activeScanBuffer = scanBuffer2;
    drawScanBuffer = scanBuffer1;
  } else {
    activeScanBuffer = scanBuffer1;
    drawScanBuffer = scanBuffer2;
  }
}

// ==========================================
// --- MAIN SETUP ---
// ==========================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_OE, OUTPUT); pinMode(PIN_A, OUTPUT); pinMode(PIN_B, OUTPUT);
  pinMode(PIN_CLK, OUTPUT); pinMode(PIN_LAT, OUTPUT); pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_OE, HIGH); digitalWrite(PIN_CLK, LOW); digitalWrite(PIN_LAT, LOW);

  Wire.begin(25,26);
  

  RS485Serial.begin(9600, SERIAL_8N1, RXD1, TXD1);

  display.setTextWrap(false);
  if (!rtc.begin())
{
    Serial.println("RTC NOT FOUND");
}else
{
    Serial.println("RTC FOUND");
}
Serial.println("RS485 Started");
Serial.printf("RX=%d TX=%d Baud=9600\n", RXD1, TXD1);

  preferences.begin("aqi", false);
  displayBrightness = preferences.getInt("bright", 0);
  scanDelay = preferences.getInt("delay", 1500);
  customFooter = preferences.getString("footer", "INSTRUMEX AQI SYSTEM");
  companyName    = preferences.getString("company", "INSTRUMEX");
  companyAddress = preferences.getString("address", "NASHIK");

  WiFi.softAP("Soarmlich AQI_BOARD", "1234567890"); 
  IPAddress IP = WiFi.softAPIP();

  server.on("/", handleRoot); 
  server.on("/save", handleSave); 
  server.on("/auto", HTTP_POST, handleAuto);
  server.begin();

  // Load Boot Screen
  memset(renderBuffer, 0, sizeof(renderBuffer));
  display.setTextSize(1); display.setTextColor(1);
  display.drawRect(0, 0, 128, 96, 1);
  display.setCursor(4, 4); display.print("FW VER: V4.2-CLASSIC");
  display.setCursor(4, 20); display.print("Soarmlich P10"); 
  display.setCursor(4, 36); display.print("IP: " + IP.toString());
  
  delay(1500);
  // Force manual compile for boot screen
  updateBuffer(); 

  // --- START THE HIGH-SPEED HARDWARE TIMER ---
  const esp_timer_create_args_t timer_args = {
    .callback = &matrix_scan_isr,
    .name = "matrix_timer"
  };
  esp_timer_create(&timer_args, &matrix_timer);
  
  // Fire every 1200us (200Hz full screen refresh rate)
  esp_timer_start_periodic(matrix_timer, 1200); 

  wifiStartTime = millis();
  bootTime = millis();
  delay(3000); 
}

void loop() {

if (millis() - bootTime > AUTO_REBOOT_INTERVAL) {
  display.setCursor(4, 20);  display.print("");
  display.setCursor(56, 20); display.print(""); 
  display.setCursor(96, 20); display.print("");  

  display.setCursor(4, 36);  display.print("");
  display.setCursor(56, 36); display.print("");
  display.setCursor(96, 36); display.print("");

  display.setCursor(4, 52);  display.print("");
  display.setCursor(56, 52); display.print("");
  display.setCursor(102, 52); display.print("");    

  display.setCursor(4, 68);  display.print("");
  display.setCursor(56, 68); display.print("");
  display.setCursor(108, 68); display.print(""); 
  display.setCursor(2, 68);  display.print("SYSTEM REBOOTING ");  

  display.setCursor(4,84);   display.print("");
  display.setCursor(56,84);  display.print("");
  display.setCursor(108,84); display.print("");
  display.setCursor(4,84);   display.print("..................");
    delay(1500);
    ESP.restart();
  }  
  if (wifiActive) {
    server.handleClient(); 
    if (millis() - wifiStartTime >= WIFI_TIMEOUT_MS) {
      WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); wifiActive = false;          
    }
  }

  if (millis() - lastHeaderChange >= 20000) {
    headerPage++;

    if (headerPage > 1)
        headerPage = 0;

    lastHeaderChange = millis();
}
  // Smooth Scrolling (30ms intervals)
  pm25_val = String(pm25, 1);
  pm10_val = String(pm10, 1);
  hum_val  = String(hum, 1);
  temp_val = String(temp, 1);
  aqi_val  = String(aqi, 1);

  DateTime now = rtc.now();

char dateBuffer[11];
sprintf(dateBuffer,
        "%02d/%02d/%04d",
        now.day(),
        now.month(),
        now.year());

date_val = String(dateBuffer);

char timeBuffer[9];
sprintf(timeBuffer,
        "%02d:%02d:%02d",
        now.hour(),
        now.minute(),
        now.second());

time_val = String(timeBuffer);
  if (millis() - lastSensorRead > 1000)
{
    readSensorRS485();
    lastSensorRead = millis();
}
delay(50); 
updateBuffer();
delay(100); 
}

uint16_t calculateCRC(uint8_t *buffer, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++)
  {
    crc ^= (uint16_t)buffer[pos];
    for (uint8_t i = 8; i != 0; i--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool checkCRC(uint8_t *buffer, uint16_t length)
{
  uint16_t receivedCRC = (buffer[length] & 0xFF) | ((buffer[length + 1] & 0xFF) << 8);
  uint16_t calculatedCRC = calculateCRC(buffer, length);
  return (receivedCRC == calculatedCRC);
}


float modbusFloat(uint8_t *buf)
{
    union
    {
        uint8_t b[4];
        float f;
    } data;

    data.b[3] = buf[0];
    data.b[2] = buf[1];
    data.b[1] = buf[2];
    data.b[0] = buf[3];

    return data.f;
}

bool readSensorRS485() {

  // Clear old buffer
  while (RS485Serial.available()) RS485Serial.read();

  uint8_t req[8];

  req[0] = 0x01; // Sensor ID
  req[1] = 0x03;
  req[2] = 0x00;
  req[3] = 0x00;
  req[4] = 0x00;
  req[5] = 0x0A;

  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;
 
 Serial.print("TX: ");

for(int i=0;i<8;i++)
{
    Serial.printf("%02X ", req[i]);
}

Serial.println();

  RS485Serial.write(req, 8);
  RS485Serial.flush();

  // ===== Wait for response (non-blocking style) =====
  uint8_t resp[32];
  int len = 0;

  unsigned long start = millis();

  while (millis() - start < 1000) { // timeout 200 ms
    if (RS485Serial.available()) {
      resp[len++] = RS485Serial.read();
      if (len >= 32) break;
    }
  }

  if (len == 0) {
    Serial.println("❌ No response from sensor");
    return false;
  }

  // ===== DEBUG PRINT =====
  Serial.print("RX: ");
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", resp[i]);
  }
  Serial.println();

  // ===== CRC CHECK =====
  if (!checkCRC(resp, len - 2)) {
    Serial.print("CRC FAIL | Len=");
  Serial.println(len);
    return false;
  }

  // ===== Parse FLOAT =====
  if (len >= 23)
{
    pm25 = modbusFloat(&resp[3]);
    pm10 = modbusFloat(&resp[7]);
    hum  = modbusFloat(&resp[11]);
    temp = modbusFloat(&resp[15]);
    aqi  = modbusFloat(&resp[19]);

    Serial.println("--------------------------------");
  Serial.printf("PM2.5 : %.1f\n", pm25);
  Serial.printf("PM10  : %.1f\n", pm10);
  Serial.printf("TEMP  : %.1f C\n", temp);
  Serial.printf("HUM   : %.1f %%\n", hum);
  Serial.printf("AQI   : %.1f\n", aqi);
  Serial.println("--------------------------------");

    return true;
}
else
{
    Serial.printf("Invalid response length: %d\n", len);
}
  return false;
}
