#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Preferences.h>
Preferences prefs;
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <HTTPUpdate.h>

// ========================== OTA  Setup ==============================
// Current firmware version
String firmware_version = "2.0";
String version_url = "https://raw.githubusercontent.com/soarmlich/esp32-ota-update/main/version.json";
void firmwareUpdate(String binURL);
String serial_id = "";

// ================== System Reboot ===================================
#define AUTO_REBOOT_INTERVAL (3UL * 60UL * 60UL * 1000UL) // 3 hours
unsigned long bootTime;

// ================================= ODAMS Configration =================

#include <time.h>
// IST offset = 5.30 = 19800 sec 
const long GMT_OFFSET_SEC = 19800;
const int DAYLIGHT_OFFSET_SEC = 0;
//  ===================================Display Config ==================
#include <NextionLite.h>
#include "EasyNextionLibrary.h"
/*      Nextion Dispaly                     */
#define DIS_RX 16
#define DIS_TX 17
HardwareSerial Display(1);
EasyNex hmi(Display);
// ==================================================================
/* ---------------- RS485 ---------------- */
#define RXD1 18
#define TXD1 5
HardwareSerial RS485(2);

/* ---------------- Analog Inputs -------- */

String endChar = String(char(0xff))+String(char(0xff))+String(char(0xff));
String received = "";
String dtd = "";
int code = 0;
int pageno = 0;
int count = 0;

// ===================================================================
float ch1_zero_count = 0.0;
float ch1_span_count = 0.0;
float ch1_zero_range = 0.0;
float ch1_span_range = 0.0;
float ch1_final_value = 0.0;

float ch2_zero_count = 0.0;
float ch2_span_count = 0.0;
float ch2_zero_range = 0.0;
float ch2_span_range = 0.0;
float ch2_final_value = 0.0;

float ch3_zero_count = 0.0;
float ch3_span_count = 0.0;
float ch3_zero_range = 0.0;
float ch3_span_range = 0.0;
float ch3_final_value = 0.0;

float ch4_zero_count = 0.0;
float ch4_span_count = 0.0;
float ch4_zero_range = 0.0;
float ch4_span_range = 0.0;
float ch4_final_value = 0.0;

float ch5_zero_count = 0.0;
float ch5_span_count = 0.0;
float ch5_zero_range = 0.0;
float ch5_span_range = 0.0;
float ch5_final_value = 0.0;

String ch1_name = "";
String ch1_unit = "";
int ch1_status = 0;

String ch2_name = "";
String ch2_unit = "";
int ch2_status = 0;

String ch3_name = "";
String ch3_unit = "";
int ch3_status = 0;

String ch4_name = "";
String ch4_unit = "";
int ch4_status = 0;

String ch5_name = "";
String ch5_unit = "";
int ch5_status = 0;

int reboot = 0;
int modbus_id = 0;
String machine_name = "";

int year = 0;
int month = 0;
int day = 0;
int hour = 0;
int minute = 0;
int second = 0;

double out_total = 0.0;
double out_flow =0.0;
double out_dailyTotal = 0.0;
double out_flowTotal = 0.0;
double out_lastNightTotal = 0.0;
double out_yesterday_total = 0.0;

double in_total = 0.0;
double in_flow = 0.0;
double in_dailyTotal = 0.0;
double in_flowTotal = 0.0;
double in_lastNightTotal = 0.0;



int relay_button = 0;
int resetDay = 25, resetMonth = 5;
int ntp_year = 0, ntp_month = 0, ntp_day = 0, ntp_hour = 0, ntp_minute = 0, ntp_second = 0;


unsigned long lastCommandCheck = 0;
const unsigned long COMMAND_INTERVAL = 60000;  // check every 60s

bool savedToday = false;


#define ADC1 13
#define ADC2 12
#define ADC3 14

#define RELAY_PIN 4 //  Relay pin mode 

// ---------------- Modbus Settings -------------------
byte SLAVE_ID = 001;
const byte FUNC_READ = 0x03;
const byte START_HIGH = 0x07;
const byte START_LOW  = 0xd0;
const byte COUNT_HIGH = 0x00;
const byte COUNT_LOW  = 0x14;  // Read 10 registers
const byte START_REG = 0x20;
uint8_t machine_id1 = 1;
uint8_t machine_id2 = 2;


// ======================== Modbus =======================
double parseToDouble(uint8_t *data){

  union{
    uint8_t b[8];
    double d;
  } u;
  for(int i = 0; i<8 ;i++){
    u.b[7-i] = data[i];
  }
  return (double)u.d;
}

float parseFloat(uint8_t *data){
  uint8_t temp[4];
  temp[0] = data[3];
  temp[1] = data[2];
  temp[2] = data[1];
  temp[3] = data[0];
  float value;
  memcpy(&value , temp,4);
  return value;
}

float bytesToFloat(uint8_t *data){
  uint32_t combined = 
  ((uint32_t)data[0]<<24)|
  ((uint32_t)data[1]<<16)|
  ((uint32_t)data[2]<<8)|
  ((uint32_t)data[3]);
  float value;
  memcpy(&value,&combined,sizeof(value));
  return value;
}
int bytesToInt(uint8_t *data){
  uint32_t combined = 
  ((uint32_t)data[0]<<24)|
  ((uint32_t)data[1]<<16)|
  ((uint32_t)data[2]<<8)|
  ((uint32_t)data[3]);
  return combined;
}


double parseFloatToDouble(uint8_t *data){

  union{
    uint8_t b[4];
    float f;
  }u;
  u.b[3] = data[0];
  u.b[2]=data[1];
  u.b[1]= data[2];
  u.b[0]= data[3];
  return( double)u.f;
}
float modbusFloat(uint8_t *data)
{
 uint8_t temp[4];
temp[0] = data[2];
temp[1] = data[3];
temp[2] = data[0];
temp[3] = data[1];
 float value;
 memcpy(&value, temp, 4);
 return value;
}
/* ---------------- WiFi ---------------- */
String ssid     = "SOARMLICH E";
String password = "Soarmlich1193";
const char* SERVER_URL   = "http://141.148.208.131/api/data";
String SERVER_RELAY      = "http://141.148.208.131/api/relay/status/";
String SERVER_COMMAND    = "http://141.148.208.131/api/device/command";
String MACHINE_ID = "Server_test_machine";
WiFiClient wifiClient;
String Relay_status = "";

/* ---------------- Sensor Data ---------- */
int SOx = 0, NOx = 0, PM = 0;
int ADC_A = 0, ADC_B = 0, ADC_C = 0;
int CH1=0,CH2=0,CH3=0,CH4=0,CH5=0;
// ===================================================================

void constructRequest( uint8_t &device_id,byte *frame) {
  frame[0] = device_id;
  frame[1] = FUNC_READ;
  frame[2] = START_HIGH;
  frame[3] = START_LOW;
  frame[4] = COUNT_HIGH;
  frame[5] = COUNT_LOW;
  uint16_t crc = calcCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;
}

// ===================================================================
int readResponse( byte *buf, int maxLen) {
  int i = 0;
  unsigned long start = millis();
  while ((millis() - start) < 300) {
    if (RS485.available()) {
      buf[i++] = RS485.read();
      if (i >= maxLen) break;
    }
  }
  return i;
}

// ===================================================================
bool verifyCRC(byte *frame, int len) {
  uint16_t received = (frame[len - 1] << 8) | frame[len - 2];
  return calcCRC(frame, len - 2) == received;
}
uint16_t calcCRC(byte *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 8; i; i--) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// ====================================================================
bool readModbus(uint8_t &device_id,double &flow, double &total) {

//  uint8_t req[8]={ device_id, 0x03, 0x00, 0x00, 0x00, 0x14, 0x45, 0x48}; 
  // Clear junk
  uint8_t req[8];
  constructRequest(device_id,req);
  while (RS485.available()) RS485.read();

  RS485.write(req,8);
  RS485.flush();
  delay(20);
 
  uint8_t buf[64];
  int len = 0;
  unsigned long t0 = millis();

  while (millis() - t0 < 500) {
    while (RS485.available() && len < sizeof(buf)) {
      buf[len++] = RS485.read();
    }
  }

  delay(50);

  if (len <8) {
    Serial.println("❌ No Modbus data");
    return false;
  }

  // DEBUG RAW RX
  Serial.print("RX: ");
  for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();

  // Python equivalent:
  // final_data = buf[3:]
  // flow  = float(final_data[4:8])
  // total = float(final_data[?])
  // Safety: check minimum index
  if (len < 25) return false;
  if(buf[0] != 0x01 && buf[1] != 0x03) return false;

  float reg1 =bytesToFloat(&buf[(5)]);
  double reg9 = parseToDouble(&buf[9]);

  uint32_t totalInt =
((uint32_t)buf[17] << 24) |
((uint32_t)buf[18] << 16) |
((uint32_t)buf[15] << 8)  |
buf[16];
  float totalFrac = modbusFloat(&buf[12]);

  flow = reg1;
  total = totalInt;

  Serial.print(" Flow rate :");
  Serial.println(flow,6);

  Serial.print(" Flow Total: ");
  Serial.println(total,6);
  return true;
}

// ================================================================================

String getTimestamp() {
  
  if (year < 2024 || year > 2035) {
    Serial.println("⚠️ Invalid RTC time detected");
    return "2026-01-01 00:00:00";
  }
  char buf[20];
  snprintf(buf, sizeof(buf),
           "%04d-%02d-%02d %02d:%02d:%02d",
           year, month, day,
           hour, minute, second);
  return String(buf);
  
}
// ==================================================================================
void checkDeviceCommands() {
  if (millis() - lastCommandCheck < COMMAND_INTERVAL) return;
  lastCommandCheck = millis();

  HTTPClient http;
  String url = SERVER_COMMAND + "?machine_id=" + MACHINE_ID;
  http.begin(wifiClient, url);
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, payload);
    if (!err && doc.containsKey("action")) {
      String action = doc["action"];
      if (action == "set_totalizer" && doc.containsKey("payload")) {
        JsonObject p = doc["payload"];
        double rawtotal = p["raw_total_m3"];
        double rawdailyTotal = 0;
        if (p.containsKey("today_totalizer_m3"))
          rawdailyTotal = p["today_totalizer_m3"];
        double rawlastNightTotal = rawtotal - rawdailyTotal;
        out_total = rawtotal;  
        out_lastNightTotal = rawlastNightTotal;
        prefs.putDouble("out_lastNightTotal", out_lastNightTotal);
        prefs.putDouble("out_total", out_total);
        Serial.println("Totalizer set from SCADA: " + String(rawtotal));
      }
      else if (action == "reboot") {
        Serial.println("Reboot command received, restarting...");
        Display.print("rest");
        Display.write(0xFF);
        Display.write(0xFF);
        Display.write(0xFF);
        delay(500);
        ESP.restart();
      }
      // Acknowledge
      http.end();
      http.begin(wifiClient, SERVER_COMMAND + "/ack?machine_id=" + MACHINE_ID);
      http.POST("{\"status\":\"applied\"}");
    }
  }
  http.end();
}

// ==================================================================================
void uploadToServer(const String &payload) {
    HTTPClient http;
    http.begin(wifiClient, SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(payload);
    Serial.print("WiFi HTTP Code: ");
    Serial.println(code);
    if(code == 200){
      hmi.writeNum("t21.pco", 2016);
    }
    else if(code == -1){
      hmi.writeNum("t21.pco", 63488);
    }
    http.end();
}
//==================================================================================
void getRelayStatus(){
  Serial.println(" Getting Status of Relay ");
  String relay_url = SERVER_RELAY+MACHINE_ID;
  HTTPClient http;
  http.begin(wifiClient,relay_url);
  int code = http.GET();
  Serial.print(" WiFi HTTP Code :   ");
  Serial.println(code);

  if (code == 200) {
    String payload = http.getString();
    Serial.print("Server Payload:  ");
    Serial.println(payload);

  // Parse JSON
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (err) {
      Serial.print("JSON Error: ");
      Serial.println(err.c_str());
      http.end();
      return;
    }

    String relayState = doc["relay_state"];     // "ON" or "OFF"
    float safetyLimit = doc["safety_limit"];    // optional

    Serial.print("Relay State: ");
    Serial.println(relayState);
    Serial.print("Safety Limit: ");
    Serial.println(safetyLimit);

    relay_button = hmi.readNumber("Relay1.bt0.val");

    Serial.print( " Relay  button status : ");Serial.println(relay_button);

    if(out_dailyTotal < safetyLimit){
    // Actuate relay
    if (relayState == "ON") {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("🔌 Relay TURNED ON");
      hmi.writeNum("t23.pco", 2016);
    }
    else if(relay_button == 1){
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("🔌 Relay TURNED ON");
      

    } else {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("🔌 Relay TURNED OFF");
      hmi.writeNum("t23.pco", 63488);
    }
    }
    else{
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("🔌 Relay TURNED OFF");
      hmi.writeNum("t23.pco", 63488);
    }
  http.end(); 
  }
}

// ========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(" System  Getting started !.......");
  
  Display.begin(9600,SERIAL_8N1,DIS_RX,DIS_TX);
  delay(10000);
  
  RS485.begin(9600, SERIAL_8N1,RXD1,TXD1);
  delay(1000);
  loaddata();
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  for (int i = 0; i < 50 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected");
  } else {
    Serial.println("\n⚠️ WiFi failed → Rebooting ");
  }
  delay(500); 
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);

  String cmd2 = " Current Firmware : " +firmware_version;
  Serial.println(cmd2);
  if (WiFi.status() == WL_CONNECTED) {
  checkForUpdates();
  }
  // =======================Auto Reboot Setup ================================
  bootTime = millis();
  getdate();
  Serial.print( " Auto reboot will happen at  : ");
  Serial.print( (hour+3));Serial.print(":");Serial.print(minute);Serial.print(":");Serial.print(second);
  Serial.println("");

  prefs.begin("total", false);
  out_lastNightTotal = prefs.getDouble("out_lastNightTotal",out_lastNightTotal);
  if(isnan(out_lastNightTotal))out_lastNightTotal = 0.0;
  Serial.print("Saved Flow Total OUT  : ");Serial.println(out_lastNightTotal);
  delay(50);
  out_yesterday_total = prefs.getDouble("out_yesterday_total",out_yesterday_total);
  if(isnan(out_yesterday_total))out_yesterday_total=0.0;
  Serial.print(" Saved Yesterday Total :  ");Serial.println(out_yesterday_total);
  delay(50);
  if(!prefs.isKey("firstReset")){
    resetDay = 11;
    resetMonth = 6;
    prefs.putInt("resetDay",resetDay);
    prefs.putInt("resetMonth",resetMonth);
  }

  resetDay   = prefs.getInt("resetDay");
  resetMonth = prefs.getInt("resetMonth");
  Serial.print(" Saved Reset Day :");Serial.println(resetDay);
  Serial.print(" Saved Reset Month : ");Serial.println(resetMonth);
  delay(50);
  in_lastNightTotal = prefs.getDouble("in_lastNightTotal",in_lastNightTotal);
  if(isnan(in_lastNightTotal))in_lastNightTotal = 0.0;
  Serial.print("Saved Flow Total IN   : ");Serial.println(in_lastNightTotal);
  delay(1000);
  
}

// ===============================================================================
void loop() {

  //  prefs.putDouble("yesterday_total",10);

  //  ==================Reboot =====================================================
  if (millis() - bootTime > AUTO_REBOOT_INTERVAL) {
    Serial.println("⏱️ Scheduled reboot");
    Display.print("rest");
    Display.write(0xFF);
    Display.write(0xFF);
    Display.write(0xFF);
    delay(500);
    ESP.restart();
  }

  // =================================================================================

  SLAVE_ID = modbus_id;
  MACHINE_ID = hmi.readStr("Deviceip.t8.txt");

  if(millis() - bootTime < 120000){
  String cmd1 = " Current Firmware : " +firmware_version;
  String cmd = cmd1 + "    " + cmd1 + "     " + cmd1;
  hmi.writeNum("button.bt2.val", 1);
  hmi.writeNum("g1.pco", 65535);
  hmi.writeStr("g1.txt",cmd);
  }
  else{
    hmi.writeNum("button.bt2.val", 0);
    hmi.writeStr("g1.txt","");
  }
  if(WiFi.status()== WL_CONNECTED){
    hmi.writeNum("t20.pco", 2016);
  }
  else if (WiFi.status()!= WL_CONNECTED){
    hmi.writeNum("t20.pco", 63488);
  }

  Serial.print(" Slave Id : "); Serial.println(SLAVE_ID);
  Serial.print(" Machine  Name : "); Serial.println(MACHINE_ID);

  // ======================= Read from RS485_A =====================================
  Serial.println("Reading RS485...");
  if (readModbus(machine_id1,out_flow, out_total)) {
  Serial.printf("Flow: %.3f m3/h\n", out_flow);
  Serial.printf("Total: %.3f\n", out_total);
}
if (readModbus(machine_id2,in_flow, in_total)) {
  Serial.printf("Flow: %.3f m3/h\n", in_flow);
  Serial.printf("Total: %.3f\n", in_total);
}
  delay(500);

  year = hmi.readNumber("rtc0");
  month = hmi.readNumber("rtc1");
  day = hmi.readNumber("rtc2");
  hour = hmi.readNumber("rtc3");
  minute = hmi.readNumber("rtc4");
  second = hmi.readNumber("rtc5");

  StaticJsonDocument<512> doc;
  doc["machine_id"] = MACHINE_ID;

  doc["timestamp"]  = getTimestamp();
  doc["Serial_no"] = serial_id;
  doc["Current_version"] = firmware_version;
  doc["sensor_type"] = "RS485_avadh_dual";
  JsonObject ch = doc.createNestedObject("channels");

  ch1_name = hmi.readStr("Parameter1.t22.txt");
  ch1_unit = hmi.readStr("Parameter1.t20.txt");
  ch1_status = hmi.readNumber("Parameter1.bt0.val");

  ch2_name = hmi.readStr("Parameter1.t8.txt");
  ch2_unit = hmi.readStr("Parameter1.t21.txt");
  ch2_status = hmi.readNumber("Parameter1.bt1.val");

  ch3_name = hmi.readStr("Parameter1.t9.txt");
  ch3_unit = hmi.readStr("Parameter1.t23.txt");
  ch3_status = hmi.readNumber("Parameter1.bt2.val");

  ch4_name = hmi.readStr("Parameter1.t10.txt");
  ch4_unit = hmi.readStr("Parameter1.t24.txt");
  ch4_status = hmi.readNumber("Parameter1.bt3.val");

  ch5_name = hmi.readStr("Parameter1.t4.txt");
  ch5_unit = hmi.readStr("Parameter1.t25.txt");
  ch5_status = hmi.readNumber("Parameter1.bt4.val");

  
  modbus_id = hmi.readNumber("Deviceip.n0.val");
  machine_name = hmi.readStr("Deviceip.t8.txt");

  Serial.print(" Machine Name : ");Serial.println(machine_name);
  Serial.print(" Modbus ID  : ");Serial.println(modbus_id);
  
  reboot = hmi.readNumber("Deviceip.bt1.val");
  Serial.print(" Reboot Status  : ");Serial.println(reboot);
  if(reboot==1){
    Display.print("rest");
    Display.write(0xFF);
    Display.write(0xFF);
    Display.write(0xFF);
    delay(500);
    ESP.restart();
  }

  Serial.println("");

  Serial.print("CH1_name : ");Serial.println(ch1_name);
  Serial.print("CH1_unit : ");Serial.println(ch1_unit);
  Serial.print("CH1_Status : ");Serial.println(ch1_status);

  Serial.print("CH2_name : ");Serial.println(ch2_name);
  Serial.print("CH2_unit : ");Serial.println(ch2_unit);
  Serial.print("CH2_Status : ");Serial.println(ch2_status);

  Serial.print("CH3_name : ");Serial.println(ch3_name);
  Serial.print("CH3_unit : ");Serial.println(ch3_unit);
  Serial.print("CH3_Status : ");Serial.println(ch3_status);

  Serial.print("CH4_name : ");Serial.println(ch4_name);
  Serial.print("CH4_unit : ");Serial.println(ch4_unit);
  Serial.print("CH4_Status : ");Serial.println(ch4_status);

  Serial.print("CH5_name : ");Serial.println(ch5_name);
  Serial.print("CH5_unit : ");Serial.println(ch5_unit);
  Serial.print("CH5_Status : ");Serial.println(ch5_status);

  if(ch1_status==1){
    // ch1_final_value = map(SOx,ch1_zero_count,ch1_span_count,ch1_zero_range,ch1_span_range);
    ch1_final_value = out_flow;
    ch["ch1"]["name"] = ch1_name;
    ch["ch1"]["unit"] = ch1_unit;
    ch["ch1"]["value"] = ch1_final_value;

  }else{
    ch1_final_value = 0;
  }
  if(ch2_status==1){
    // ch2_final_value = map(NOx,ch2_zero_count,ch2_span_count,ch2_zero_range,ch2_span_range);
    ch2_final_value =  out_total-out_lastNightTotal;
    ch["ch2"]["name"] = ch2_name;
    ch["ch2"]["unit"] = ch2_unit;
    ch["ch2"]["value"] = ch2_final_value;
  }else{
    ch2_final_value = 0;
  }

  if(ch3_status==1){
    // ch3_final_value = map(PM,ch3_zero_count,ch3_span_count,ch3_zero_range,ch3_span_range);
    ch3_final_value = out_yesterday_total;
    ch["ch3"]["name"] = ch3_name;
    ch["ch3"]["unit"] = ch3_unit;
    ch["ch3"]["value"] = ch3_final_value;
  }else{
    ch3_final_value = 0;
  }

  if(ch4_status==1){
    // ch4_final_value = map(CH4,ch4_zero_count,ch4_span_count,ch4_zero_range,ch4_span_range);
    ch4_final_value = in_flow;
    ch["ch4"]["name"] = ch4_name;
    ch["ch4"]["unit"] = ch4_unit;
    ch["ch4"]["value"] = ch4_final_value;
  }else{
    ch4_final_value = 0;
  }

  if(ch5_status==1){
    // ch5_final_value = map(CH5,ch5_zero_count,ch5_span_count,ch5_zero_range,ch5_span_range);
  ch5_final_value =  in_total-in_lastNightTotal;  
  ch["ch5"]["name"] = ch5_name;
  ch["ch5"]["unit"] = ch5_unit;
  ch["ch5"]["value"] = ch5_final_value;
  }else{
    ch5_final_value = 0;
  }


  hmi.writeStr("Calibration.t9.txt", String(out_flow));
  hmi.writeStr("Calibration.t49.txt", String(ch1_final_value));

  hmi.writeStr("Calibration.t10.txt", String(out_lastNightTotal));
  hmi.writeStr("Calibration.t50.txt", String(ch2_final_value));
  
  hmi.writeStr("Calibration.t11.txt", String(out_total));
  hmi.writeStr("Calibration.t51.txt", String(ch3_final_value));

  hmi.writeStr("Calibration.t12.txt", String(CH4));
  hmi.writeStr("Calibration.t52.txt", String(ch4_final_value));

  hmi.writeStr("Calibration.t13.txt", String(CH5));
  hmi.writeStr("Calibration.t53.txt", String(ch5_final_value));
  out_dailyTotal = out_total - out_lastNightTotal;
  in_dailyTotal = in_total - in_lastNightTotal;
  getRelayStatus();

  Serial.print(" Sent parameter 1 :");Serial.println(ch1_final_value);
  Serial.print(" Sent parameter 2 :");Serial.println(ch2_final_value);
  Serial.print(" Sent parameter 3 :");Serial.println(ch3_final_value);
  Serial.print(" Sent parameter 4 :");Serial.println(ch4_final_value);
  Serial.print(" Sent parameter 5 :");Serial.println(ch5_final_value);

  hmi.writeStr("Main.t49.txt", String(ch1_final_value));
  hmi.writeStr("Main.t50.txt", String(ch2_final_value));
  hmi.writeStr("Main.t51.txt", String(ch3_final_value));
  hmi.writeStr("Main.t52.txt", String(ch4_final_value));
  hmi.writeStr("Main.t53.txt", String(ch5_final_value));

  if(minute%1 == 0 && second>= 0 && second<=10){
  String payload;
  serializeJson(doc, payload);
  Serial.println("\nSending Payload:");
  Serial.println(payload);
  uploadToServer(payload);
  }
  
  // Daily reset logic
  if (month > resetMonth || (month == resetMonth && day > resetDay)) {
    prefs.putDouble("out_yesterday_total", out_dailyTotal);
    out_yesterday_total = out_dailyTotal;
    Serial.print("Yesterday total saved: "); Serial.println(out_dailyTotal);
  delay(50);
      out_lastNightTotal = out_total;
      prefs.putDouble("out_lastNightTotal", out_total);
      Serial.print("lastNightTotal updated: "); Serial.println(out_total);
    
  delay(50);
      in_lastNightTotal = in_total;
      prefs.putDouble("in_lastNightTotal", in_total);
      Serial.print("lastNightTotal updated: "); Serial.println(in_total);
    
    out_dailyTotal = 0.0;
    in_dailyTotal = 0.0;
    resetDay = ntp_day;
    resetMonth = ntp_month;
    prefs.putInt("resetDay", ntp_day);
    prefs.putInt("resetMonth", ntp_month);
    prefs.putBool("firstReset", true);
    Serial.println("Reset complete – new reset point: " + String(day) + "/" + String(month));
  }

  checkDeviceCommands();
  // lastNightTotal = prefs.getUShort("lastNightTotal", lastNightTotal);
  Serial.print("Saved Flow Total  : ");Serial.println(out_lastNightTotal);

  delay(1000);
}

void getdate(){
  year = hmi.readNumber("rtc0");
  month = hmi.readNumber("rtc1");
  day = hmi.readNumber("rtc2");
  hour = hmi.readNumber("rtc3");
  minute = hmi.readNumber("rtc4");
  second = hmi.readNumber("rtc5");
}

void loaddata(){

  ch1_name = hmi.readStr("Parameter1.t22.txt");
  ch1_unit = hmi.readStr("Parameter1.t20.txt");
  ch1_status = hmi.readNumber("Parameter1.bt0.val");

  ch2_name = hmi.readStr("Parameter1.t8.txt");
  ch2_unit = hmi.readStr("Parameter1.t21.txt");
  ch2_status = hmi.readNumber("Parameter1.bt1.val");

  ch3_name = hmi.readStr("Parameter1.t9.txt");
  ch3_unit = hmi.readStr("Parameter1.t23.txt");
  ch3_status = hmi.readNumber("Parameter1.bt2.val");

  ch4_name = hmi.readStr("Parameter1.t10.txt");
  ch4_unit = hmi.readStr("Parameter1.t24.txt");
  ch4_status = hmi.readNumber("Parameter1.bt3.val");

  ch5_name = hmi.readStr("Parameter1.t4.txt");
  ch5_unit = hmi.readStr("Parameter1.t25.txt");
  ch5_status = hmi.readNumber("Parameter1.bt4.val");

  reboot = hmi.readNumber("Deviceip.bt1.val");
  modbus_id = hmi.readNumber("Deviceip.n0.val");
  machine_name = hmi.readStr("Relay1.t1.txt");

  ch1_zero_count = hmi.readNumber("Calibration.n0.val");
  ch1_span_count = hmi.readNumber("Calibration.n8.val");
  ch1_zero_range = hmi.readNumber("Calibration.n16.val");
  ch1_span_range = hmi.readNumber("Calibration.n24.val");

  ch2_zero_count = hmi.readNumber("Calibration.n1.val");
  ch2_span_count = hmi.readNumber("Calibration.n9.val");
  ch2_zero_range = hmi.readNumber("Calibration.n17.val");
  ch2_span_range = hmi.readNumber("Calibration.n25.val");

  ch3_zero_count = hmi.readNumber("Calibration.n2.val");
  ch3_span_count = hmi.readNumber("Calibration.n10.val");
  ch3_zero_range = hmi.readNumber("Calibration.n18.val");
  ch3_span_range = hmi.readNumber("Calibration.n26.val");

  ch4_zero_count = hmi.readNumber("Calibration.n3.val");
  ch4_span_count = hmi.readNumber("Calibration.n11.val");
  ch4_zero_range = hmi.readNumber("Calibration.n19.val");
  ch4_span_range = hmi.readNumber("Calibration.n27.val");

  ch5_zero_count = hmi.readNumber("Calibration.n4.val");
  ch5_span_count = hmi.readNumber("Calibration.n12.val");
  ch5_zero_range = hmi.readNumber("Calibration.n20.val");
  ch5_span_range = hmi.readNumber("Calibration.n28.val");

  year = hmi.readNumber("rtc0");
  month = hmi.readNumber("rtc1");
  day = hmi.readNumber("rtc2");
  hour = hmi.readNumber("rtc3");
  minute = hmi.readNumber("rtc4");
  second = hmi.readNumber("rtc5");

  SLAVE_ID = modbus_id;
  MACHINE_ID = hmi.readStr("Deviceip.t8.txt");
  ssid     = hmi.readStr("Deviceip.t6.txt");
  password = hmi.readStr("Deviceip.t7.txt");
  relay_button = hmi.readNumber("Relay1.bt0.val");

  Serial.print(" WiFi Name : ");Serial.println(ssid);
  Serial.print(" WiFi Password : ");Serial.println(password);
  serial_id = hmi.readStr("About.t3.txt");
  Serial.print("Serial_id: ");Serial.println(serial_id);
  delay(500);
}
void checkForUpdates() {

  HTTPClient http;

  Serial.println("Checking for updates...");

  http.begin(version_url);

  int httpCode = http.GET();

  if (httpCode == 200) {

    String payload = http.getString();

    Serial.println(payload);

    DynamicJsonDocument doc(1024);

    deserializeJson(doc, payload);

    String latest_version = doc["version"];
    String firmware_url = doc["firmware"];
    String target_device = doc["device"];
    

    Serial.println("Latest Version: " + latest_version);
    Serial.println("Target Device: " + target_device);
    Serial.println("My Device: " + serial_id);

    if (latest_version != firmware_version && target_device == serial_id ) {

      Serial.println("New Firmware Detected");

      firmwareUpdate(firmware_url);

    } else {

      Serial.println("Device Already Up-To-Date");
    }

  } else {

    Serial.println("Version Check Failed");
  }

  http.end();
}
void firmwareUpdate(String binURL) {

  Serial.println("Starting OTA Update");

  WiFiClientSecure client;
  client.setInsecure();   // VERY IMPORTANT

  t_httpUpdate_return ret = httpUpdate.update(client, binURL);

  switch (ret) {

    case HTTP_UPDATE_FAILED:
      Serial.printf("Update Failed Error (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("No Updates");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("Update Success");
      break;
  }
}   