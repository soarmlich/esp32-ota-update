
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
#define AUTO_REBOOT_INTERVAL (6UL * 60UL * 60UL * 1000UL) // 6 hours
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

// NTP time backup – used to protect the midnight reset from HMI clock drift
bool ntpValid = false;
int ntp_year = 0, ntp_month = 0, ntp_day = 0, ntp_hour = 0, ntp_minute = 0, ntp_second = 0;

double total = 0.0;
double flow = 0.0;
double dailyTotal = 0.0;
double flowTotal = 0.0;
double lastNightTotal = 0.0;
double yesterday_total = 0.0;

float Qmax = 100.0;
bool systemON = false;
unsigned long  lastTime = 0;
unsigned long  now = 0;
unsigned long  lastSaveTime = 0;

static bool savedToday = false;


bool intialTime = false;

int relay_button = 0;
int resetDay = 25, resetMonth = 5;

unsigned long lastCommandCheck = 0;
const unsigned long COMMAND_INTERVAL = 60000; 

static float lastFlow = 15;

/* ---------------- Analog Inputs -------- */
#define ADC1 34
#define ADC2 35
#define ADC3 32

#define RELAY_PIN 4 //  Relay pin mode 

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
int CH1=0,CH2=0,CH3=0,CH4=0,CH5=0;

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
// =================================================================================

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
      // ===== FIX: read values from nested "payload" object =====
      if (action == "set_totalizer" && doc.containsKey("payload")) {
        JsonObject p = doc["payload"];
        double rawtotal = p["raw_total_m3"];
        double rawdailyTotal = 0;
        if (p.containsKey("today_totalizer_m3"))
          rawdailyTotal = p["today_totalizer_m3"];
        double rawlastNightTotal = rawtotal - rawdailyTotal;
        total = rawtotal;
        lastNightTotal = rawlastNightTotal;
        lastTime = 0;   // ===== FIX: reset analog integrator =====
        prefs.putDouble("lastNightTotal", lastNightTotal);
        prefs.putDouble("total", total);
        Serial.println("Totalizer set from SCADA: " + String(rawtotal));
      }
      else if (action == "reboot") {
        Serial.println("Reboot command received, restarting...");
        Serial.println("⏱️ Scheduled reboot");
        Display.print("rest");
        Display.write(0xFF);
        Display.write(0xFF);
        Display.write(0xFF);
        delay(500);
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
    http.setTimeout(10000);
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
  // Serial.println(" Getting Status of Relay ");
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

    if(dailyTotal < safetyLimit){
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

  // ─────────────────────────────────────────────────────────────────
  // NTP time sync – added for accurate midnight reset
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 10000)) {
    Serial.println("NTP synced: " + String(asctime(&timeinfo)));
    // Save NTP time so midnight logic uses it
    ntp_year   = timeinfo.tm_year + 1900;
    ntp_month  = timeinfo.tm_mon + 1;
    ntp_day    = timeinfo.tm_mday;
    ntp_hour   = timeinfo.tm_hour;
    ntp_minute = timeinfo.tm_min;
    ntp_second = timeinfo.tm_sec;
    ntpValid = true;
    // Also set the HMI display to the synced time
    hmi.writeNum("rtc0", ntp_year);
    hmi.writeNum("rtc1", ntp_month);
    hmi.writeNum("rtc2", ntp_day);
    hmi.writeNum("rtc3", ntp_hour);
    hmi.writeNum("rtc4", ntp_minute);
    hmi.writeNum("rtc5", ntp_second);
  } else {
    Serial.println("NTP failed – using HMI time fallback");
    ntpValid = false;
  }
  // ─────────────────────────────────────────────────────────────────

  pinMode(RELAY_PIN,OUTPUT);
  pinMode(ADC1,INPUT);
  pinMode(ADC2,INPUT);
  pinMode(ADC3,INPUT);
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
  lastNightTotal = prefs.getDouble("lastNightTotal",lastNightTotal);
  if(isnanf(lastNightTotal))lastNightTotal=0;
  Serial.print("Saved  last Day Flow Total  : ");Serial.println(lastNightTotal);
  total =  prefs.getDouble("total", total);
  if(isnanf(total))total=0;
  Serial.print("Saved Flow Total  : ");Serial.println(total);
  yesterday_total = prefs.getDouble("yesterday_total",yesterday_total);
  if(isnan(yesterday_total))yesterday_total=0.0;
  if(!prefs.isKey("firstReset")){
    resetDay = 6;
    resetMonth = 6;
    prefs.putInt("resetday",resetDay);
    prefs.putInt("resetMonth",resetMonth);
  }
  resetDay   = prefs.getInt("resetDay",   25);
  resetMonth = prefs.getInt("resetMonth", 5);
  Serial.print("Saved Flow Total: "); Serial.println(lastNightTotal);
  delay(1000);
  
}

// ===============================================================================
void loop() {

  // prefs.putDouble("lastNightTotal", 0.0);
  // prefs.putDouble("total", 0.0);
  // prefs.putDouble("yesterday_total",2.0);

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

  // ─────────────────────────────────────────────────────────────────
  // Periodic NTP re‑sync every 5 minutes
  static unsigned long lastNtpUpdate = 0;
  if (millis() - lastNtpUpdate > 300000) {
    struct tm dummy;
    if (getLocalTime(&dummy, 5000)) {
      ntp_year   = dummy.tm_year + 1900;
      ntp_month  = dummy.tm_mon + 1;
      ntp_day    = dummy.tm_mday;
      ntp_hour   = dummy.tm_hour;
      ntp_minute = dummy.tm_min;
      ntp_second = dummy.tm_sec;
      ntpValid = true;
      lastNtpUpdate = millis();
    } else {
      ntpValid = false;
    }
  }
  // ─────────────────────────────────────────────────────────────────

  // =================================================================================

  MACHINE_ID = hmi.readStr("Deviceip.t8.txt");
  Serial.print(" Machine  Name : "); Serial.println(MACHINE_ID);
  delay(500);

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

  year = hmi.readNumber("rtc0");
  month = hmi.readNumber("rtc1");
  day = hmi.readNumber("rtc2");
  hour = hmi.readNumber("rtc3");
  minute = hmi.readNumber("rtc4");
  second = hmi.readNumber("rtc5");

  // If NTP is valid, use its time for the midnight reset (keep HMI display untouched)
  if (ntpValid) {
    year   = ntp_year;
    month  = ntp_month;
    day    = ntp_day;
    hour   = ntp_hour;
    minute = ntp_minute;
    second = ntp_second;
  }

  StaticJsonDocument<512> doc;
  doc["machine_id"] = MACHINE_ID;
  doc["timestamp"]  = getTimestamp();
  doc["Serial_no"] = serial_id;
  doc["Current_version"] = firmware_version;
  doc["sensor_type"] = "4-20mA single";
  JsonObject ch = doc.createNestedObject("channels");

  // ===== CALIBRATION VALUES ARE NOT RE‑READ IN LOOP (original behaviour) =====
  // They are loaded once in loaddata(). Reboot after calibration changes.
  // ch1_zero_count = hmi.readNumber("Calibration.n0.val");
  // ch1_span_count = hmi.readNumber("Calibration.n8.val");
  // ch1_zero_range = hmi.readNumber("Calibration.n16.val");
  // ch1_span_range = hmi.readNumber("Calibration.n24.val");
  // ch2_zero_count = hmi.readNumber("Calibration.n1.val");
  // ch2_span_count = hmi.readNumber("Calibration.n9.val");
  // ch2_zero_range = hmi.readNumber("Calibration.n17.val");
  // ch2_span_range = hmi.readNumber("Calibration.n25.val");
  // ch3_zero_count = hmi.readNumber("Calibration.n2.val");
  // ch3_span_count = hmi.readNumber("Calibration.n10.val");
  // ch3_zero_range = hmi.readNumber("Calibration.n18.val");
  // ch3_span_range = hmi.readNumber("Calibration.n26.val");
  // ch4_zero_count = hmi.readNumber("Calibration.n3.val");
  // ch4_span_count = hmi.readNumber("Calibration.n11.val");
  // ch4_zero_range = hmi.readNumber("Calibration.n19.val");
  // ch4_span_range = hmi.readNumber("Calibration.n27.val");
  // ch5_zero_count = hmi.readNumber("Calibration.n4.val");
  // ch5_span_count = hmi.readNumber("Calibration.n12.val");
  // ch5_zero_range = hmi.readNumber("Calibration.n20.val");
  // ch5_span_range = hmi.readNumber("Calibration.n28.val");

  // ch1_name = hmi.readStr("Parameter1.t22.txt");
  // ch1_unit = hmi.readStr("Parameter1.t20.txt");
  // ch1_status = hmi.readNumber("Parameter1.bt0.val");

  // ch2_name = hmi.readStr("Parameter1.t8.txt");
  // ch2_unit = hmi.readStr("Parameter1.t21.txt");
  // ch2_status = hmi.readNumber("Parameter1.bt1.val");

  // ch3_name = hmi.readStr("Parameter1.t9.txt");
  // ch3_unit = hmi.readStr("Parameter1.t23.txt");
  // ch3_status = hmi.readNumber("Parameter1.bt2.val");

  // ch4_name = hmi.readStr("Parameter1.t10.txt");
  // ch4_unit = hmi.readStr("Parameter1.t24.txt");
  // ch4_status = hmi.readNumber("Parameter1.bt3.val");

  // ch5_name = hmi.readStr("Parameter1.t4.txt");
  // ch5_unit = hmi.readStr("Parameter1.t25.txt");
  // ch5_status = hmi.readNumber("Parameter1.bt4.val");

  
  modbus_id = hmi.readNumber("Deviceip.n0.val");
  machine_name = hmi.readStr("Deviceip.t8.txt");

  // Serial.print(" Machine Name : ");Serial.println(machine_name);
  // Serial.print(" Modbus ID  : ");Serial.println(modbus_id);
  
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

//  CH1 = analogRead(ADC1);

long sum = 0;

for(int i=0;i<10;i++)
{
    sum += analogRead(ADC1);
    delay(2);
}

CH1 = sum / 10;

 flow = calculateFlow(CH1,
                     ch1_zero_count,
                     ch1_span_count,
                     ch1_zero_range,
                     ch1_span_range);

  
Serial.print("ADC=");
Serial.print(CH1);

Serial.print(" Flow=");
Serial.print(flow);

Serial.print(" Total=");
Serial.println(total);

// ===== ZERO FLOW DEAD BAND =====
float minimumFlowCutoff = 1.5;   // 1 LPM cutoff

if(flow < minimumFlowCutoff){
  flow = 0;
}
// ===== Integrate only when real flow exists =====
  updateTotal(flow,total,lastTime);

 double total_now = prefs.getDouble("total", total);
 if(isnanf(total_now)) total_now = 0 ;
 
 if(millis() - lastSaveTime > 30000 && total>total_now){

    prefs.putDouble("total", total);
    lastSaveTime = millis();
    Serial.println("Total Saved");

}
  // ======================================================================
  // MIDNIGHT RESET – placed BEFORE channel value computation
  // ======================================================================

  Serial.print(" Saved Reset Day :");Serial.println(resetDay);
  Serial.print("Saved  Reset Month :");Serial.println(resetMonth);
  if(month > resetMonth || (month == resetMonth && day > resetDay)){
    // Save yesterday total = current daily total before reset
    double oldDaily = total - lastNightTotal;
    prefs.putDouble("yesterday_total", oldDaily);
    yesterday_total = oldDaily;
    Serial.print("Yesterday total saved: "); Serial.println(oldDaily);

    // Update baseline to current overall total
    if (total > lastNightTotal) {
      lastNightTotal = total;
    }
    prefs.putDouble("lastNightTotal", lastNightTotal);
    Serial.print("lastNightTotal updated: "); Serial.println(lastNightTotal);

    // Today's total becomes zero
    dailyTotal = 0.0;

    resetDay = ntp_day;
    resetMonth = ntp_month;
    prefs.putInt("resetDay", ntp_day);
    prefs.putInt("resetMonth", ntp_month);
    Serial.println("Reset complete – new reset point: " + String(day) + "/" + String(month));

    // ═══════════════════════════════════════════════════════════════
    // Immediately update the HMI with the new daily and yesterday totals
    hmi.writeStr("Main.t50.txt", "0.00");                        // daily total
    hmi.writeStr("Main.t52.txt", String(yesterday_total, 2));    // yesterday total
    hmi.writeStr("Calibration.t50.txt", "0.00");
    hmi.writeStr("Calibration.t52.txt", String(yesterday_total, 2));
    // ═══════════════════════════════════════════════════════════════

  } else {
    // Normal day – today's total = overall minus last night baseline
    dailyTotal = total - lastNightTotal;
  }

  // Serial.println("");
  // Serial.print("CH1_name : ");Serial.println(ch1_name);
  // Serial.print("CH1_unit : ");Serial.println(ch1_unit);
  // Serial.print("CH1_Status : ");Serial.println(ch1_status);

  // Serial.print("CH2_name : ");Serial.println(ch2_name);
  // Serial.print("CH2_unit : ");Serial.println(ch2_unit);
  // Serial.print("CH2_Status : ");Serial.println(ch2_status);

  // Serial.print("CH3_name : ");Serial.println(ch3_name);
  // Serial.print("CH3_unit : ");Serial.println(ch3_unit);
  // Serial.print("CH3_Status : ");Serial.println(ch3_status);

  // Serial.print("CH4_name : ");Serial.println(ch4_name);
  // Serial.print("CH4_unit : ");Serial.println(ch4_unit);
  // Serial.print("CH4_Status : ");Serial.println(ch4_status);

  // Serial.print("CH5_name : ");Serial.println(ch5_name);
  // Serial.print("CH5_unit : ");Serial.println(ch5_unit);
  // Serial.print("CH5_Status : ");Serial.println(ch5_status);

  if(ch1_status==1){
    // ch1_final_value = map(SOx,ch1_zero_count,ch1_span_count,ch1_zero_range,ch1_span_range);
    ch1_final_value = flow;
    ch["ch1"]["name"] = ch1_name;
    ch["ch1"]["unit"] = ch1_unit;
    ch["ch1"]["value"] = ch1_final_value;

  }else{
    ch1_final_value = 0;
  }
  if(ch2_status==1){
    // ch2_final_value = map(NOx,ch2_zero_count,ch2_span_count,ch2_zero_range,ch2_span_range);
    ch2_final_value = dailyTotal;   // today's total (already updated after reset)
    ch["ch2"]["name"] = ch2_name;
    ch["ch2"]["unit"] = ch2_unit;
    ch["ch2"]["value"] = ch2_final_value;
  }else{
    ch2_final_value = 0;
  }

  if(ch3_status==1){
    // ch3_final_value = map(PM,ch3_zero_count,ch3_span_count,ch3_zero_range,ch3_span_range);
    ch3_final_value = total;
    ch["ch3"]["name"] = ch3_name;
    ch["ch3"]["unit"] = ch3_unit;
    ch["ch3"]["value"] = ch3_final_value;
  }else{
    ch3_final_value = 0;
  }

  if(ch4_status==1){
    // ch4_final_value = map(CH4,ch4_zero_count,ch4_span_count,ch4_zero_range,ch4_span_range);
    ch4_final_value = yesterday_total;
    ch["ch4"]["name"] = ch4_name;
    ch["ch4"]["unit"] = ch4_unit;
    ch["ch4"]["value"] = ch4_final_value;
  }else{
    ch4_final_value = 0;
  }

  if(ch5_status==1){
    ch5_final_value = map(CH5,ch5_zero_count,ch5_span_count,ch5_zero_range,ch5_span_range);
  ch["ch5"]["name"] = ch5_name;
  ch["ch5"]["unit"] = ch5_unit;
  ch["ch5"]["value"] = ch5_final_value;
  }else{
    ch5_final_value = 0;
  }


  hmi.writeStr("Calibration.t9.txt", String(CH1));
  hmi.writeStr("Calibration.t49.txt", String(ch1_final_value));

  hmi.writeStr("Calibration.t10.txt", String(lastNightTotal));
  hmi.writeStr("Calibration.t50.txt", String(ch2_final_value));
  
  hmi.writeStr("Calibration.t11.txt", String(total));
  hmi.writeStr("Calibration.t51.txt", String(ch3_final_value));

  hmi.writeStr("Calibration.t12.txt", String(CH4));
  hmi.writeStr("Calibration.t52.txt", String(ch4_final_value));

  hmi.writeStr("Calibration.t13.txt", String(CH5));
  hmi.writeStr("Calibration.t53.txt", String(ch5_final_value));

  // The original assignment dailyTotal = total-lastNightTotal; is now handled earlier; removed duplicate.

  getRelayStatus();

  // Serial.print(" Sent parameter 1 :");Serial.println(ch1_final_value);
  // Serial.print(" Sent parameter 2 :");Serial.println(ch2_final_value);
  // Serial.print(" Sent parameter 3 :");Serial.println(ch3_final_value);
  // Serial.print(" Sent parameter 4 :");Serial.println(ch4_final_value);
  // Serial.print(" Sent parameter 5 :");Serial.println(ch5_final_value);

  hmi.writeStr("Main.t49.txt", String(ch1_final_value));
  hmi.writeStr("Main.t50.txt", String(ch2_final_value));
  hmi.writeStr("Main.t51.txt", String(ch3_final_value));
  hmi.writeStr("Main.t52.txt", String(ch4_final_value));
  hmi.writeStr("Main.t53.txt", String(ch5_final_value));

  if(minute%5 == 0 && second>= 0 && second<= 04){
  String payload;
  serializeJson(doc, payload);
  Serial.println("\nSending Payload:");
  Serial.println(payload);
  uploadToServer(payload);
  }

  String payload;
    serializeJson(doc, payload);
    Serial.println("\nSending Payload:");
    Serial.println(payload);
    uploadToServer(payload);

  // The old midnight reset block has been moved up; removed from here.

  checkDeviceCommands();
  // lastNightTotal = prefs.getUShort("lastNightTotal", lastNightTotal);
  

  // Serial.print("Saved Flow Total  : ");Serial.println(lastNightTotal);

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

  MACHINE_ID = hmi.readStr("Deviceip.t8.txt");
  ssid     = hmi.readStr("Deviceip.t6.txt");
  password = hmi.readStr("Deviceip.t7.txt");
  relay_button = hmi.readNumber("Relay1.bt0.val");
  delay(500);

Serial.print(" WiFi Name : ");
Serial.println(ssid);

Serial.print(" WiFi Password : ");
Serial.println(password);
serial_id = hmi.readStr("About.t3.txt");
Serial.print("Serial_id: ");Serial.println(serial_id);

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

float calculateFlow(int CH, float zero_count, float span_count, float zero_range, float span_range){

  // 🔴 PROTECTION
  if (span_count == zero_count) {
    Serial.println("⚠️ Calibration Error: span_count == zero_count");
    return 0;
  }

  float slope = (span_range - zero_range) / 
                (span_count - zero_count);

  float offset = zero_range - (slope * zero_count);

  float flowRate = slope * CH + offset;

  // Clamp
  if (flowRate < 0) flowRate = 0;
  if (flowRate > span_range) flowRate = span_range;

  // 🔴 NaN protection
  if (isnan(flowRate) || isinf(flowRate)) {
    flowRate = 0;
  }

  if(abs(flow - lastFlow) > 10)
{
    Serial.println("Flow spike ignored");
    flow = lastFlow;
}

lastFlow = flow;

  return flowRate;
}

void updateTotal(float flowRate, double &total, unsigned long &lastTime) {

  if (isnan(flowRate) || isinf(flowRate)) {
    return;   // ❌ ignore bad value
  }

  unsigned long now = millis();

  if (lastTime == 0) {
    lastTime = now;
    return;
  }

  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if(flowRate <= 1.5)
        return;

  double delta = flowRate *0.06* (dt / 3600.0);

  if (!isnan(delta) && !isinf(delta)) {
    total += delta;
  }
}