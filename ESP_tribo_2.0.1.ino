
#include <ArduinoJson.h>
#include <Wire.h>
#include <Preferences.h>
Preferences prefs;

#include <WiFi.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

// ========================== OTA  Setup ==============================
// Current firmware version
String firmware_version = "2.2";
String ssid = "SOARMLICH E";
String password = "Soarmlich1193";
// GitHub RAW URLs
String version_url = "https://raw.githubusercontent.com/soarmlich/esp32-ota-update/main/version.json";

void firmwareUpdate(String binURL);

String serial_id = "";
#include <time.h>


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

// ==================================================================

String endChar = String(char(0xff))+String(char(0xff))+String(char(0xff));
String received = "";
String dtd = "";
int code = 0;
int pageno = 0;

int count = 0;

union{
  unsigned long getDataLong;
  byte getDataByteArray[4];
} getDataUnion;

// ===================================================================
long ch1_zero_count = 0;
long ch1_span_count = 0;
long ch1_zero_range = 0;
long ch1_span_range = 0;
float ch1_final_value = 0;

long ch2_zero_count = 0;
long ch2_span_count = 0;
long ch2_zero_range = 0;
long ch2_span_range = 0;
float ch2_final_value = 0;

long ch3_zero_count = 0;
long ch3_span_count = 0;
long ch3_zero_range = 0;
long ch3_span_range = 0;
long ch3_final_value = 0;

long ch4_zero_count = 0;
long ch4_span_count = 0;
long ch4_zero_range = 0;
long ch4_span_range = 0;
long ch4_final_value = 0;

long ch5_zero_count = 0;
long ch5_span_count = 0;
long ch5_zero_range = 0;
long ch5_span_range = 0;
long ch5_final_value = 0;

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

int high_alarm = 0;
int low_alarm =0;
int alarm_l = 0;

int relay_status = 0;

long lastread = 0;



/* ---------------- RS485 ---------------- */
#define RXD1 18
#define TXD1 5
HardwareSerial RS485Serial(2);

/* ---------------- Analog Inputs -------- */
#define ADC1 34
#define ADC2 35
#define ADC3 32
#define ADC4 33

#define DAC1 26
#define DAC2 25

#define RELAY_PIN 4 //  Relay pin mode 

// ---------------- Modbus Settings -------------------
// ---------- Modbus helper globals ----------
const uint16_t MODBUS_MAX_FRAME = 256;
const uint16_t MODBUS_INTERBYTE_MS = 4;   // time in ms used as inter-byte timeout (~3.5 char @ 9600)
const int DE_PIN = -1;                    // set to DE pin number if your RS485 transceiver needs driver enable, otherwise -1
// buffer used by the frame reader
uint8_t frameBuffer[MODBUS_MAX_FRAME];
uint16_t frameLen = 0;

//  modbus varialbles .............................................
uint8_t requestBuffer[256];
uint8_t responseBuffer[256];
uint16_t registerAddress;
uint16_t registerValue;
const uint16_t registerCount =10;
uint16_t slaveAddress = 01;
uint16_t modbusRegisters[registerCount];


/* ---------------- Sensor Data ---------- */
int ADC_A = 0, ADC_B = 0, ADC_C = 0;
int CH1=0,CH2=0,CH3=0,CH4=0,CH5=0;

// ===================================================================


// ========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(" System  Getting started !.......");
  
  Display.begin(115200,SERIAL_8N1,DIS_RX,DIS_TX);
  delay(5000);
  
  RS485Serial.begin(9600, SERIAL_8N1, RXD1, TXD1);
  delay(1000);
  loaddata();

  pinMode(ADC1,INPUT);
  pinMode(ADC2,INPUT);
  pinMode(ADC3,INPUT);
  pinMode(ADC4,INPUT);
  pinMode(DAC1,OUTPUT);
  pinMode(DAC1,OUTPUT);
  
  pinMode(RELAY_PIN,OUTPUT);

  digitalWrite(RELAY_PIN,LOW);

  String cmd2 = " Current Firmware : " +firmware_version;
  Serial.println(cmd2);
  //  =============================== WIFI Setup ============================
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int t = 0;
  while (t <= 25) {
    delay(1000);
    Serial.print(".");
    t++;
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi Connected");
      break;}

  }

  if (WiFi.status() == WL_CONNECTED) {
  checkForUpdates();
  }

  // =======================Auto Reboot Setup ================================
  bootTime = millis();
  getdate();
  Serial.print( " Auto reboot will happen at  : ");
  Serial.print( (hour+3));Serial.print(":");Serial.print(minute);Serial.print(":");Serial.print(second);
  Serial.println("");
  delay(1000);
  
}

EasyNex hmi(Display);

// ===============================================================================
void loop() {
  //  ==================Reboot =====================================================
  if (millis() - bootTime > AUTO_REBOOT_INTERVAL) {
    Serial.println("⏱ Scheduled reboot");
    Display.print("rest");
    Display.write(0xFF);
    Display.write(0xFF);
    Display.write(0xFF);
    delay(500);
    ESP.restart();
  }
 alarm_l=0;
  // =================================================================================
  if(RS485Serial.available()){
  readRegisters();
  }

  // ======================= Read from RS485_A =====================================
  delay(50);

  CH1 = analogRead(ADC1);
  CH2 = analogRead(ADC2);
  CH3 = analogRead(ADC3);
  CH4 = analogRead(ADC4);

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
  
  if(millis() -lastread >= 30000){

  high_alarm = hmi.readNumber("Relay_Sampling.n8.val");
  low_alarm = hmi.readNumber("Relay_Sampling.n0.val");

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
  }
  if(RS485Serial.available()){
  readRegisters();
  }

  modbus_id = hmi.readNumber("Deviceip.n0.val");
  Serial.print(" Modbus ID  : ");Serial.println(modbus_id);
  slaveAddress = modbus_id;
  
  reboot = hmi.readNumber("Deviceip.bt1.val");
  Serial.print(" Reboot Status  : ");Serial.println(reboot);
  if(reboot==1){
    Display.print("rest");
    Display.write(0xFF);
    Display.write(0xFF);
    Display.write(0xFF);
    delay(50);
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
  if(RS485Serial.available()){
  readRegisters();
  }

  if(ch1_status==1){

    ch1_final_value = ((float)(CH1 - ch1_zero_count) * 
                          (float)(ch1_span_range - ch1_zero_range) / 
                          (float)(ch1_span_count - ch1_zero_count) ) 
                        + ch1_zero_range;
    // ch1_final_value = map(CH1,ch1_zero_count,ch1_span_count,ch1_zero_range,ch1_span_range);
    ch1_final_value = constrain(ch1_final_value,ch1_zero_range,ch1_span_range);
    writeFloatRegister(0, (float)ch1_final_value);
    Serial.print("FINAL1: "); Serial.println(ch1_final_value, 6);
    if(ch1_final_value>high_alarm){
      alarm_l++;
    }
    
  }else{
    ch1_final_value = 0;
    dacWrite(DAC1,0);
    writeFloatRegister(0,0);
  }
  if(ch2_status==1){
    ch2_final_value = ((float)(CH2 - ch2_zero_count) * 
                          (float)(ch2_span_range - ch2_zero_range) / 
                          (float)(ch2_span_count - ch2_zero_count) ) 
                        + ch2_zero_range;
    // ch2_final_value = map(CH2,ch2_zero_count,ch2_span_count,ch2_zero_range,ch2_span_range);
    ch2_final_value = constrain(ch2_final_value,ch2_zero_range,ch2_span_range);
    writeFloatRegister(2, (float)ch2_final_value);
    Serial.print("FINAL2: "); Serial.println(ch2_final_value, 6);
    if(ch2_final_value>high_alarm){
      alarm_l++;
    }
    
  }else{
    ch2_final_value = 0;
    dacWrite(DAC2,0);
    writeFloatRegister(2,0);
  }

  if(ch3_status==1){
    // ch3_final_value = ((float)(CH3 - ch3_zero_count) * 
    //                       (float)(ch3_span_range - ch3_zero_range) / 
    //                       (float)(ch3_span_count - ch3_zero_count) ) 
    //                     + ch3_zero_range;
    ch3_final_value = map(CH3,ch3_zero_count,ch3_span_count,ch3_zero_range,ch3_span_range);
    ch3_final_value = constrain(ch3_final_value,ch3_zero_range,ch3_span_range);
    writeFloatRegister(4, (float)ch3_final_value);
    Serial.print("FINAL3: "); Serial.println(ch3_final_value, 6);
    if(ch3_final_value>high_alarm){
      alarm_l++;
    }
    
  }else{
    ch3_final_value = 0;
    writeFloatRegister(4,0);
  }

  if(ch4_status==1){
    // ch4_final_value = ((float)(CH4 - ch4_zero_count) * 
    //                       (float)(ch4_span_range - ch4_zero_range) / 
    //                       (float)(ch4_span_count - ch4_zero_count) ) 
    //                     + ch4_zero_range;
    ch4_final_value = map(CH4,ch4_zero_count,ch4_span_count,ch4_zero_range,ch4_span_range);
    ch4_final_value = constrain(ch4_final_value,ch4_zero_range,ch4_span_range);
    writeFloatRegister(6, (float)ch4_final_value);
    Serial.print("FINAL4: "); Serial.println(ch4_final_value, 6);
    if(ch4_final_value>high_alarm){
      alarm_l++;
    }
       
  }else{
    ch4_final_value = 0;
    writeFloatRegister(6,0);
  }

  if(ch5_status==1){
    // ch5_final_value = ((float)(CH5 - ch5_zero_count) * 
    //                       (float)(ch5_span_range - ch5_zero_range) / 
    //                       (float)(ch5_span_count - ch5_zero_count) ) 
    //                     + ch5_zero_range;
    ch5_final_value = map(CH5,ch5_zero_count,ch5_span_count,ch5_zero_range,ch5_span_range);
    ch5_final_value = constrain(ch5_final_value,ch5_zero_range,ch5_span_range);
    writeFloatRegister(8, (float)ch5_final_value);
    Serial.print("FINAL5: "); Serial.println(ch5_final_value, 6);
    if(ch5_final_value>high_alarm){
      alarm_l++;
    }
        
  }else{
    ch5_final_value = 0;
    writeFloatRegister(8,0);
  }

  for (int i = 6; i < registerCount; i++)
        {
          modbusRegisters[i] = 0;
        }
      if(RS485Serial.available()){
  readRegisters();
  }

  if(alarm_l > 0){
    digitalWrite(RELAY_PIN,HIGH);
    Display.print("vis g1,1");
    Display.write(0xFF);
    Display.write(0xFF);
    Display.write(0xFF);
    }
  else{
    digitalWrite(RELAY_PIN,LOW);
    Display.print("vis g1,0");
    Display.write(0xFF);
    Display.write(0xFF);
    Display.write(0xFF);
  }


  hmi.writeStr("Calibration.t9.txt", String(CH1));
  hmi.writeStr("Calibration.t49.txt",String(ch1_final_value));

  hmi.writeStr("Calibration.t10.txt", String(CH2));
  hmi.writeStr("Calibration.t50.txt",String(ch2_final_value));
  
  hmi.writeStr("Calibration.t11.txt", String(CH3));
  hmi.writeStr("Calibration.t51.txt", String(ch3_final_value));

  hmi.writeStr("Calibration.t12.txt", String(CH4));
  hmi.writeStr("Calibration.t52.txt", String(ch4_final_value));

  hmi.writeStr("Calibration.t13.txt", String(CH5));
  hmi.writeStr("Calibration.t53.txt",String(ch5_final_value));
  readRegisters();

  Serial.print(" Sent parameter 1 :");Serial.println(ch1_final_value);
  Serial.print(" Sent parameter 2 :");Serial.println(ch2_final_value);
  Serial.print(" Sent parameter 3 :");Serial.println(ch3_final_value);
  Serial.print(" Sent parameter 4 :");Serial.println(ch4_final_value);
  Serial.print(" Sent parameter 5 :");Serial.println(ch5_final_value);

  hmi.writeStr("Main.t49.txt", String(ch1_final_value,1));
  hmi.writeStr("Main.t50.txt", String(ch2_final_value,1));
  hmi.writeStr("Main.t51.txt", String(ch3_final_value));
  hmi.writeStr("Main.t52.txt", String(ch4_final_value));
  hmi.writeStr("Main.t53.txt", String(ch5_final_value));

  delay(200);
}
uint8_t getPageNo(){
  Display.print("sendme");
  Display.write(0xff);
  Display.write(0xff);
  Display.write(0xff);
  delay(100);
  String dfd="";
  while(Display.available()>0){
    char character = char(Display.read());
    dfd = dfd += character;
    if(dfd.endsWith(endChar)){
      code = dfd[0];
      pageno = dfd[1];
      dfd="";
    }   
  }
 Serial.print(" Received Data : ");
 Serial.println(pageno);
 delay(50);
  return pageno;
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

  slaveAddress = modbus_id;

  high_alarm = hmi.readNumber("Relay_Sampling.n8.val");
  low_alarm = hmi.readNumber("Relay_Sampling.n0.val");

  serial_id = hmi.readStr("About.t3.txt");
  Serial.print("Serial_id: ");Serial.println(serial_id);

  ssid = hmi.readStr("Deviceip.t6.txt");
  password = hmi.readStr("Deviceip.t7.txt");

  Serial.print(" WIFI Name : ");Serial.println(ssid);
  Serial.print(" WIFI Password : ");Serial.println(password);

  delay(500);
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

void processWriteSingleRegister()
{
  // requestBuffer holds the request
  uint16_t startAddress = (requestBuffer[2] << 8) | requestBuffer[3];
  uint16_t value = (requestBuffer[4] << 8) | requestBuffer[5];

  if (startAddress < registerCount) {
    modbusRegisters[startAddress] = value;
  } else {
    Serial.println("WriteSingle: invalid address");
    // could send exception response if desired
    return;
  }

  // Echo the request back as per Modbus spec (addr func addrHi addrLo valHi valLo CRClo CRChi)
  uint8_t resp[8];
  memcpy(resp, requestBuffer, 6); // copy first 6 bytes
  uint16_t crc = calculateCRC(resp, 6);
  resp[6] = crc & 0xFF;
  resp[7] = (crc >> 8) & 0xFF;

  if (DE_PIN >= 0) digitalWrite(DE_PIN, HIGH);
  RS485Serial.write(resp, 8);
  RS485Serial.flush();
  delayMicroseconds(200);
  if (DE_PIN >= 0) digitalWrite(DE_PIN, LOW);
}

void writeFloatRegister(uint16_t registerAddress, float value)
{
  if (registerAddress < registerCount - 1)
  {
    uint32_t floatBits;
    memcpy(&floatBits, &value, sizeof(float));
    modbusRegisters[registerAddress] = (floatBits >> 16) & 0xFFFF;
    modbusRegisters[registerAddress + 1] = floatBits & 0xFFFF;
  }
}

void writeRegister(uint16_t registerAddress, uint16_t value)
{
  if (registerAddress < registerCount)
  {
    modbusRegisters[registerAddress] = value;
  }
}

uint16_t getRegister(uint16_t registerAddress)
{
  if (registerAddress < registerCount)
  {
    return modbusRegisters[registerAddress];
  }
  return 0;
}

uint16_t getFloatRegister(uint16_t registerAddress)
{
  if (registerAddress < registerCount - 1)
  {
    uint32_t floatBits = ((uint32_t)modbusRegisters[registerAddress] << 16) | modbusRegisters[registerAddress + 1];
    float value;
    memcpy(&value, &floatBits, sizeof(float));
    return value;
  }
  return 0.0;
}

void processReadHoldingRegisters()
{
  // requestBuffer must contain the full request (we copied it earlier)
  uint16_t startAddress = (requestBuffer[2] << 8) | requestBuffer[3];
  uint16_t _registerCount = (requestBuffer[4] << 8) | requestBuffer[5];

  // basic bounds check (avoid overflow)
  if ((startAddress + _registerCount) > registerCount || _registerCount == 0 || _registerCount > 125) {
    Serial.println("ReadHolding: invalid register range");
    return;
  }

  // build response in a local byte buffer
  // response: addr(1) func(1) byteCount(1) data(2*count) crcLo crcHi
  uint16_t byteCount = _registerCount * 2;
  uint16_t respLenNoCrc = 3 + byteCount; // bytes before CRC
  uint8_t resp[256];
  resp[0] = (uint8_t)slaveAddress;
  resp[1] = 0x03;
  resp[2] = (uint8_t)byteCount;

  for (int i = 0; i < _registerCount; i++) {
    uint16_t regValue = modbusRegisters[startAddress + i];
    resp[3 + i*2] = (regValue >> 8) & 0xFF; // high byte
    resp[4 + i*2] = regValue & 0xFF;        // low byte
  }

  uint16_t crc = calculateCRC(resp, respLenNoCrc);
  resp[respLenNoCrc]     = crc & 0xFF;       // CRC low
  resp[respLenNoCrc + 1] = (crc >> 8) & 0xFF; // CRC high

  // send response (use helper to handle DE if needed)
  // total bytes to send:
  uint16_t totalSend = respLenNoCrc + 2;
  // send now:
  if (DE_PIN >= 0) digitalWrite(DE_PIN, HIGH);
  RS485Serial.write(resp, totalSend);
  RS485Serial.flush();
  // tiny delay to ensure last bits out (depends on baud/rx)
  delayMicroseconds(200);
  if (DE_PIN >= 0) digitalWrite(DE_PIN, LOW);
}

// Receive a frame using inter-byte timeout. Returns length (0 = no complete frame)
uint16_t recvFrame(uint8_t *buf, uint16_t maxLen) {
  uint32_t startTime = millis();
  uint16_t idx = 0;

  // wait for first byte (with a small timeout to avoid blocking)
  uint32_t waitStart = millis();
  while (!RS485Serial.available()) {
    if (millis() - waitStart > 200) return 0; // no incoming data
    yield();
  }

  // read bytes until silence longer than inter-byte timeout
  startTime = millis();
  while (true) {
    while (RS485Serial.available() && idx < maxLen) {
      int c = RS485Serial.read();
      if (c < 0) break;
      buf[idx++] = (uint8_t)c;
      startTime = millis(); // reset inter-byte timer
    }
    // if we've read something and we see silence for inter-byte ms, assume frame end
    if (idx > 0 && (millis() - startTime) >= MODBUS_INTERBYTE_MS) {
      break;
    }
    // small yield to avoid hogging CPU
    yield();
  }
  return idx;
}

// New robust readRegisters: reads a frame and dispatches according to function code
bool readRegisters() {
  uint16_t len = recvFrame(frameBuffer, MODBUS_MAX_FRAME);
  if (len == 0) return false;

  // Basic sanity: minimum modbus RTU request length is 4 (id + func + crc(2)) + payload depends on func.
  if (len < 4) {
    // too short -> ignore
    return false;
  }

  // Verify CRC: pass length-2 as payload length to calc
  if (!checkCRC(frameBuffer, len - 2)) {
    // CRC failed -> ignore frame
    Serial.print("CRC error on request, RX len=");
    Serial.println(len);
    // optionally dump bytes for debugging:
    Serial.print("RX (bad CRC): ");
    for (int i = 0; i < len; i++) {
      Serial.print("0x");
      if (frameBuffer[i] < 0x10) Serial.print("0");
      Serial.print(frameBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    return false;
  }

  // Address check
  uint8_t addr = frameBuffer[0];
  if (addr != (uint8_t)slaveAddress && addr != 0x00) {
    // not for us and not broadcast -> ignore
    return false;
  }

  uint8_t func = frameBuffer[1];

  // dispatch supported functions (read holding 0x03, write single 0x06)
  if (func == 0x03) {
    // frame should be: addr(1) func(1) startHi(1) startLo(1) qtyHi(1) qtyLo(1) crcLo(1) crcHi(1)
    if (len < 8) return false; // malformed
    // copy request into requestBuffer for existing processing code compatibility
    memcpy(requestBuffer, frameBuffer, len);
    processReadHoldingRegisters();
    return true;
  } else if (func == 0x06) {
    if (len < 8) return false;
    memcpy(requestBuffer, frameBuffer, len);
    processWriteSingleRegister();
    return true;
  } else {
    Serial.print("Unsupported function: 0x");
    Serial.println(func, HEX);
    // optionally respond with Modbus Exception (function + 0x80) — not implemented here
    return false;
  }
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

#include <WiFiClientSecure.h>   // ADD THIS AT TOP

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


