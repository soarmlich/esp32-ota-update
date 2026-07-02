
#include <Wire.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <NextionLite.h>
#include <RTClib.h>
#include <time.h>
#include "EasyNextionLibrary.h"
#include <Preferences.h>
Preferences prefs;
/*      Nextion Dispaly                     */
#define DIS_RX 16
#define DIS_TX 17
HardwareSerial Display(1);
EasyNex hmi(Display);
RTC_DS3231 rtc;

String ssid = "SOARMLICH E";
String password = "Soarmlich1193";


// ==========================================================================

String endChar = String(char(0xff))+String(char(0xff))+String(char(0xff));
String received = "";
// ===================================================================
long zero_count = 0;
long span_count = 0;
long zero_range = 0;
long span_range = 0;
long final_value = 0;

int low_alarm = 0;
int high_alarm = 0;

int alarm_save = 0;
int cal_save = 0;

int battery_percent = 100;
int dueDays = 120;

bool standby = false;
bool pressed = false;

unsigned long lastCalibrationEpoch;
unsigned long currentEpoch;
unsigned long firstStartEpoch;


long firstStartDate;
long lastCalibrationDate;
long calibrationIntervalDays=365;

int isCharging;
int isCharged ;




int year = 0;
int month = 0;
int day = 0;
int hour = 0;
int minute = 0;
int second = 0;

int relay_status = 0;

String message = "";

enum DeviceState { 
  NOMODE,
  BOOT, 
  ACTIVE, 
  STANDBY, 
  CALIBRATION_REQUIRED, 
  CALIBRATION_MODE, 
  ERROR_STATE 
  };
  DeviceState currentState;

volatile bool touchDetected = false;
unsigned long pressStartTime = 0;
unsigned long daysPassed = 0;


bool buttonHeld = false;

void IRAM_ATTR interrupt()
{
    touchDetected = true;
}

/* ---------------- Analog Inputs -------- */
#define sensor 34
#define red_led 32
#define green_led 33
#define buzzer 5
#define vibrator 18           
#define power_button 15
#define display_power 19
#define battery_lel 35
#define charging 27
#define charged 12
#define sensorEnable 14


#define SDA 22
#define SCL 21


/* ---------------- Sensor Data ---------- */
int ADC = 0;
int CH1=0;

// ========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(" System  Getting started !.......");
  
  Display.begin(9600,SERIAL_8N1,DIS_RX,DIS_TX);
  
  delay(100);
  
  Serial.print("Reset Reason: ");
  Serial.println(esp_reset_reason());
  Wire.begin(SDA, SCL); // Master
  
  pinMode(sensor,INPUT);
  pinMode(sensorEnable,OUTPUT);
  pinMode(red_led,OUTPUT);
  
  
  pinMode(green_led,OUTPUT);
  pinMode(buzzer,OUTPUT);
  
  pinMode(vibrator,OUTPUT);
  pinMode(display_power,OUTPUT);
  
  pinMode(power_button,INPUT);
  pinMode(battery_lel,INPUT);
  pinMode(charging,INPUT);
  pinMode(charged,INPUT);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);
  digitalWrite(display_power,LOW);
  digitalWrite(sensorEnable,LOW);

  
  

  WiFi.begin(ssid,password);
  Serial.print(" Connecting Wifi ");
  int count = 0;
  while(count <20 && WiFi.status()!=WL_CONNECTED){
    Serial.print(".");
    count++;
    delay(500);

  }
  Serial.println("");
  if(WiFi.status() == WL_CONNECTED){
    Serial.println(" WIFI Connected ");
  }
  if(WiFi.status()!=WL_CONNECTED){
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  // rtc.begin();
  // configTime(19800, 0, "pool.ntp.org");
  // struct tm timeinfo;

  // if(getLocalTime(&timeinfo))
  // {
  //     Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");

  //     // rtc.adjust(DateTime(
  //     // timeinfo.tm_year + 1900,
  //     // timeinfo.tm_mon + 1,
  //     // timeinfo.tm_mday,
  //     // timeinfo.tm_hour,
  //     // timeinfo.tm_min,
  //     // timeinfo.tm_sec
  //     // ));
      
  // }
  // else { Serial.println("Failed To Get Time"); }

  prefs.begin("memory",false);
  currentState = BOOT;
    // (DeviceState)prefs.getInt("currentState", BOOT);

    Serial.print("Restored State: ");
    Serial.println(currentState);

    // if(!prefs.isKey("firstStart"))
    // {
    //     prefs.putULong("firstStart", rtc.now().unixtime());
          // prefs.putInt("calDays",365);
          // currentEpoch = rtc.now().unixtime();
          // prefs.putULong("lastCal", CurrentEpoch);
          //  
    // }
    // currentState = BOOT;
    //  prefs.putInt("currentState", currentState);
      //  currentEpoch = rtc.now().unixtime();
    
//     lastCalibrationEpoch = prefs.getULong("lastCal");
//     calibrationIntervalDays = prefs.getInt("calDays");
//     firstStartEpoch = prefs.getULong("firstStart");

}



// ===============================================================================
void loop() {
  //  ==================Reboot =====================================================
  
  handlePowerButton();
  switch(currentState)
    {
      case BOOT: {
        Serial.println(" BOOT Mode ");
        boot();
        break;
      }
      case ACTIVE:{
        Serial.println(" Active Mode ");
        readSensor();
        break;
      }
      case STANDBY:{
        Serial.println(" STANDBY Mode ");
        break;
      }
      case CALIBRATION_REQUIRED:{
        Serial.println(" CALIBRATION REQUIRED ");
        calibration_required();
        break;
      }
      case ERROR_STATE :{
        Serial.println("  ERROR STATE ");
        errorCheck();
        break;
      }
    }
  delay(500);
}


void readSensor(){

  digitalWrite(display_power,HIGH);
  digitalWrite(sensorEnable,HIGH);

  batteryStatus();
  delay(250);
  printDue();
  delay(200);

  CH1 = analogRead(sensor);
  Serial.print(" Sensor ADC Value : ");Serial.println(CH1);
  zero_count = hmi.readNumber("Calibration.n1.val");
  span_count = hmi.readNumber("Calibration.n2.val");
  zero_range = hmi.readNumber("Calibration.n3.val");
  span_range = hmi.readNumber("Calibration.n4.val");
  final_value = map(CH1,zero_count,span_count,zero_range,span_range);
  final_value = constrain(final_value,zero_range,span_range);

  
  hmi.writeNum("Calibration.n0.val",CH1);
  hmi.writeNum("Calibration.n5.val",final_value);

  hmi.writeStr("Main.t1.txt", String(final_value));

  hmi.writeStr("Main.t6.txt", String(low_alarm));
  hmi.writeStr("Main.t7.txt", String(high_alarm));
  delay(250);
  alarm_save = hmi.readNumber("alarm_setup.bt3.val");
  cal_save = hmi.readNumber("Calibration.bt1.val");
  if(alarm_save==1){
    Serial.println(" Alarm Setup Upadted ");
    low_alarm = hmi.readNumber("alarm_setup.n0.val");
    high_alarm = hmi.readNumber("alarm_setup.n1.val");
    prefs.putInt("low_alarm", low_alarm);
    prefs.putInt("high_alarm",high_alarm);
    hmi.writeNum("alarm_setup.bt3.val",0);
  }
  if(cal_save == 1){
    Serial.println(" Claibration Setup Upadted ");
    zero_count=hmi.readNumber("Calibration.n1.val");
    span_count=hmi.readNumber("Calibration.n2.val");
    zero_range=hmi.readNumber("Calibration.n3.val");
    span_range=hmi.readNumber("Calibration.n4.val");
    
    prefs.putLong("zero_count", zero_count);
    prefs.putLong("span_count", span_count);
    prefs.putLong("zero_range", zero_range);
    prefs.putLong("span_range", span_range);

    hmi.writeNum("Calibration.bt1.val", 0);
  }
  if(final_value>=low_alarm && final_value<high_alarm){
    alarm_low();
  }        
  else if(final_value>=high_alarm){
    alarm_high();
  }
  else{
    digitalWrite(green_led,HIGH);
    delay(500);
    digitalWrite(green_led,LOW);
  }
  // errorCheck();
  


  delay(500);
}

void alarm_low(){

  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,HIGH);
  digitalWrite(vibrator,HIGH);
  delay(500);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);

}
void alarm_high(){
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,HIGH);
  digitalWrite(vibrator,HIGH);
  delay(90);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);
  delay(90);
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,HIGH);
  digitalWrite(vibrator,HIGH);
  delay(90);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);
  delay(90);
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,HIGH);
  digitalWrite(vibrator,HIGH);
  delay(90);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);
  delay(90);
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,HIGH);
  digitalWrite(vibrator,HIGH);
  delay(90);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);
}

void handlePowerButton(){
    bool pressed =  digitalRead(power_button);

    // button pressed first time
    if (pressed && !buttonHeld)
    {
        buttonHeld = true;
        pressStartTime = millis();

        Serial.println("Button Pressed");
    }

    // long press detected
    while (buttonHeld)
    {
        bool pressed =  digitalRead(power_button);
        // button released
    if (!pressed)
    {
        buttonHeld = false;
    }
        if (standby && buttonHeld &&
        (millis() - pressStartTime > 2000))
        {
            currentState = BOOT;
            prefs.putInt("currentState", currentState);
            digitalWrite(buzzer,HIGH);
            delay(500);
            ESP.restart();
            
        }
        else if(!standby && buttonHeld &&
        (millis() - pressStartTime > 2000))
        {
            currentState = STANDBY;

            prefs.putInt("currentState", currentState);
            standby = true;
            digitalWrite(red_led,HIGH);
            digitalWrite(green_led,LOW);
            digitalWrite(buzzer,HIGH);
            digitalWrite(vibrator,HIGH);
            delay(1000);
            digitalWrite(display_power,LOW);
            digitalWrite(sensorEnable,LOW);
            digitalWrite(red_led,LOW);
            digitalWrite(green_led,LOW);
            digitalWrite(buzzer,LOW);
            digitalWrite(vibrator,LOW);
            if(WiFi.status()==WL_CONNECTED){
              WiFi.disconnect(true);
              WiFi.mode(WIFI_OFF);
            }
            delay(3000);
            
            
            break;
        }
    }
}
void boot(){
  Serial.println(" Booting the SYSTEM ! .......");
  digitalWrite(display_power,HIGH);
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,HIGH);
  digitalWrite(vibrator,HIGH);
  delay(1000);
  zero_count=prefs.getLong("zero_count");
  span_count=prefs.getLong("span_count");
  zero_range=prefs.getLong("zero_range");
  span_range=prefs.getLong("span_range");
  low_alarm=prefs.getInt("low_alarm");
  high_alarm=prefs.getInt("high_alarm");
  hmi.writeNum("Calibration.n0.val",CH1);
  hmi.writeNum("Calibration.n5.val",final_value);

  hmi.writeStr("Main.t1.txt", String(final_value));

  hmi.writeStr("Main.t6.txt", String(low_alarm));
  hmi.writeStr("Main.t7.txt", String(high_alarm));

  hmi.writeNum("alarm_setup.n0.val",low_alarm);
  hmi.writeNum("alarm_setup.n1.val",high_alarm);
  hmi.writeNum("Calibration.n1.val",zero_count);
  hmi.writeNum("Calibration.n2.val",span_count);
  hmi.writeNum("Calibration.n3.val",zero_range);
  hmi.writeNum("Calibration.n4.val",span_range);
  delay(800);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,HIGH);
  delay(1000);
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(vibrator,LOW);

//   DateTime now = rtc.now();
//   Serial.print(" RTC Date & Time : ");
//   Serial.printf(
//     "%02d/%02d/%04d %02d:%02d:%02d\n",
//     now.day(),
//     now.month(),
//     now.year(),
//     now.hour(),
//     now.minute(),
//     now.second()
// );
//   daysPassed =(currentEpoch - lastCalibrationEpoch) / 86400;
//   if(daysPassed >= calibrationIntervalDays)
//   {
//       currentState = CALIBRATION_REQUIRED;        
//       prefs.putInt("currentState", currentState);
//   }
//   errorCheck();

  currentState = ACTIVE;
  prefs.putInt("currentState",currentState); 
}

void batteryStatus(){
  isCharging = digitalRead(charging);
  isCharged =  digitalRead(charged);
  long battery_sum = 0;
  for(int i=0;i<10;i++){
  long battery_percent_raw_intial = analogRead(battery_lel);
  battery_sum += battery_percent_raw_intial;
  delay(50);
  }
  long battery_percent_raw =  battery_sum/10;
  Serial.print(" ADC value of Battery :");Serial.println(battery_percent_raw);
  battery_percent = map (battery_percent_raw,1850,2390,0,100);
  battery_percent = constrain( battery_percent,0,100);
  

 if(isCharging == 1 || battery_percent_raw >= 2400){
    hmi.writeStr("Main.t12.txt","   Charging.......  ");
    hmi.writeNum("Main.t12.bco", 19067);
    hmi.writeStr("Main.t11.txt","C");
}
 else if(isCharged == 1 || battery_percent == 100){
    hmi.writeStr("Main.t12.txt","   Charged !  ");
    hmi.writeNum("Main.t12.bco", 2016);
    hmi.writeStr("Main.t11.txt","100");
}

else if(battery_percent <= 20){
    hmi.writeStr("Main.t12.txt","   Low Battery !  ");
    hmi.writeNum("Main.t12.bco", 63488);
    hmi.writeStr("Main.t11.txt",String(battery_percent));

}
else{
    hmi.writeStr("Main.t12.txt","   ");
    hmi.writeNum("Main.t12.bco", 65535);
    hmi.writeStr("Main.t11.txt",String(battery_percent));
}
  
}
void calibration_required(){
  batteryStatus();
  CH1 = analogRead(sensor);
  zero_count = hmi.readNumber("Calibration.n1.val");
  span_count = hmi.readNumber("Calibration.n2.val");
  zero_range = hmi.readNumber("Calibration.n3.val");
  span_range = hmi.readNumber("Calibration.n4.val");
  final_value = map(CH1,zero_count,span_count,zero_range,span_range);
  final_value = constrain(final_value,zero_range,span_range);
  hmi.writeNum("Calibration.n0.val",CH1);
  hmi.writeNum("Calibration.n5.val",final_value);

  hmi.writeStr("Main.t1.txt", "---");

  hmi.writeStr("Main.t12.txt","   CALIBRATION  DUE !  ");
  hmi.writeNum("Main.t12.bco", 63488);

  hmi.writeStr("Main.t6.txt", String(low_alarm));
  hmi.writeStr("Main.t7.txt", String(high_alarm));

  
  hmi.writeNum("alarm_setup.n0.val",low_alarm);
  hmi.writeNum("alarm_setup.n1.val",high_alarm);
  hmi.writeNum("Calibration.n1.val",zero_count);
  hmi.writeNum("Calibration.n2.val",span_count);
  hmi.writeNum("Calibration.n3.val",zero_range);
  hmi.writeNum("Calibration.n4.val",span_range);

  int calibration_save = hmi.readNumber("Calibration.bt1.val");

  if(calibration_save == 1){
    Serial.println(" Claibration Setup Upadted ");
    zero_count=hmi.readNumber("Calibration.n1.val");
    span_count=hmi.readNumber("Calibration.n2.val");
    zero_range=hmi.readNumber("Calibration.n3.val");
    span_range=hmi.readNumber("Calibration.n4.val");
    
    prefs.putLong("zero_count", zero_count);
    prefs.putLong("span_count", span_count);
    prefs.putLong("zero_range", zero_range);
    prefs.putLong("span_range", span_range);

    // currentEpoch = rtc.now().unixtime();  
    // prefs.putULong("lastCal", currentEpoch);
    // prefs.putInt("calDays", 90);

    currentState = ACTIVE;
    prefs.putInt("currentState", currentState);

    hmi.writeNum("Calibration.bt1.val", 0);
  }
  else{
    Serial.println(" Calibration Failed ! ......");
    Serial.println("Please Connect the wifi to procced with calibration ");
  }
  errorCheck();

}

void printDue(){
  int dayLaspe = (currentEpoch-firstStartEpoch) / 86400;
  if(dayLaspe >= 365){
    dueDays = calibrationIntervalDays - daysPassed;
    Serial.print(" Calibration Due in : "); Serial.println(dueDays);
    String cmd = " Calibration Due in "+ String(dueDays)+" Days";
    hmi.writeStr("Main.t12.txt",cmd);
  }
}

void errorCheck(){
  digitalWrite(sensorEnable,HIGH);
  delay(100);
  int Analog_check = analogRead(sensor);
  if (!rtc.begin()) { 
    Serial.println("RTC NOT FOUND"); 
    currentState = ERROR_STATE;
    prefs.putInt("currentState",currentState);
    }
    else if(Analog_check<1400){
      Serial.println(" Analog Read  Error ! ");
      currentState = ERROR_STATE;
      prefs.putInt("currentState",currentState);
    }
    else if(Display.available() == 0){
      Serial.println(" Display Read  Error ! ");
      currentState = ERROR_STATE;
      prefs.putInt("currentState",currentState);
    }
    else{
      Serial.println(" NO ERROR FOUND ! ");
      Serial.println(" Back to Boot mode .........");
      currentState=BOOT;
      prefs.putInt("currentState", currentState);
    }

}

