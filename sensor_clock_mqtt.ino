#include "LedControl.h"
#include <Ticker.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <Timezone.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

/*
 NodeMCU -> MAX7219
 5V   -> 5V
 D8   -> CS
 D5   -> DIN
 D7   -> CLK
*/

const char* WIFI_SSID = "";
const char* WIFI_PASSWD = "";

const byte MAX_DIN_PIN = 2;
const byte MAX_CS_PIN = 16;
const byte MAX_CLOCK_PIN = 4;

const byte I2C_SDA = 12;
const byte I2C_SCL = 5;

const byte MOTION_SENSOR_PIN = 13;
boolean lastMotionSensorState = false;

IPAddress mqttServer(192, 168, 31, 100); // mosquitto address
const char* MQTT_USER = "smhome";
const char* MQTT_PASS = "smhome";
const char* MQTT_CLIENT = "ESP8266";
const char* MQTT_CO2_TOPIC = "sensor_clock/co2_level";
const char* MQTT_LIGHT_LEVEL_TOPIC = "sensor_clock/light_level";
const char* MQTT_MOTION_SENSOR_TOPIC = "sensor_clock/motion_sensor";

const int BH1750_I2C_ADDRESS = 0x23;
const int BH1750_CONTINUOUS_LOW_RES_MODE = 0x13;

unsigned int lastLightLevel = 0;

TimeChangeRule EEST = {"EEST", Last, Sun, Mar, 3, 180};  //Daylight time = UTC + 3 hours
TimeChangeRule EET = {"EET", Last, Sun, Oct, 4, 120}; //Standard time = UTC + 2 hours
Timezone CE(EEST, EET);
TimeChangeRule *tcr; 

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte ntpPacketBuffer[NTP_PACKET_SIZE];

const char* NTP_SERVER = "pool.ntp.org";
const int NTP_CLIENT_PORT = 2390;
const int NTP_SERVER_PORT = 123;
const int NTP_SERVER_RETRY_DELAY = 16000;

const double NTP_SERVER_UPDATE_INTERVAL = 86400;
const double CO2_UPDATE_INTERVAL = 30;
const double TIME_UPDATE_INTERVAL = 60;
const float TIME_TICK_UPDATE_INTERVAL = 0.5;
const byte MAX_WIFI_CONNECT_DELAY = 50;

WiFiUDP udp;
IPAddress ntpServerIP;

WiFiClient httpClient;

PubSubClient mqttClient(httpClient, mqttServer);

const unsigned char LED_DIGITS[][4] = {
  0xFE, 0x82, 0xFE, 0x0, // 0
  0x0,  0x40, 0xFE, 0x0, // 1
  0x9E, 0x92, 0xF2, 0x0, // 2
  0x92, 0x92, 0xFE, 0x0, // 3
  0xF0, 0x10, 0xFE, 0x0, // 4
  0xF2, 0x92, 0x9E, 0x0, // 5
  0xFE, 0x92, 0x9E, 0x0, // 6
  0x80, 0x80, 0xFE, 0x0, // 7
  0xFE, 0x92, 0xFE, 0x0, // 8
  0xF2, 0x92, 0xFE, 0x0  // 9  
};

const byte MIN_CLOCK_BRIGHTNESS = 1;
const byte MAX_CLOCK_BRIGHTNESS = 5;

const byte NIGHT_MODE_CLOCK_LUX_LEVEL = 7;

LedControl ledDisp = LedControl(MAX_DIN_PIN, MAX_CLOCK_PIN, MAX_CS_PIN, 2);
Ticker blinker;

byte tickShown = 0;

void changeClockTick() {
  if (tickShown == 1) {
    ledDisp.setColumn(0, 7, 0x00);
    tickShown = 0;
  }
  else {
    ledDisp.setColumn(0, 7, 0x24);
    tickShown = 1;
  }
}

void displayTime(int pos, int n) {
  int i;
  switch (pos) {
    case 0:
      if (n == 1) {
        ledDisp.setColumn(0, 0, 0x40);
        ledDisp.setColumn(0, 1, 0xFE);
      }
      else {
        ledDisp.setColumn(0, 0, 0x00);
        ledDisp.setColumn(0, 1, 0x00);
      }
      break;
    case 1:
      for (i = 0; i < 4; i++) {
        ledDisp.setColumn(0, i + 3, LED_DIGITS[n][i]);
      }
      break;
    case 2:    
      for (i = 0; i < 4; i++) {
        ledDisp.setColumn(1, i + 1, LED_DIGITS[n][i]);
      }
      break;
    case 3:
      for (i = 0; i < 4; i++) {
        ledDisp.setColumn(1, i + 5, LED_DIGITS[n][i]);
      }
      break;
  }
}

time_t getNTPtime() {
  time_t epoch = 0UL;
  while((epoch = getFromNTP()) == 0) {
    delay(NTP_SERVER_RETRY_DELAY);
  }
  epoch -= 2208988800UL;
  return CE.toLocal(epoch, &tcr);
}

unsigned long getFromNTP() {
  udp.begin(NTP_CLIENT_PORT);
  if(!WiFi.hostByName(NTP_SERVER, ntpServerIP)) {
     return 0UL;
  }
  memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);
  ntpPacketBuffer[0] = 0b11100011;
  ntpPacketBuffer[1] = 0;
  ntpPacketBuffer[2] = 6;
  ntpPacketBuffer[3] = 0xEC;
  ntpPacketBuffer[12]  = 49;
  ntpPacketBuffer[13]  = 0x4E;
  ntpPacketBuffer[14]  = 49;
  ntpPacketBuffer[15]  = 52;

  udp.beginPacket(ntpServerIP, NTP_SERVER_PORT);
  udp.write(ntpPacketBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
  
  delay(3000);
  int cb = udp.parsePacket();
  if (!cb) {
    udp.flush();
    udp.stop();
    return 0UL;
  }
  udp.read(ntpPacketBuffer, NTP_PACKET_SIZE);
  udp.flush();
  udp.stop();
    
  unsigned long highWord = word(ntpPacketBuffer[40], ntpPacketBuffer[41]);
  unsigned long lowWord = word(ntpPacketBuffer[42], ntpPacketBuffer[43]);
  return (unsigned long) highWord << 16 | lowWord;
}

void waitForWifiConnection() {
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && (retries < MAX_WIFI_CONNECT_DELAY)) {
    retries++;
    delay(500);
   }
  if (WiFi.status() != WL_CONNECTED) {
   restart();
  }
}

int readCo2Sensor() {
  byte command[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
  byte response[] = {0,0,0,0,0,0,0};
  
  while (!Serial.available()) {
    Serial.write(command, 7);
    delay(50);
  }
  
  byte timeout = 0;
  while (Serial.available() < 7 ) {
    timeout++;  
    if (timeout > 10) {
        while(Serial.available())
          Serial.read();
          break;
    }
    delay(50);
  }
  for (int i = 0; i < 7; i++) {
    response[i] = Serial.read();
  }
  return response[3] * 256 + response[4];
}

void getCo2Data() {
  int co2 = readCo2Sensor();
  String payload = "{\"level\": ";
  payload += co2;
  payload += "}";
  publishMqtt(MQTT_CO2_TOPIC, payload);
}

void publishMqtt(String pubTopic, String payload) {
  if (mqttClient.connected()){
    if (!mqttClient.publish(pubTopic, (char*) payload.c_str())) {
      restart();
    }
  }
  else {
    if (mqttClient.connect(MQTT::Connect(MQTT_CLIENT).set_auth(MQTT_USER, MQTT_PASS))) {
      publishMqtt(pubTopic, payload);
    }
    else {
      restart();
    }
  } 
}

void showTime() {
  int hours = hour();
  int minutes = minute();

  if (hours > 12) {
    hours -= 12;
  }
  displayTime(0, hours / 10);
  displayTime(1, (hours - ((hours / 10) * 10)));
    
  if (minutes < 10) {
    displayTime(2, 0);
  }
  displayTime(2, minutes / 10);
  displayTime(3, (minutes - ((minutes / 10) * 10)));
}

void restart() {
  //abort();
}

void bh1750Init() {
  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.write(BH1750_CONTINUOUS_LOW_RES_MODE);
  Wire.endTransmission();
}

void readLightLevel() {
  unsigned int level;

  Wire.beginTransmission(BH1750_I2C_ADDRESS);
  Wire.requestFrom(BH1750_I2C_ADDRESS, 2);
  level = Wire.read();
  level <<= 8;
  level |= Wire.read();
  Wire.endTransmission();

  level = level / 1.2; // convert to lux

  if (level != lastLightLevel) {
    lastLightLevel = level;

    byte brightness = level > NIGHT_MODE_CLOCK_LUX_LEVEL ? MAX_CLOCK_BRIGHTNESS : MIN_CLOCK_BRIGHTNESS;
    ledDisp.setIntensity(0, brightness);
    ledDisp.setIntensity(1, brightness);
  
    String payload = "{\"level\": ";
    payload += level;
    payload += "}";
    publishMqtt(MQTT_LIGHT_LEVEL_TOPIC, payload);
  }  
} 

void motionSensorInit() {
  pinMode(MOTION_SENSOR_PIN, INPUT);
}

void readMotionSensor() {
  boolean sensorState = digitalRead(MOTION_SENSOR_PIN);
  if (sensorState != lastMotionSensorState) {
    lastMotionSensorState = sensorState;
    
    String payload = "{\"state\": ";
    payload += sensorState;
    payload += "}";
    publishMqtt(MQTT_MOTION_SENSOR_TOPIC, payload);
  }
}

void ledDisplayInit() {
  ledDisp.shutdown(0, false);
  ledDisp.shutdown(1, false);

  ledDisp.setIntensity(0, MIN_CLOCK_BRIGHTNESS);
  ledDisp.setIntensity(1, MIN_CLOCK_BRIGHTNESS);
  
  ledDisp.clearDisplay(0);
  ledDisp.clearDisplay(1);
}

void co2SensorInit() {
  Serial.begin(9600); 
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  bh1750Init();
  motionSensorInit();
  ledDisplayInit();
  co2SensorInit();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  
  waitForWifiConnection();

  delay(1000);
  setSyncProvider(getNTPtime);
  setSyncInterval(NTP_SERVER_UPDATE_INTERVAL);

  showTime();

  Alarm.timerRepeat(CO2_UPDATE_INTERVAL, getCo2Data);
  Alarm.timerRepeat(TIME_UPDATE_INTERVAL, showTime);

  blinker.attach(TIME_TICK_UPDATE_INTERVAL, changeClockTick); 
}

void loop() {
  readLightLevel();
  readMotionSensor();
  Alarm.delay(100);
}
