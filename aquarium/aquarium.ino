#include <Wire.h>
#include <ACROBOTIC_SSD1306.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <TaskScheduler.h>

#define ONE_WIRE_BUS D1
#define TEMPERATURE_PRECISION 12
#define TZ 0
#define MAX_AQUA_TEMP 22
#define FORCED_AQUA_COOLING_TEMP 24
#define RELAY_DEBOUNCE_SECS 300

#define AIO_SERVER      "mqtt.senseapp.space"
#define AIO_SERVERPORT  1883                   // 8883 for MQTTS
#define AIO_USERNAME    "*******"
#define AIO_KEY         "*******"

RTC_Millis RTC;                           // RTC (soft)
DateTime now;                             // current time
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "0.es.pool.ntp.org";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP udp;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish water_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/aquarium/metric/water-temp");
Adafruit_MQTT_Publish air_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/aquarium/metric/air-temp");

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress aquaTempDevice  = { 0x28, 0xFF, 0x45, 0xBC, 0xB5, 0x16, 0x03, 0xDD };
DeviceAddress boardTempDevice = { 0x28, 0xFF, 0x91, 0x51, 0xC1, 0x16, 0x04, 0xBA };
float tempAqua = 0;
int lastStateChangeHeather = 0;
int lastStateChangePump = 0;
int lastStateChangeFan = 0;
int lastStateChangeLight = 0;

float tempBoard = 0;

const int ledStatusPin = D8;
const int lightPin = D0;
const int pumpPin = D5;
const int heatherPin = D6;
const int fanPin = D7;
boolean lightStatus = false;
boolean pumpStatus = true;
boolean heatherStatus = true;
boolean fanStatus = true;
boolean ledStatus = true;

void updateDeviceStates(){
  setRelayStates();
  setLightStatus();
  setHeatherStatus();
  setFanStatus();
  printRelayStatus();
}

void updateTemperatures(){
  obtainTemperature();
  printTemperature();
}
void sendDataByMqtt(){
  water_temp.publish(tempAqua);
  air_temp.publish(tempBoard);
}

void updateDate(){
  now = RTC.now();
  printDate();
}

Task tUpdateDeviceStates(10000, TASK_FOREVER, &updateDeviceStates);
Task tSendDataByMqtt(10000, TASK_FOREVER, &sendDataByMqtt);
Task tUpdateTemperatures(5000, TASK_FOREVER, &updateTemperatures);
Task tUpdateDate(1000, TASK_FOREVER, &updateDate);

Scheduler runner;

void setup(){
  //Serial port
  Serial.begin(57600);
  Serial.println("Starting");

 //OLED
  Wire.begin(D2, D3);
  oled.init();                      // Initialze SSD1306 OLED display
  oled.clearDisplay();              // Clear screen
  oled.setTextXY(0,2);              // Set cursor position, start of line 0
  oled.putString("ESP-AQUARIUM");

  //WIFI
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  if (!wifiManager.autoConnect("ESP-aquarium")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
  }

  //NTP
  RTC.begin(DateTime(F(__DATE__), F(__TIME__)));    // initially set to compile date & time
  udp.begin(localPort);

  //TEMP SENSORS
  sensors.begin();
  sensors.setResolution(boardTempDevice, TEMPERATURE_PRECISION);
  sensors.setResolution(aquaTempDevice, TEMPERATURE_PRECISION);

  //OTA HANDLER
  ArduinoOTA.setHostname("esp-aquarium");
  ArduinoOTA.begin();

  //PIN MODE
  pinMode(lightPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(heatherPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(ledStatusPin, OUTPUT);

  obtainTemperature();
  setRelayStates();

  //Start routine
  obtainDate();
  oled.setTextXY(2,0);              // Set cursor position, start of line 0
  oled.putString("Starting......");
  printWifiInfo();

  runner.init();
  runner.addTask(tUpdateDate);
  runner.addTask(tSendDataByMqtt);
  runner.addTask(tUpdateDeviceStates);
  runner.addTask(tUpdateTemperatures);
  tUpdateDate.enable();
  tSendDataByMqtt.enable();
  tUpdateDeviceStates.enable();
  tUpdateTemperatures.enable();
}

void loop() {
  ArduinoOTA.handle();
  MQTT_connect();

  runner.execute();

  updateDateIfRequired();
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.println(mqtt.connectErrorString(ret));
      Serial.println("Retrying MQTT connection in 5 seconds...");
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
      retries--;
      if (retries == 0) {
        oled.setTextXY(2,0);              // Set cursor position, start of line 0
        oled.putString("ERROR!!!!");
        // basically die and wait for WDT to reset me
        while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

void setRelayStates() {
  if((lastStateChangeLight + RELAY_DEBOUNCE_SECS) < now.unixtime() && digitalRead(lightPin) != lightStatus){
    digitalWrite(lightPin, lightStatus);
    lastStateChangeLight = now.unixtime();
  }
  if((lastStateChangeFan + RELAY_DEBOUNCE_SECS) < now.unixtime() && digitalRead(fanPin) != !fanStatus){
    digitalWrite(fanPin, !fanStatus);
    lastStateChangeFan = now.unixtime();
  }
  if((lastStateChangePump + RELAY_DEBOUNCE_SECS) < now.unixtime() && digitalRead(pumpPin) != pumpStatus){
    digitalWrite(pumpPin, pumpStatus);
    lastStateChangePump = now.unixtime();
  }
  if((lastStateChangeHeather + RELAY_DEBOUNCE_SECS) < now.unixtime() && digitalRead(heatherPin) != heatherStatus){
    digitalWrite(heatherPin, heatherStatus);
    lastStateChangeHeather = now.unixtime();
  }
  ledStatus? analogWrite(ledStatusPin, 0):analogWrite(ledStatusPin, 200);
  ledStatus = !ledStatus;
}

void updateDateIfRequired() {
    if(now.hour() == 0 && now.minute() == 0){
      obtainDate();
    }
}

void setLightStatus() {
  lightStatus = now.hour() >= 16;
}

void setFanStatus() {
    fanStatus = tempAqua > FORCED_AQUA_COOLING_TEMP;
}

void setHeatherStatus() {
  heatherStatus = tempAqua < MAX_AQUA_TEMP;
}

unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void printTemperature() {
  oled.setTextXY(1,0);              // Set cursor position, start of line 0
  oled.putString("Aqua:      ");
  oled.putFloat(tempAqua);
  oled.setTextXY(2,0);              // Set cursor position, start of line 0
  oled.putString("Board:     ");
  oled.putFloat(tempBoard);
}

void printRelayStatus() {
  oled.setTextXY(4,1);              // Set cursor position, start of line 0
  oled.putString("HEAT LUX  PUMP");
  oled.setTextXY(5,1);              // Set cursor position, start of line 0
  (heatherStatus)? oled.putString("ON   "):oled.putString("OFF  ");
  (lightStatus)? oled.putString("ON   "):oled.putString("OFF  ");
  (pumpStatus)? oled.putString("ON   "):oled.putString("OFF  ");
  oled.setTextXY(6,3);              // Set cursor position, start of line 0
  oled.putString("FAN ");
  (fanStatus)? oled.putString("ON   "):oled.putString("OFF  ");
}


void obtainTemperature() {
  sensors.requestTemperatures();
  tempBoard = sensors.getTempC(boardTempDevice);
  tempAqua = sensors.getTempC(aquaTempDevice);
}

// IsDST(): returns true if during DST, false otherwise
boolean IsDST(int mo, int dy, int dw) {
  if (mo < 3 || mo > 11) { return false; }                // January, February, and December are out.
  if (mo > 3 && mo < 11) { return true;  }                // April to October are in
  int previousSunday = dy - dw;
  if (mo == 3) { return previousSunday >= 8; }            // In March, we are DST if our previous Sunday was on or after the 8th.
  return previousSunday <= 0;                             // In November we must be before the first Sunday to be DST. That means the previous Sunday must be before the 1st.
}

void printWifiInfo() {
  oled.setTextXY(3,0);              // Set cursor position, start of line 0
  oled.putString("IP:");
  oled.putString(WiFi.localIP().toString());
}

void obtainDate() {
    //get a random server from the pool
  oled.setTextXY(6,0);              // Set cursor position, start of line 0
  oled.putString("Updating date.");
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(10000);

  int cb = udp.parsePacket();
  int counter = 0;
  while (!cb) {
    oled.setTextXY(7,0);
    oled.putNumber(++counter);
    Serial.println("no packet yet");
    sendNTPpacket(timeServerIP); // send an NTP packet to a time server
    delay(10000);
    cb = udp.parsePacket();
  }

  Serial.print("packet received, length=");
  Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);

    int tz;                                            // adjust for EST time zone
    DateTime gt(epoch - (TZ*60*60));                       // obtain date & time based on NTP-derived epoch...
    tz = IsDST(gt.month(), gt.day(), gt.dayOfTheWeek())? TZ+2:TZ+1;  // if in DST correct for GMT-4 hours else GMT-5
    DateTime ntime(epoch + (tz*60*60));                    // if in DST correct for GMT-4 hours else GMT-5
    Serial.println("EPOCH:");
    Serial.println(epoch);
    Serial.println("TZ:GTM + ");
    Serial.println(tz);
    RTC.adjust(ntime);

    now = RTC.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    oled.setTextXY(6,0);              // Set cursor position, start of line 0
    oled.putString("                ");
}

void printDate() {
  oled.setTextXY(7,0);              // Set cursor position, start of line 0
  oled.putNumber(now.day());
  oled.putString("/");
  oled.putNumber(now.month());
  oled.putString("/");
  oled.putNumber(now.year());

  oled.setTextXY(7,11);              // Set cursor position, start of line 0
  if(now.hour() < 10){
    oled.putString("0");
  }
  oled.putNumber(now.hour());
  oled.putString(":");
  if(now.minute() < 10){
    oled.putString("0");
  }
  oled.putNumber(now.minute());
  oled.putString("  ");
}
