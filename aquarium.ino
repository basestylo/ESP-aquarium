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

#define ONE_WIRE_BUS D1
#define TEMPERATURE_PRECISION 12 
#define TZ 0

RTC_Millis RTC;                           // RTC (soft)
DateTime now;                             // current time
unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP udp;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress aquaTempDevice  = { 0x28, 0xFF, 0x45, 0xBC, 0xB5, 0x16, 0x03, 0xDD };
DeviceAddress boardTempDevice = { 0x28, 0xFF, 0x91, 0x51, 0xC1, 0x16, 0x04, 0xBA };
float tempAqua=0;
float tempBoard=0;
 
const int lightPin = D0;
const int pumpPin = D5;
const int heatherPin = D6;
boolean lightStatus = true;
boolean pumpStatus = true;
boolean heatherStatus = true;

void setup()
{ 
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
  setRelayStates();
  
  //Start routine
  obtainDate();
  oled.setTextXY(2,0);              // Set cursor position, start of line 0
  oled.putString("Starting......");
}

void loop() {    
  ArduinoOTA.handle();
  now = RTC.now();

  obtainTemperature();
  printTemperature();
  printWifiInfo();
  printDate();
  setLightStatus();
  setHeatherStatus();
  printRelayStatus();
  setRelayStates();
}

void setRelayStates() {
  digitalWrite(lightPin, lightStatus);
  digitalWrite(pumpPin, pumpStatus);
  digitalWrite(heatherPin, heatherStatus);
}

void setLightStatus() {
  lightStatus = now.hour() >= 16;
}

void setHeatherStatus() {
  heatherStatus = tempAqua < 24;
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
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(10000);
  
  int cb = udp.parsePacket();
  while (!cb) {
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
    tz = IsDST(gt.month(), gt.day(), gt.dayOfTheWeek())? TZ:TZ+1;  // if in DST correct for GMT-4 hours else GMT-5
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
}

void printDate() {
  oled.setTextXY(7,0);              // Set cursor position, start of line 0
  oled.putNumber(now.day());
  oled.putString("/");
  oled.putNumber(now.month());
  oled.putString("/");
  oled.putNumber(now.year());
  oled.setTextXY(7,11);              // Set cursor position, start of line 0
  oled.putNumber(now.hour());
  oled.putString(":");
  oled.putNumber(now.minute());
  oled.putString(" ")
}

