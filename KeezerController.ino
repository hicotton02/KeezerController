#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <gfxfont.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define HIGH_SIDE_TEMP_F 34
#define LOW_SIDE_TEMP_F 36
#define ONE_WIRE_BUS 25  // DS18B20 pin
#define FAN_RELAY 16
#define FREEZER_RELAY 17
#define FLOW_PIN_1 34
#define FLOW_PIN_2 35
#define FLOW_PIN_3 36
#define FLOW_PIN_4 37
#define FLOW_PIN_5 38
#define FLOW_PIN_6 39
#define OLED_RESET NULL

Adafruit_SSD1306 display(OLED_RESET);

const char* ssid = "Skaz";
const char* password = "File_Server";

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress coffinSensor, keezerSensor;

unsigned long previousMillisForFlow = 0;
unsigned long previousMillisForRelay = 0;
long interval = 180000; //3 minute delay for keezer compressor

//This line is the number of flow sensors connected.
const uint8_t numSensors = 6;
//This line initializes an array with the pins connected to the flow sensors
uint8_t pulsePin[] = {FLOW_PIN_1,FLOW_PIN_2,FLOW_PIN_3,FLOW_PIN_4,FLOW_PIN_5,FLOW_PIN_6};
//number of milliseconds to wait after pour before sending message
unsigned int pourMsgDelay = 300;
unsigned int pulseCount[numSensors];
unsigned int kickedCount[numSensors];
unsigned long lastPourTime = 0;
unsigned long lastPinStateChangeTime[numSensors];
int lastPinState[numSensors];

unsigned long lastSend = 0;
void setup() {
  sensors.setResolution(9);
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Booting");
  display.display();
    Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  ArduinoOTA.setHostname("keezer");

    ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  for( int i = 0; i < numSensors; i++ ) {
    pinMode(pulsePin[i], INPUT);
    digitalWrite(pulsePin[i], HIGH);
    kickedCount[i] = 0;
    lastPinState[i] = digitalRead(pulsePin[i]);
  }
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(FREEZER_RELAY, OUTPUT);
  digitalWrite(FREEZER_RELAY, LOW);
  digitalWrite(FAN_RELAY, LOW);
  if(!sensors.getAddress(keezerSensor, 0)) Serial.println("Unable to fins address for 0");
  if(!sensors.getAddress(coffinSensor, 1)) Serial.println("Unable to fins address for 1");
  sensors.setResolution(coffinSensor, 9);
  sensors.setResolution(keezerSensor, 9);
  sensors.setWaitForConversion(true);
}

void loop() {
  ArduinoOTA.handle();
  unsigned long currentMillis = millis();
  previousMillisForFlow = currentMillis;
  unsigned int topTemp;
  unsigned int bottomTemp;
  pollPins();
   if ( (currentMillis - lastPourTime) > pourMsgDelay && lastPourTime > 0) {
    //only send pour messages after all taps have stopped pulsing for a short period
    //use lastPourTime=0 to ensure this code doesn't get run constantly
    lastPourTime = 0;
    checkPours();
    checkKicks();
  }
  sensors.requestTemperatures();
  bottomTemp = (int)sensors.getTempF(keezerSensor); 
  topTemp = (int)sensors.getTempF(coffinSensor); 
  if(bottomTemp < 250 && topTemp < 250){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,16);
    display.print("Coffin Temp: ");
    display.println(topTemp);
    display.setCursor(0,30);
    display.print("Bottom Temp: ");
    display.println(bottomTemp);
    display.display();
    if((topTemp - bottomTemp) > 5){
      //Turn on fans
      if(digitalRead(FAN_RELAY) == HIGH){
        digitalWrite(FAN_RELAY, LOW);
      }
    }
    if((topTemp - bottomTemp) <= 5) {
      if(digitalRead(FAN_RELAY) == LOW){
        digitalWrite(FAN_RELAY, HIGH);
      }
    }
    if(bottomTemp >= HIGH_SIDE_TEMP_F)
    {
      if(currentMillis - previousMillisForRelay >= interval) {
        if(digitalRead(FREEZER_RELAY) == HIGH){
          //previousMillis = currentMillis;
          digitalWrite(FREEZER_RELAY, LOW);
        }
      }
    }
    if(bottomTemp <= LOW_SIDE_TEMP_F){
      //turn off Freezer
      previousMillisForRelay = currentMillis;
      digitalWrite(FREEZER_RELAY, HIGH);
    }
  }
 }
void pollPins() {
  for ( int i = 0; i < numSensors; i++ ) {
    int pinState = digitalRead(pulsePin[i]);
    if ( pinState != lastPinState[i] ) {
      if ( pinState == HIGH ) {
        //separate high speed pulses to detect kicked kegs
        if( previousMillisForFlow - lastPinStateChangeTime[i] > 0 ){
          pulseCount[i] ++;
        }
        else{
          kickedCount[i] ++;
        }
        lastPinStateChangeTime[i] = previousMillisForFlow;
        lastPourTime = previousMillisForFlow;
      }
      lastPinState[i] = pinState;
    }
  }
}

void checkPours() {
  for( int i = 0; i < numSensors; i++ ) {
    if ( pulseCount[i] > 0 ) {
      if ( pulseCount[i] > 100 ) {
      //filter out tiny bursts
        sendPulseCount(0, pulsePin[i], pulseCount[i]);
      }
      pulseCount[i] = 0;
    }
  }
}
void sendPulseCount(uint8_t addr, int pinNum, unsigned int pulseCount) {
  Serial.print("P;");
  Serial.print(addr);
  Serial.print(";");
  Serial.print(pinNum);
  Serial.print(";");
  Serial.println(pulseCount);
}

void sendKickedMsg(uint8_t addr, int pinNum) {
  Serial.print("K;");
  Serial.print(addr);
  Serial.print(";");
  Serial.println(pinNum);
}
void checkKicks() {
  for( int i = 0; i < numSensors; i++ ) {
    if ( kickedCount[i] > 0 ) {
      if ( kickedCount[i] > 30 ) {
        //if there are enough high speed pulses, send a kicked message
        sendKickedMsg(0, pulsePin[i]);
      }
      //reset the counter if any high speed pulses exist
      kickedCount[i] = 0;
    }
  }
}
