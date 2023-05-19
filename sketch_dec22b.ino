#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>

#include <EEPROM.h>

//Important Config
#define PUMP_PIN 6
#define SW_PIN 4
#define MOIST_BORDER 450
#define MOIST_BORDER_LOWEST 550
#define PUMP_DURATION 10000

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_ADDR   0x3C
#define OLED_RESET -1   //   QT-PY / XIAO

Adafruit_SH1106G display = Adafruit_SH1106G(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key index number (needed only for WEP)

int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 12     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated


// Start RTC
#include "RTClib.h"

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 2

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {
//Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Start RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //Disable SQW to enable interupts
  rtc.writeSqwPinMode(DS3231_OFF);

  //we don't need the 32K Pin, so disable it
    rtc.disable32K();
  
  // Making it so, that the alarm will trigger an interrupt
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  
  // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
  // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(1);
  Serial.println("==--===--==");
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = (sensor.min_delay / 1000);
  Serial.println(delayMS);

  display.begin(OLED_ADDR, true);
  display.display();
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println("Welcome");

  display.setTextSize(2);
  display.setTextColor(1);
  display.setCursor(0, 17);
  display.println("This is");
  display.println("a Splash");
  display.println("Screen");

  display.display();


  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);      // set the LED pin mode

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);
    sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));

   }


  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();


  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println("- Network");

  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 17);
  display.println(WiFi.localIP());

  display.display();

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT);

}

void loop() {
      // Read DHT
    unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= delayMS) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }
    display.fillRect(0, 34, 128, 28, 0);

    //Update OLED
    display.setTextSize(1);
    display.setTextColor(1);
    //Print Temperature to OLED
    display.setCursor(0, 45);
    display.println("T:");
    display.setCursor(10, 45);
    display.println(event.temperature);

    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }

    //Print Humidity to OLED
    display.setCursor(0, 55);
    display.println("H:");
    display.setCursor(10, 55);
    display.println(event.relative_humidity);


    //TIME
    display.setCursor(115, 55);
    display.println(rtc.now().second());

    display.setCursor(95, 55);
    display.println(rtc.now().minute());

    display.setCursor(75, 55);
    display.println(rtc.now().hour());

    display.setCursor(105, 55);
    display.println("::");
    display.setCursor(85, 55);
    display.println(":");
    display.setCursor(45, 55);
    display.println("TIME: ");

    //ALARM
    display.setCursor(115, 45);
    display.println(rtc.getAlarm2().second());

    display.setCursor(95, 45);
    display.println(rtc.getAlarm2().minute());

    display.setCursor(75, 45);
    display.println(rtc.getAlarm2().hour());

    display.setCursor(105, 45);
    display.println("::");
    display.setCursor(85, 45);
    display.println(":");
    display.setCursor(45, 45);
    display.println("ALRM: ");
    
    //MOISTURE
    display.setCursor(0,35);
    display.println("MOIST: ");
    display.setCursor(35,35);
    display.println(analogRead(A0));
    //Border
    display.setCursor(0,25);
    display.println("MOIST_BORDER: ");
    display.setCursor(90,25);
    display.println(MOIST_BORDER);
  
    //Magic Number 450 and under --> WET
    //Display Switch-State
    display.setCursor(60, 35);
    display.println("CTRL: ");
    if (digitalRead(SW_PIN) == HIGH){
      display.setCursor(90,35);
      display.println("Yes");
    }
    else {
      display.setCursor(90,35);
      display.println("No");
    }
    //Display Password for AP
    display.setCursor(69,16);
    display.println(pass);
    

    display.display();

    if(rtc.alarmFired(2)){
      Serial.print("HELLO ALARM");
      rtc.clearAlarm(2);
      delay(PUMP_DURATION);
      digitalWrite(PUMP_PIN, LOW);
      //TODO: Find Better Way for this maybe?
    }
    Serial.println(rtc.getAlarm2().minute());
    }
    // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentData = "";
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected

      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");
            client.print(" <form action=\"/form\" method=\"get\"> <input type=\"number\" id=\"name\" name=\"user_name\" /> <button type=\"submit\">Send your message</button> </form> ");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            if (currentLine.indexOf("?") == 9){
              Serial.println("FOUND IT");
              currentData = currentLine;
              Serial.println(currentData);
              Serial.println("------------");
              Serial.println(currentData.substring(currentData.indexOf("=")+1, currentData.length()-9));
              setAlarmForWater(currentData.substring(currentData.indexOf("=")+1, currentData.length()-9));
            }
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(led, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led, LOW);                // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /form")) {
          digitalWrite(led, LOW);                // GET /L turns the LED off

          //Serial.println(currentLine.substring(currentLine.indexOf(0, "=")).substring(currentLine.indexOf(0,"H")));
        }
        
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");


}
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}


void setAlarmForWater(String currentData){
  display.setCursor(40,40);  
  display.println(currentData);
  display.display();
  //Formating
  int hours = currentData.substring(0,2).toInt();
  int minutes = currentData.substring(2,4).toInt();
  //Set alarm

  rtc.clearAlarm(2);
  if(!rtc.setAlarm2(
           DateTime(rtc.now().year(), rtc.now().month(), rtc.now().day(), hours, minutes, 0),
            DS3231_A2_Hour // this mode triggers the alarm when the seconds match. See Doxygen for other options
  )){
    Serial.println("ERERERERRERRREFERGFDFDBFD");
  }
  /*rtc.setAlarm2(
           rtc.now()+ TimeSpan(70),
            DS3231_A2_PerMinute // this mode triggers the alarm when the seconds match. See Doxygen for other options
  );*/
  Serial.println("SetAlm to: ");
  Serial.print( hours );
  Serial.print("  ");
  Serial.println(minutes);
}

void onAlarm() {  
  Serial.println("Alarm occured!");
  //Check if Control-Conditions are met
  if(digitalRead(SW_PIN)== HIGH){
    //Control enabled
    //Check Soil hudidity
    if(analogRead(A0) > MOIST_BORDER){
      //Time -> noon
      if(rtc.now().hour() == 12 || rtc.now().hour() == 11 || rtc.now().hour() == 13){
        if(analogRead(A0) > MOIST_BORDER_LOWEST){
          digitalWrite(PUMP_PIN, HIGH);   
        }
      }
      else{
        digitalWrite(PUMP_PIN, HIGH);
      }
    }
    return;
  }
  else{
    //Control disabled
    digitalWrite(PUMP_PIN, HIGH);
  }

}