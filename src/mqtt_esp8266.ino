#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>

#define SSR_ALIM D2
#define SSR_PID D1

#define THERMO_CS D3
#define DC    D4      //Also A0     Data command
#define SCLK  D5
//#define MISO  D6
//#define MOSI  D7

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       ""
#define WLAN_PASS       ""

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      ""
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""
typedef struct{
  int code;
  int p;
  int i;
  int d;
 }Token;
 
/************ Global State (you don't need to change this!) ******************/

WiFiClient client;

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

const char ESPRESSO_IN[] PROGMEM = AIO_USERNAME "/espresso/state_in";
Adafruit_MQTT_Subscribe espressoIn = Adafruit_MQTT_Subscribe(&mqtt, ESPRESSO_IN);

const char ESPRESSO_OUT[] PROGMEM = AIO_USERNAME "/espresso/state_out";
Adafruit_MQTT_Publish espressoOut = Adafruit_MQTT_Publish(&mqtt, ESPRESSO_OUT);

const char TEMPS[] PROGMEM = AIO_USERNAME "/espresso/state_temps";
Adafruit_MQTT_Publish temps = Adafruit_MQTT_Publish(&mqtt, TEMPS);

/************************* Thermo *********************************/
MAX6675 thermocouple(SCLK, THERMO_CS, DC);

/*************************** Sketch Code ************************************/


void MQTT_connect();



bool state = false;
bool lastState = false;
String serial_input;
double Reading, Setpoint = 96, Input, Output;
double Kp=7, Ki=97, Kd=18;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

void setup() {

  Serial.begin(115200);
  delay(10);

  pinMode(SSR_PID, OUTPUT);
  pinMode(SSR_ALIM, OUTPUT);

  // Connect to WiFi access point.
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());

  mqtt.subscribe(&espressoIn);
  delay(5000);

}


void loop() {
  if (state) {

    digitalWrite(SSR_ALIM, HIGH);
    
    if (!lastState){
         Serial.println("PID ON");
        windowStartTime = millis();

        //tell the PID to range between 0 and the full window size
        myPID.SetOutputLimits(0, WindowSize);

        //turn the PID on
        myPID.SetMode(AUTOMATIC);     
    }

    Input = thermocouple.readCelsius();

    myPID.Compute();

    if (millis() - windowStartTime > WindowSize){ //time to shift the Relay Window
        windowStartTime += WindowSize;
    }

    if(Output < millis() - windowStartTime){
        digitalWrite(SSR_PID, HIGH);
        Serial.println("PID HIGH");
    }else{ 
        digitalWrite(SSR_PID, LOW);
        Serial.println("PID LOW");
    }
        
    temps.publish(Input);
    espressoOut.publish("ON");

    Serial.print("TEMPS");
    Serial.println(Input);
    
    lastState =  state;

  } else {
    digitalWrite(SSR_ALIM, LOW);
    digitalWrite(SSR_PID, LOW);
    espressoOut.publish("OFF");
    lastState = state;
  }
  MQTT_manager();

  getString(serial_input);

  manageCommand(serial_input);

}

Token parseCommand(String & s){
    for (int i =0 ; i< CODE_SIZE; i++){
       if(s.startsWith(CODE_ARRAY[i])){
                
       }
    }
}
void manageCommand(String & s){
   
}

void getString(String & s){
    
    if (Serial.available()){
        char c;
        c = Serial.read();
        while(c != '\n' || c != '\r'){
            s += c;
            c = Serial.read();
        }   
    }
}

void MQTT_manager(){

  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &espressoIn) {
      if (strcmp((char *)espressoIn.lastread, "1") == 0) {
        state = true;
        Serial.println("should switch to on");
      } else if (strcmp((char *)espressoIn.lastread, "0") == 0) {
        state = false;
        Serial.println("should switch to off");
      }
    }
  }

  if (! mqtt.ping()) {
    mqtt.disconnect();
  }
    
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
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
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}


