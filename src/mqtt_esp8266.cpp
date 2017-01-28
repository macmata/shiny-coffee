#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <max6675.h>
#include <PID_v1.h>

/**************************Wifi Manager ************************************/

WiFiManager wifiManager;

char mqtt_server [40];
char mqtt_server_publish [40];
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_publish("server_publish", "mqtt server publish", mqtt_server_publish, 40);

WiFiClient wiFiClient;
PubSubClient mqtt_client(wiFiClient);
/************************* Thermo *********************************/

/*#define THERMO_CS D3
#define DC    D4      //Also A0     Data command
#define SCLK  D5
#define SSR_PID D1

MAX6675 thermocouple(SCLK, THERMO_CS, DC);

double Reading, Setpoint = 96, Input, Output;
double Kp=7, Ki=97, Kd=18;
int WindowSize = 5000;
unsigned long windowStartTime;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);*/
/*************************** Sketch Code ************************************/


void setup() {

    Serial.begin(115200);
    delay(10);

    //pinMode(SSR_PID, OUTPUT);

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_publish);
    wifiManager.autoConnect("shiny_machine", "1234567890");


    Serial.print("Connecting to ");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("mqtt_server"); Serial.println(mqtt_server);
    Serial.print("mqtt_publish"); Serial.println(mqtt_server_publish);

    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_server_publish, custom_mqtt_publish.getValue());

    mqtt_client.setServer(mqtt_server,1883);

}


void loop() {

    /*Input = thermocouple.readCelsius();
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


    Serial.print("TEMPS");
    Serial.println(Input);*/
}


