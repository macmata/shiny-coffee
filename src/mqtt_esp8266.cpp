#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <max6675.h>
#include <PID_v1.h>

/**************************Wifi Manager ************************************/

WiFiManager wifiManager;
char buffer [10];
char mqtt_server [40];
char mqtt_server_publish [40];
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_publish("server_publish", "mqtt server publish", mqtt_server_publish, 40);

WiFiClient wiFiClient;
PubSubClient mqtt_client(wiFiClient);
/************************* Thermo *********************************/

#define THERMO_CS D3
#define DC    D4      //Also A0     Data command
#define SCLK  D5
#define SSR_PID D1

MAX6675 thermocouple(SCLK, THERMO_CS, DC);

double Reading, Setpoint = 96, Input, Output;
double Kp=17.37, Ki=155.5, Kd=93.87;
int WindowSize = 5000;
unsigned long windowStartTime;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*************************** Sketch Code ************************************/

void reconnect() {
    // Loop until we're reconnected
    while (!mqtt_client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (mqtt_client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqtt_client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setup() {

    Serial.begin(115200);
    delay(10);

    //pinMode(SSR_PID, OUTPUT);

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_publish);
    wifiManager.startConfigPortal("shiny_config","1234567890");


    Serial.print("Connecting to ");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_server_publish, custom_mqtt_publish.getValue());

    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("mqtt_server"); Serial.println(mqtt_server);
    Serial.print("mqtt_publish"); Serial.println(mqtt_server_publish);


    mqtt_client.setServer(mqtt_server,1883);

    if (!mqtt_client.connected()) {
        reconnect();
    }

    mqtt_client.publish("espresso/mqtt/status","on");

}


void loop() {

    if (!mqtt_client.connected()) {
        reconnect();
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

    Serial.print("TEMPS");
    Serial.println(Input);

    dtostrf(Input,10,2,buffer);
    
    mqtt_client.publish("espresso/temps", buffer);
}


