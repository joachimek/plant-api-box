#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SoftwareSerial.h>

#define DHTTYPE DHT11 

int tempSensorPin = 2;

int lightSensorPin = A0;

int soilHumidityPin = A2;

int pumpIn1Pin = 10;
int pumpIn2Pin = 9;
int pumpEnablePin = 11;

DHT dht = DHT(tempSensorPin, DHT11);

SoftwareSerial ESPserial(0, 1); // RX | TX

String ESPCommand(const char *toSend, unsigned long milliseconds) {
  String result;
  Serial.print("Sending: ");
  Serial.println(toSend);
  //sending command to esp
  ESPserial.println(toSend);
  unsigned long startTime = millis();
  Serial.print("Received: ");
  //timeouting
  while (millis() - startTime < milliseconds) {
    //receiving result from esp
    if (ESPserial.available()) {
      char c = ESPserial.read();
      Serial.write(c);
      result += c;  // append to the result string
    }
  }
  Serial.println();  // new line after timeout.
  return result;
}

void ConnectToWifi(String ssid, String password) {
  while(true){
    ESPCommand("AT+CWJAP_CUR="+ssid+","+password, 30000)
    if(AT+CIPSTATUS){
      int pingResult = ESPCommand("AT+CIPSTATUS", 30000)
      //todo: break loop
      }
  }
}

void setup() {
  Serial.begin(9600);        
  pinMode(0, OUTPUT);
  dht.begin();
  ConnectToWifi("2.4G-Vectra-WiFi-C142DB", "uqr4bli7rinyxvnd");
}

void ReceiveTemperature() {
  int temperature = dht.readTemperature();
  int humidity = dht.readHumidity();
  Serial.println(humidity);
  Serial.println(temperature);
}

void ReceiveLight() {
  int lightSensorReading = analogRead(lightSensorPin);      
  Serial.print(lightSensorReading); 
  Serial.println("");
}

void ReceiveSoilHumidity() {
  int sensorValue = analogRead(A0);
  float percentage = (float)((sensorValue) * 100) / (950);
  Serial.print(percentage); 
  Serial.println("");
}

void PingPump() {
  analogWrite(pumpEnablePin, 30);
  digitalWrite(pumpIn1Pin, HIGH);
  delay(500);
  digitalWrite(pumpIn1Pin, LOW);
}

void loop() {
  ReceiveLight();
  ReceiveTemperature();
  ReceiveSoilHumidity();
  
  delay(10000);
}