#include <HardwareSerial.h>
#include <Arduino.h>
#include <Wire.h>

#define BOARD_NAME "Sensors00"

#define CMD_SENSOR_DATA 16

const int trigPin = 9;
const int echoPin = 10;

long systemTimestamp = 0;
int distance = 0;

struct Command {
  uint8_t cmd;
  long timestamp;
};

struct SensorsData {
    uint8_t cmd;
    float us_distance;
};

void updateDistance() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  
  distance = duration * 0.034 / 2;
}

void handleCommand(Command cmd) {
  switch (cmd.cmd) {
    case CMD_SENSOR_DATA:
      systemTimestamp = cmd.timestamp;
      break;
    default:
      break;
  }
}

void sendSensorData() {
  SensorsData sensors;
  sensors.cmd = CMD_SENSOR_DATA;
  sensors.us_distance = distance;
  Serial.write((byte*)&sensors, sizeof(SensorsData));
}

void receiveCommand()
{
  if (Serial.available() >= sizeof(Command))
  {
    Command receivedCommand;
    Serial.readBytes((byte *)&receivedCommand, sizeof(Command));
    handleCommand(receivedCommand);
  }
}

void setup(){
    Serial.begin(115200);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void loop() {
  updateDistance();

  receiveCommand();

  sendSensorData();

  delay(100);
}
