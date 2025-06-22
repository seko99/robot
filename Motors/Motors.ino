#include <HardwareSerial.h>
#include <Arduino.h>
#include <Wire.h>

#define CMD_LEFT_FORWARD 0
#define CMD_LEFT_BACKWARD 1
#define CMD_RIGHT_FORWARD 2
#define CMD_RIGHT_BACKWARD 3
#define CMD_LEFT_STOP 4
#define CMD_RIGHT_STOP 5
#define CMD_STOP_MOTORS 6

#define TICKS_IN_CYCLE 233
#define RIGHT_H1_PIN 2
#define RIGHT_H2_PIN 3
#define LEFT_H1_PIN 4
#define LEFT_H2_PIN 12
#define TICKS_IN_METER 0.94 / 1000
// 1058 1041 1035 1072
#define LEFT_TICKS_IN_METER 1.052 / 1000
// 1059 1047 1047 1056
#define RIGHT_TICKS_IN_METER 1.051 / 1000
#define STBY 11

const bool motorsEnabled = true;

// Пины для первого двигателя (Motor A, left)
const int in1 = 5;
const int in2 = 6;
const int enA = 9;

// Пины для второго двигателя (Motor B, right)
const int in3 = 7;
const int in4 = 8;
const int enB = 10;

struct Command
{
  uint8_t cmd;
  uint8_t value;
};

struct SensorData
{
  uint32_t timestamp;
  float left_odometry;
  float right_odometry;
  long left_ticks;
  long right_ticks;
  bool left_h2_state;
  bool right_h2_state;
  bool left_flag;
  bool right_flag;
};

bool left_flag = false;
bool right_flag = false;
bool left_h2_state = false;
bool right_h2_state = false;
long left_ticks = 0;
long right_ticks = 0;

long t = millis();
float odomLeft = 0.0;
float odomRight = 0.0;

void handleCommand(Command cmd)
{
  switch (cmd.cmd)
  {
  case CMD_LEFT_FORWARD:
    moveLeftForward(cmd.value);
    break;
  case CMD_LEFT_BACKWARD:
    moveLeftBackward(cmd.value);
    break;
  case CMD_RIGHT_FORWARD:
    moveRightForward(cmd.value);
    break;
  case CMD_RIGHT_BACKWARD:
    moveRightBackward(cmd.value);
    break;
  case CMD_LEFT_STOP:
    stopLeftMotor();
    break;
  case CMD_RIGHT_STOP:
    stopRightMotor();
    break;
  case CMD_STOP_MOTORS:
    stopMotors();
    break;
  default:
    break;
  }
}

void moveRightForward(int speed)
{
  if (!motorsEnabled)
    return;

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
}

void moveRightBackward(int speed)
{
  if (!motorsEnabled)
    return;

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
}

void moveLeftForward(int speed)
{
  if (!motorsEnabled)
    return;

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, speed);
}

void moveLeftBackward(int speed)
{
  if (!motorsEnabled)
    return;

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, speed);
}

void stopRightMotor()
{
  if (!motorsEnabled)
    return;

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

void stopLeftMotor()
{
  if (!motorsEnabled)
    return;

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}

void stopMotors()
{
  stopLeftMotor();
  stopRightMotor();
}

float updateLeftOdometry()
{
  bool state = digitalRead(LEFT_H1_PIN);
  if (state && !left_flag)
  { // фронт LOW-HIGH
    left_flag = true;
  }
  if (!state && left_flag)
  { // фронт HIGH-LOW
    left_flag = false;
  }

  bool new_h2_state = digitalRead(LEFT_H2_PIN);
  if (state && !left_h2_state && new_h2_state)
  {
    left_ticks++;
  }

  if (state && left_h2_state && !new_h2_state)
  {
    left_ticks--;
  }

  left_h2_state = new_h2_state;

  return left_ticks * TICKS_IN_METER;

  // 233 ticks == 1 оборот - замер
  // 69.60*3.14= 218.54 mm длина окружности - рассчёт
  // 1 tick == 218.54/2s33 = 0.94 mm
}

float updateRightOdometry()
{
  bool state = digitalRead(RIGHT_H1_PIN);
  if (state && !right_flag)
  { // фронт LOW-HIGH
    right_flag = true;
  }
  if (!state && right_flag)
  { // фронт HIGH-LOW
    right_flag = false;
  }

  bool new_h2_state = digitalRead(RIGHT_H2_PIN);
  if (state && !right_h2_state && new_h2_state)
  {
    right_ticks--;
  }

  if (state && right_h2_state && !new_h2_state)
  {
    right_ticks++;
  }

  right_h2_state = new_h2_state;

  return right_ticks * TICKS_IN_METER;

  // 233 ticks == 1 оборот - замер
  // 69.60*3.14= 218.54 mm длина окружности - рассчёт
  // 1 tick == 218.54/2s33 = 0.94 mm
}

void setup()
{
  Serial.begin(115200);

  left_h2_state = digitalRead(LEFT_H2_PIN);
  right_h2_state = digitalRead(RIGHT_H2_PIN);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(STBY, OUTPUT);

  stopMotors();

  digitalWrite(STBY, HIGH);
}

void sendSensorData(SensorData sensorData)
{
  Serial.write((byte *)&sensorData, sizeof(SensorData));
}

void loop()
{
  if (Serial.available() >= sizeof(Command))
  {
    Command receivedCommand;
    Serial.readBytes((byte *)&receivedCommand, sizeof(Command));
    handleCommand(receivedCommand);
  }

  odomLeft = updateLeftOdometry();
  odomRight = updateRightOdometry();

  if (millis() - t > 50)
  {
    t = millis();

    SensorData sensorData;
    sensorData.timestamp = millis();
    sensorData.left_odometry = odomLeft;
    sensorData.right_odometry = odomRight;
    sensorData.left_ticks = left_ticks;
    sensorData.right_ticks = right_ticks;
    sensorData.left_h2_state = left_h2_state;
    sensorData.right_h2_state = right_h2_state;
    sensorData.left_flag = left_flag;
    sensorData.right_flag = right_flag;

    sendSensorData(sensorData);
  }
}
