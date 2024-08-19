#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1
#define SERVO_X 13
#define SERVO_Y 15 

Servo servoMotorX;
Servo servoMotorY;

// servoMotor2.write(pos);


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
int newData[2];


void setup() 
{
  servoMotorX.attach(SERVO_X);  
  servoMotorY.attach(SERVO_Y);
  Serial.begin(115200);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP("RXNO", "RXNO_password", CHANNEL, 0);
  
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    servoMotorX.write(newData[1]);
    servoMotorY.write(180 - newData[0]); 
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(&newData, data, sizeof(newData));
}
