#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define CHANNEL 1
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 25
#define SERVO5_PIN 26
#define SERVO6_PIN 27

Servo servo1, servo2, servo3, servo4, servo5, servo6;

int currentAngle1 = 90;
int currentAngle2 = 90;
int currentAngle3 = 90;
int currentAngle4 = 90;
int currentAngle5 = 90;
int currentAngle6 = 90;

typedef struct {
  uint8_t servo1Signal;
  uint8_t servo2Signal;
  uint8_t servo3Signal;
  uint8_t servo4Signal;
  uint8_t servo5Signal;
  uint8_t servo6Signal;
} ServoControl;
ServoControl servoData;


//Change veriables for speed
unsigned long lastServoUpdate1 = 0, lastServoUpdate2 = 0, lastServoUpdate3 = 0;
unsigned long lastServoUpdate4 = 0, lastServoUpdate5 = 0, lastServoUpdate6 = 0;
const int updateInterval = 5; //Reduced to 5ms for faster updates (10 for low speed)
const int STEP_SIZE_BUTTON = 4; //Unchanged for servos 1 and 6 (4 for low speed)
const int STEP_SIZE_JOYSTICK = 24; //Increased to 24 for very fast movement (16 for low speed)
const int INTERPOLATION_STEP = 3; //3-degree increments for smoothness (2 for low speed)


//Logic of servo-1
void rotateServo1(uint8_t signal) {
  if (millis() - lastServoUpdate1 >= updateInterval) {
    if (signal == 1 && currentAngle1 < 180) {
      currentAngle1 = min(currentAngle1 + STEP_SIZE_BUTTON, 180);
      servo1.write(currentAngle1);
      Serial.println("Servo1: Increasing angle");
    } else if (signal == 2 && currentAngle1 > 0) {
      currentAngle1 = max(currentAngle1 - STEP_SIZE_BUTTON, 0);
      servo1.write(currentAngle1);
      Serial.println("Servo1: Decreasing angle");
    } else {
      servo1.write(currentAngle1);
      Serial.println("Servo1: Holding angle");
    }
    lastServoUpdate1 = millis();
  }
}


//Logic of servo-2
void rotateServo2(uint8_t signal) {
  if (millis() - lastServoUpdate2 >= updateInterval) {
    int targetAngle = currentAngle2;
    if (signal == 3 && currentAngle2 < 180) {
      targetAngle = min(currentAngle2 + STEP_SIZE_JOYSTICK, 180);
      Serial.println("Servo2: Increasing angle");
    } else if (signal == 4 && currentAngle2 > 0) {
      targetAngle = max(currentAngle2 - STEP_SIZE_JOYSTICK, 0);
      Serial.println("Servo2: Decreasing angle");
    } else {
      Serial.println("Servo2: Holding angle");
    }
    if (targetAngle != currentAngle2) {
      int step = (targetAngle > currentAngle2) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
      currentAngle2 = constrain(currentAngle2 + step, min(currentAngle2, targetAngle), max(currentAngle2, targetAngle));
      servo2.write(currentAngle2);
    }
    lastServoUpdate2 = millis();
  }
}


//Logic of servo-3
void rotateServo3(uint8_t signal) {
  if (millis() - lastServoUpdate3 >= updateInterval) {
    int targetAngle = currentAngle3;
    if (signal == 6 && currentAngle3 < 180) {
      targetAngle = min(currentAngle3 + STEP_SIZE_JOYSTICK, 180);
      Serial.println("Servo3: Increasing angle");
    } else if (signal == 7 && currentAngle3 > 0) {
      targetAngle = max(currentAngle3 - STEP_SIZE_JOYSTICK, 0);
      Serial.println("Servo3: Decreasing angle");
    } else {
      Serial.println("Servo3: Holding angle");
    }
    if (targetAngle != currentAngle3) {
      int step = (targetAngle > currentAngle3) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
      currentAngle3 = constrain(currentAngle3 + step, min(currentAngle3, targetAngle), max(currentAngle3, targetAngle));
      servo3.write(currentAngle3);
    }
    lastServoUpdate3 = millis();
  }
}


//Logic of servo-4
void rotateServo4(uint8_t signal) {
  if (millis() - lastServoUpdate4 >= updateInterval) {
    int targetAngle = currentAngle4;
    if (signal == 9 && currentAngle4 < 180) {
      targetAngle = min(currentAngle4 + STEP_SIZE_JOYSTICK, 180);
      Serial.println("Servo4: Increasing angle");
    } else if (signal == 10 && currentAngle4 > 0) {
      targetAngle = max(currentAngle4 - STEP_SIZE_JOYSTICK, 0);
      Serial.println("Servo4: Decreasing angle");
    } else {
      Serial.println("Servo4: Holding angle");
    }
    if (targetAngle != currentAngle4) {
      int step = (targetAngle > currentAngle4) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
      currentAngle4 = constrain(currentAngle4 + step, min(currentAngle4, targetAngle), max(currentAngle4, targetAngle));
      servo4.write(currentAngle4);
    }
    lastServoUpdate4 = millis();
  }
}


//Logic of servo-5
void rotateServo5(uint8_t signal) {
  if (millis() - lastServoUpdate5 >= updateInterval) {
    int targetAngle = currentAngle5;
    if (signal == 12 && currentAngle5 < 180) {
      targetAngle = min(currentAngle5 + STEP_SIZE_JOYSTICK, 180);
      Serial.println("Servo5: Increasing angle");
    } else if (signal == 13 && currentAngle5 > 0) {
      targetAngle = max(currentAngle5 - STEP_SIZE_JOYSTICK, 0);
      Serial.println("Servo5: Decreasing angle");
    } else {
      Serial.println("Servo5: Holding angle");
    }
    if (targetAngle != currentAngle5) {
      int step = (targetAngle > currentAngle5) ? INTERPOLATION_STEP : -INTERPOLATION_STEP;
      currentAngle5 = constrain(currentAngle5 + step, min(currentAngle5, targetAngle), max(currentAngle5, targetAngle));
      servo5.write(currentAngle5);
    }
    lastServoUpdate5 = millis();
  }
}


//Logic of servo-6
void rotateServo6(uint8_t signal) {
  if (millis() - lastServoUpdate6 >= updateInterval) {
    if (signal == 15 && currentAngle6 < 180) {
      currentAngle6 = min(currentAngle6 + STEP_SIZE_BUTTON, 180);
      servo6.write(currentAngle6);
      Serial.println("Servo6: Increasing angle");
    } else if (signal == 16 && currentAngle6 > 0) {
      currentAngle6 = max(currentAngle6 - STEP_SIZE_BUTTON, 0);
      servo6.write(currentAngle6);
      Serial.println("Servo6: Decreasing angle");
    } else {
      servo6.write(currentAngle6);
      Serial.println("Servo6: Holding angle");
    }
    lastServoUpdate6 = millis();
  }
}


void OnDataReceived(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(ServoControl)) {
    memcpy(&servoData, data, sizeof(ServoControl));
    Serial.print("Received: S1="); Serial.print(servoData.servo1Signal);
    Serial.print(" S2="); Serial.print(servoData.servo2Signal);
    Serial.print(" S3="); Serial.print(servoData.servo3Signal);
    Serial.print(" S4="); Serial.print(servoData.servo4Signal);
    Serial.print(" S5="); Serial.print(servoData.servo5Signal);
    Serial.print(" S6="); Serial.println(servoData.servo6Signal);
    rotateServo1(servoData.servo1Signal);
    rotateServo2(servoData.servo2Signal);
    rotateServo3(servoData.servo3Signal);
    rotateServo4(servoData.servo4Signal);
    rotateServo5(servoData.servo5Signal);
    rotateServo6(servoData.servo6Signal);
  } else {
    Serial.println("Invalid data length");
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Receiver Started");
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo1.write(currentAngle1);
  servo2.attach(SERVO2_PIN, 500, 2500);
  servo2.write(currentAngle2);
  servo3.attach(SERVO3_PIN, 500, 2500);
  servo3.write(currentAngle3);
  servo4.attach(SERVO4_PIN, 500, 2500);
  servo4.write(currentAngle4);
  servo5.attach(SERVO5_PIN, 500, 2500);
  servo5.write(currentAngle5);
  servo6.attach(SERVO6_PIN, 500, 2500);
  servo6.write(currentAngle6);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(CHANNEL);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }
  esp_now_register_recv_cb(OnDataReceived);
}


unsigned long lastUpdateTime = 0;
void loop() {
  if (millis() - lastUpdateTime >= 1000) {
    Serial.print("S1: "); Serial.print(servoData.servo1Signal);
    Serial.print(" S2: "); Serial.print(servoData.servo2Signal);
    Serial.print(" S3: "); Serial.print(servoData.servo3Signal);
    Serial.print(" S4: "); Serial.print(servoData.servo4Signal);
    Serial.print(" S5: "); Serial.print(servoData.servo5Signal);
    Serial.print(" S6: "); Serial.print(servoData.servo6Signal);
    Serial.print(" A1: "); Serial.print(currentAngle1);
    Serial.print(" A2: "); Serial.print(currentAngle2);
    Serial.print(" A3: "); Serial.print(currentAngle3);
    Serial.print(" A4: "); Serial.print(currentAngle4);
    Serial.print(" A5: "); Serial.print(currentAngle5);
    Serial.print(" A6: "); Serial.println(currentAngle6);
    lastUpdateTime = millis();
  }
}