#include <esp_now.h>
#include <WiFi.h>

//Define button and channel
#define CHANNEL 1
#define BUTTON1_PIN 14
#define BUTTON2_PIN 15
#define BUTTON3_PIN 27
#define BUTTON4_PIN 26
#define JOYSTICK1_X_PIN 34
#define JOYSTICK1_Y_PIN 35
#define JOYSTICK2_X_PIN 32
#define JOYSTICK2_Y_PIN 33

//Define a structure to transfer the value to other ESP
typedef struct {
  uint8_t servo1Signal;
  uint8_t servo2Signal;
  uint8_t servo3Signal;
  uint8_t servo4Signal;
  uint8_t servo5Signal;
  uint8_t servo6Signal;
} ServoControl;
ServoControl servoData;

esp_now_peer_info_t slave;

// Joystick calibration thresholds
const float JOY1_X_MIN = 100, JOY1_X_MAX = 4000, JOY1_Y_MIN = 100, JOY1_Y_MAX = 4000;
const float JOY2_X_MIN = 100, JOY2_X_MAX = 4000, JOY2_Y_MIN = 100, JOY2_Y_MAX = 4000;
float emaX1 = 0, emaY1 = 0, emaX2 = 0, emaY2 = 0;
const float alpha = 0.5;
const int SAMPLE_COUNT = 5;

float getAverageAnalogValue(int pin) {
  float sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return sum / SAMPLE_COUNT;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void sendData() {
  const int maxRetries = 3;
  int retries = 0;
  esp_err_t result;
  do {
    result = esp_now_send(slave.peer_addr, (uint8_t *)&servoData, sizeof(servoData));
    if (result == ESP_OK) {
      break;
    }
    Serial.print("Send error: ");
    Serial.println(result);
    retries++;
    delay(10);
  } while (retries < maxRetries);
  if (result != ESP_OK) {
    Serial.println("Failed to send data after retries");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Sender Started");
  pinMode(BUTTON1_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON2_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON3_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON4_PIN, INPUT_PULLDOWN);
  pinMode(JOYSTICK1_X_PIN, INPUT);
  pinMode(JOYSTICK1_Y_PIN, INPUT);
  pinMode(JOYSTICK2_X_PIN, INPUT);
  pinMode(JOYSTICK2_Y_PIN, INPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(CHANNEL);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  memset(&slave, 0, sizeof(slave));
  uint8_t receiver_mac[6] = { 0x78, 0x42, 0x1C, 0x6d, 0x3f, 0xb4 };
  memcpy(slave.peer_addr, receiver_mac, 6);
  slave.channel = CHANNEL;
  slave.encrypt = false;
  if (esp_now_add_peer(&slave) != ESP_OK) {
    Serial.println("Peer add failed");
    while (1);
  }

  esp_now_register_send_cb(OnDataSent);
}

unsigned long lastSendTime = 0;
unsigned long lastPrintTime = 0;

void loop() {
  if (millis() - lastSendTime >= 20) {
    uint8_t button1State = digitalRead(BUTTON1_PIN);
    uint8_t button2State = digitalRead(BUTTON2_PIN);
    servoData.servo1Signal = (button1State == HIGH) ? 1 : (button2State == HIGH) ? 2 : 0;

    float x1Value = getAverageAnalogValue(JOYSTICK1_X_PIN);
    emaX1 = alpha * x1Value + (1 - alpha) * emaX1;
    float y1Value = getAverageAnalogValue(JOYSTICK1_Y_PIN);
    emaY1 = alpha * y1Value + (1 - alpha) * emaY1;

    float joy1XCenter = (JOY1_X_MIN + JOY1_X_MAX) / 2;
    float joy1XThreshold = (JOY1_X_MAX - JOY1_X_MIN) / 4;
    float joy1YCenter = (JOY1_Y_MIN + JOY1_Y_MAX) / 2;
    float joy1YThreshold = (JOY1_Y_MAX - JOY1_Y_MIN) / 4;

    if (emaX1 < joy1XCenter - joy1XThreshold) {
      servoData.servo2Signal = 3;
      servoData.servo3Signal = 8;
    } else if (emaX1 > joy1XCenter + joy1XThreshold) {
      servoData.servo2Signal = 4;
      servoData.servo3Signal = 8;
    } else {
      servoData.servo2Signal = 5;
      if (emaY1 < joy1YCenter - joy1YThreshold) servoData.servo3Signal = 6;
      else if (emaY1 > joy1YCenter + joy1YThreshold) servoData.servo3Signal = 7;
      else servoData.servo3Signal = 8;
    }

    float x2Value = getAverageAnalogValue(JOYSTICK2_X_PIN);
    emaX2 = alpha * x2Value + (1 - alpha) * emaX2;
    float y2Value = getAverageAnalogValue(JOYSTICK2_Y_PIN);
    emaY2 = alpha * y2Value + (1 - alpha) * emaY2;

    float joy2XCenter = (JOY2_X_MIN + JOY2_X_MAX) / 2;
    float joy2XThreshold = (JOY2_X_MAX - JOY2_X_MIN) / 4;
    float joy2YCenter = (JOY2_Y_MIN + JOY2_Y_MAX) / 2;
    float joy2YThreshold = (JOY2_Y_MAX - JOY2_Y_MIN) / 4;

    if (emaX2 < joy2XCenter - joy2XThreshold) {
      servoData.servo4Signal = 9;
      servoData.servo5Signal = 14;
    } else if (emaX2 > joy2XCenter + joy2XThreshold) {
      servoData.servo4Signal = 10;
      servoData.servo5Signal = 14;
    } else {
      servoData.servo4Signal = 11;
      if (emaY2 < joy2YCenter - joy2YThreshold) servoData.servo5Signal = 12;
      else if (emaY2 > joy2YCenter + joy2YThreshold) servoData.servo5Signal = 13;
      else servoData.servo5Signal = 14;
    }

    uint8_t button3State = digitalRead(BUTTON3_PIN);
    uint8_t button4State = digitalRead(BUTTON4_PIN);
    servoData.servo6Signal = (button3State == HIGH) ? 15 : (button4State == HIGH) ? 16 : 17;

    sendData();
    lastSendTime = millis();

    if (millis() - lastPrintTime >= 1000) {
      Serial.print("J1 X Raw: "); Serial.print(x1Value);
      Serial.print(" J1 X EMA: "); Serial.print(emaX1);
      Serial.print(" J1 Y Raw: "); Serial.print(y1Value);
      Serial.print(" J1 Y EMA: "); Serial.print(emaY1);
      Serial.print(" J2 X Raw: "); Serial.print(x2Value);
      Serial.print(" J2 X EMA: "); Serial.print(emaX2);
      Serial.print(" J2 Y Raw: "); Serial.print(y2Value);
      Serial.print(" J2 Y EMA: "); Serial.print(emaY2);
      Serial.print(" S1: "); Serial.print(servoData.servo1Signal);
      Serial.print(" S2: "); Serial.print(servoData.servo2Signal);
      Serial.print(" S3: "); Serial.print(servoData.servo3Signal);
      Serial.print(" S4: "); Serial.print(servoData.servo4Signal);
      Serial.print(" S5: "); Serial.print(servoData.servo5Signal);
      Serial.print(" S6: "); Serial.println(servoData.servo6Signal);
      lastPrintTime = millis();
    }
  }
}