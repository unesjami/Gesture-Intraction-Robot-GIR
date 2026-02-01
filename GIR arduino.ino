#include <Servo.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <EEPROM.h>
#include <math.h>

// ====== سروو موتورها ======
Servo servo1;  // دست راست
Servo servo2;  // دست چپ
Servo servo3;  // سر
Servo servo4;  // بدن

int right = 0;
int left = 180;
int head = 90;
int body = 90;

String headState = "CENTER";
String bodyState = "BODY_NONE";

bool servoBusy = false;
bool rightPlayed = false;
bool leftPlayed = false;
bool headPlayed = false;

// ====== DFPlayer Mini ======
SoftwareSerial mySerial(7, 8); // RX, TX
DFRobotDFPlayerMini dfplayer;

// ====== کنترل بدن ======
bool bodyAttached = false;
unsigned long previousMillis = 0;
const int speedDelay = 15;

// ====== چشم‌ها (IR Sensor) ======
const int sensorPin = 2;
const int eyesPin = 12;
int lastSensorState = LOW;
bool eyesOn = false;
unsigned long lastToggle = 0;
const unsigned long debounceMs = 300;
const int EEPROM_ADDR = 0;

// ====== میکروفون ======
const int micPin = 3;
int lastMicState = LOW;
int clapCount = 0;
unsigned long firstClapTime = 0;
const unsigned long clapWindow = 1000;
bool introducing = false;
unsigned long introStartTime = 0;
const unsigned long introDuration = 6000;

// =====================================================
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  if (!dfplayer.begin(mySerial)) {
    Serial.println("DFPlayer not connected!");
    while (true);
  }
  dfplayer.volume(25);

  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);

  servo1.write(right);
  servo2.write(left);
  servo3.write(head);

  pinMode(sensorPin, INPUT);
  pinMode(eyesPin, OUTPUT);
  pinMode(micPin, INPUT);

  byte savedState = EEPROM.read(EEPROM_ADDR);
  eyesOn = (savedState == 1);
  digitalWrite(eyesPin, eyesOn ? HIGH : LOW);

  Serial.println("System Ready...");
}

// =====================================================
void loop() {
  if (introducing) {
    handleIntroAnimation();
    return;
  }

// ----- کنترل چشم‌ها با IR -----
int sensorState = digitalRead(sensorPin);

// تغییر: از LOW به HIGH حذف شد، حالا وقتی سیگنال LOW میشه (دست نزدیکه) چشم‌ها تغییر می‌کنن
if (sensorState == LOW && lastSensorState == HIGH) {
  if (millis() - lastToggle > debounceMs) {
    eyesOn = !eyesOn;
    digitalWrite(eyesPin, eyesOn ? HIGH : LOW);
    EEPROM.write(EEPROM_ADDR, eyesOn ? 1 : 0);
    Serial.print("Eyes state changed to: ");
    Serial.println(eyesOn ? "ON" : "OFF");
    lastToggle = millis();
  }
}
lastSensorState = sensorState;



  if (!eyesOn) return;

  // ----- تشخیص دو بار کف زدن -----
  int micState = digitalRead(micPin);
  if (micState == HIGH && lastMicState == LOW) {
    unsigned long now = millis();
    if (clapCount == 0) {
      clapCount = 1;
      firstClapTime = now;
    } else if (clapCount == 1 && (now - firstClapTime) <= clapWindow) {
      clapCount = 2;
    } else {
      clapCount = 1;
      firstClapTime = now;
    }
  }
  lastMicState = micState;

  if (clapCount == 2) {
    Serial.println("👏 دو بار کف زدن تشخیص داده شد — شروع معرفی");
    dfplayer.play(5);
    introducing = true;
    introStartTime = millis();
    clapCount = 0;
    attachBodyServo();
    return;
  }

  // ----- دستورات سریال -----
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();      // حذف فاصله‌های اضافی
    command.toLowerCase(); // تبدیل همه حروف به کوچک
    if (command.length() == 0) return;

    if (!servoBusy) {
      if (command == "raise your right hand" && !rightPlayed) {
        dfplayer.play(2);
        servoBusy = true;
        smoothMoveEase(servo1, right, 180, 2000);
        rightPlayed = true;
        servoBusy = false;
      } 
      else if (command == "lower your right hand") {
        servoBusy = true;
        smoothMoveEase(servo1, right, 0, 2000);
        rightPlayed = false;
        servoBusy = false;
      }

      else if (command == "raise your left hand" && !leftPlayed) {
        dfplayer.play(1);
        servoBusy = true;
        smoothMoveEase(servo2, left, 0, 2000);
        leftPlayed = true;
        servoBusy = false;
      } 
      else if (command == "lower your left hand") {
        servoBusy = true;
        smoothMoveEase(servo2, left, 180, 2000);
        leftPlayed = false;
        servoBusy = false;
      }

      else if (command == "look to the right" && !headPlayed) {
        dfplayer.play(7);
        headState = "LEFT";
        headPlayed = true;
      } 
      else if (command == "look to the left" && !headPlayed) {
        dfplayer.play(6);
        headState = "RIGHT";
        headPlayed = true;
      } 
      else if (command == "look straight ahead" || command == "NONE") {
        headState = "CENTER";
        headPlayed = false;
      }

      else if (command == "turn to the right") {
        dfplayer.play(4);
        attachBodyServo();
        smoothMoveEase(servo4, body, 0, 2500);
        detachBodyServo();
      } 
      else if (command == "turn to the left") {
        dfplayer.play(3);
        attachBodyServo();
        smoothMoveEase(servo4, body, 180, 2500);
        detachBodyServo();
      } 
      else if (command == "stand straight") {
        attachBodyServo();
        smoothMoveEase(servo4, body, 90, 2500);
        detachBodyServo();
      }

      else if (command == "introduce yourself") {
        dfplayer.play(5);
      }
    }
  }

  // 🧠 کنترل سر
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= speedDelay) {
    previousMillis = currentMillis;
    if (headState == "RIGHT" && head < 180) head++;
    else if (headState == "LEFT" && head > 0) head--;
    else if (headState == "CENTER") {
      if (head < 90) head++;
      else if (head > 90) head--;
    }
    servo3.write(head);
  }
}

// =====================================================
// فعال‌سازی موقت سروو بدن
void attachBodyServo() {
  if (!bodyAttached) {
    servo4.attach(6);
    bodyAttached = true;
  }
}

// آزادسازی سروو بدن
void detachBodyServo() {
  if (bodyAttached) {
    servo4.detach();
    bodyAttached = false;
  }
}

// =====================================================
// حرکت نرم انسان‌مانند
void smoothMoveEase(Servo &servo, int &currentAngle, int targetAngle, int duration_ms) {
  int steps = abs(targetAngle - currentAngle);
  if (steps == 0) return;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float ease = 0.5 - 0.5 * cos(t * PI);
    int angle = currentAngle + (int)(ease * (targetAngle - currentAngle));
    servo.write(angle);
    delay(duration_ms / steps);
  }
  currentAngle = targetAngle;
}

// =====================================================
// ✨ انیمیشن معرفی با حرکت ۱۰ درجه‌ای بدن
void handleIntroAnimation() {
  unsigned long elapsed = millis() - introStartTime;

  // چشمک زدن نرم
  //if ((elapsed / 400) % 2 == 0) digitalWrite(eyesPin, HIGH);
  //else digitalWrite(eyesPin, LOW);

  if (elapsed < 2000) {
    smoothMoveEase(servo4, body, 100, 1000); // ۱۰ درجه به راست
  } 
  else if (elapsed < 4000) {
    smoothMoveEase(servo4, body, 80, 1000);  // ۱۰ درجه به چپ
  } 
  else if (elapsed < 5500) {
    smoothMoveEase(servo4, body, 90, 1000);  // بازگشت به مرکز
  } 
  else if (elapsed >= introDuration) {
    introducing = false;
    //digitalWrite(eyesPin, eyesOn ? HIGH : LOW);
    detachBodyServo(); // بعد از معرفی سروو بدن خاموش
    Serial.println("🎤 معرفی پایان یافت");
  }
}
