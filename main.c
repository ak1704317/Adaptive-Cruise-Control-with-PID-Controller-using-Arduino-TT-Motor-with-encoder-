#include <Ultrasonic.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Motor Pins
const int motorIn1 = 5;
const int motorIn2 = 6;
const int motorEnable = 9;

// Ultrasonic Sensor
const int trigPin = 10;
const int echoPin = 8;
Ultrasonic ultrasonic(trigPin, echoPin);

// LEDs
const int led1 = 13;
const int led2 = 12;

// Buttons
const int btn_inc = A0;
const int btn_dec = A1;
const int btn_cancel = A2;
const int btn_set = A3;
const int btn_acc = A4;

// Encoder
volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
const int pulsesPerRevolution = 20;
float actualRPM = 0;
float smoothedRPM = 0;
const float alpha = 0.3;
unsigned long lastRPMCalcTime = 0;

// System Variables
int speed = 0;
int set_speed = 0;
float distance = 0;
int mode = 0; // 0: Normal, 1: Cruise, 2: ACC
int constant = 0;

// Display timing
bool blinkState = false;
unsigned long previousMillis = 0;
const long blinkInterval = 200;
unsigned long lastDisplayUpdate = 0;
const long displayInterval = 300;

// Debounce flags
bool incFlag = false, decFlag = false, accFlag = false;
bool setFlag = false, cancelFlag = false;

// PID Control
double Setpoint = 0, Input = 0, Output = 0;
double Kp = 1.2, Ki = 0.3, Kd = 0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Encoder ISR with stronger noise filter
void countPulse() {
  unsigned long now = micros();
  if (now - lastPulseTime > 20000) {  // Accept only 1 pulse every 20 ms max
    pulseCount++;
    lastPulseTime = now;
  }
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), countPulse, FALLING);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ACC System Start");
  delay(1000);
  lcd.clear();

  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print((mode == 0) ? "Normal " : (mode == 1) ? "Cruise " : "ACC    ");
  lcd.print("S:");
  lcd.print((int)Setpoint);

  lcd.setCursor(0, 1);
  lcd.print("R:");
  lcd.print((int)actualRPM);
  lcd.print(" P:");
  lcd.print((int)Output);

  Serial.print("Mode: ");
  Serial.print((mode == 0) ? "Normal" : (mode == 1) ? "Cruise" : "ACC");
  Serial.print(" | SetRPM: "); Serial.print(Setpoint);
  Serial.print(" | RPM: "); Serial.print(actualRPM);
  Serial.print(" | PWM: "); Serial.print(Output);
  Serial.print(" | Distance: "); Serial.print(distance, 0);
  Serial.println(" cm");
}

void checkButtons() {
  int incVal = analogRead(btn_inc);
  int decVal = analogRead(btn_dec);
  int cancelVal = analogRead(btn_cancel);
  int setVal = analogRead(btn_set);
  int accVal = analogRead(btn_acc);

  if (incVal > 600 && !incFlag) {
    speed++;
    incFlag = true;
  } else if (incVal < 100) incFlag = false;

  if (decVal > 600 && !decFlag) {
    speed--;
    decFlag = true;
  } else if (decVal < 100) decFlag = false;

  if (setVal > 600 && !setFlag) {
    mode = 1;
    set_speed = speed;
    setFlag = true;
  } else if (setVal < 100) setFlag = false;

  if (accVal > 600 && !accFlag) {
    mode = 2;
    constant = speed;
    accFlag = true;
  } else if (accVal < 100) accFlag = false;

  if (cancelVal > 600 && !cancelFlag) {
    mode = 0;
    cancelFlag = true;
  } else if (cancelVal < 100) cancelFlag = false;
}

void loop() {
  unsigned long currentMillis = millis();
  distance = ultrasonic.read();

  checkButtons();

  // Mode Logic
  if (mode == 0) {
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
    Setpoint = speed;
  } else if (mode == 1) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
    Setpoint = set_speed;
  } else if (mode == 2) {
    if (distance < 20.0) {
      if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;
        blinkState = !blinkState;
        digitalWrite(led1, blinkState);
        digitalWrite(led2, blinkState);
      }
      speed--;
    } else {
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      speed++;
    }
    speed = constrain(speed, 0, constant);
    Setpoint = speed;
  }

  Setpoint = constrain(Setpoint, 0, 135);  // Max safe limit

  // RPM Calculation
  if (currentMillis - lastRPMCalcTime >= 1000) {
    noInterrupts();
    int pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float rawRPM = (pulses * 60.0) / pulsesPerRevolution;
    actualRPM = (alpha * rawRPM) + ((1 - alpha) * smoothedRPM);
    smoothedRPM = actualRPM;

    // Clamp RPM to avoid fake spikes
    actualRPM = constrain(actualRPM, 0, 150);
    if (actualRPM > Setpoint * 2) {
      actualRPM = Setpoint;  // override fake values
    }

    Input = actualRPM;
    myPID.Compute();

    lastRPMCalcTime = currentMillis;
  }

  // Motor Control using PID Output
  if (Output > 0) {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorEnable, (int)Output);
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorEnable, 0);
  }

  // LCD update
  if (currentMillis - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }

  delay(10);
}