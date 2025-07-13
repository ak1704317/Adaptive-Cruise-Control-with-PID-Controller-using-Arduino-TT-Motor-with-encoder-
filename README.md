# Adaptive-Cruise-Control-with-PID-Controller-using-Arduino-TT-Motor-with-encoder-


This project implements a real-time **Adaptive Cruise Control** system for small robotic vehicles using:

- **PID controller** for motor speed regulation  
- **Rotary encoder** for accurate RPM measurement  
- **Ultrasonic sensor** for distance-based cruise adaptation  
- **LCD** for real-time display  
- **Five push buttons** to manually control speed and mode  

---

##  Components Used

| Component               | Description                             |
|------------------------|-----------------------------------------|
| Arduino Uno            | Main controller                         |
| DC Motor with Encoder  | 20 pulses/revolution                    |
| L298N Motor Driver     | For motor direction and speed control   |
| Ultrasonic Sensor (HC-SR04) | For object distance detection     |
| I2C LCD 16x2           | For display of RPM, PWM, Mode, Speed    |
| Push Buttons (x5)      | Speed +, Speed –, Set, Cancel, ACC mode |
| LEDs (2)               | Mode indication (Normal, Cruise, ACC)   |
| Power Supply           | External power for motor (recommended)  |

---

##  Pin Configuration

| Function            | Arduino Pin |
|---------------------|-------------|
| Encoder Signal A    | D2 (INT0)   |
| Motor IN1           | D5          |
| Motor IN2           | D6          |
| Motor Enable (PWM)  | D9          |
| Trig (Ultrasonic)   | D10         |
| Echo (Ultrasonic)   | D8          |
| I2C SDA             | A4          |
| I2C SCL             | A5          |
| LED1                | D13         |
| LED2                | D12         |
| Button Increase     | A0          |
| Button Decrease     | A1          |
| Button Cancel       | A2          |
| Button Set          | A3          |
| Button ACC          | A4          |

---

## Operating Modes

### 🔹 Normal Mode (Mode = 0)
- Activated by default or via **Cancel button**
- Speed manually controlled with **Increase/Decrease buttons**
- LED1 = OFF, LED2 = ON

### 🔹 Cruise Mode (Mode = 1)
- Activated by **Set button**
- Locks speed at time of activation
- Motor maintains target speed via PID
- LED1 = ON, LED2 = OFF

### 🔹 Adaptive Cruise Control (Mode = 2)
- Activated via **ACC button**
- If obstacle < 20 cm, slows down
- If clear, resumes cruise speed
- Both LEDs ON; blink if obstacle too close

---

##  Features & Logic

### ✔ PID Motor Control
- Uses `PID_v1.h` library
- Dynamically adjusts PWM to match set RPM

### ✔ Encoder RPM Calculation
- Interrupt-based on pin D2
- Smoothed RPM using low-pass filter
- Filters out noise & spikes

### ✔ LCD Display Output
Displays:

---

## 📈 PID Tuning Parameters

| Parameter | Value |
|-----------|-------|
| Kp        | 1.2   |
| Ki        | 0.3   |
| Kd        | 0.05  |

> You can fine-tune these for different motors or load responses.

---

## Libraries Used

- [`PID_v1.h`](https://playground.arduino.cc/Code/PIDLibrary/)
- [`Ultrasonic.h`](https://github.com/ErickSimoes/Ultrasonic)
- [`LiquidCrystal_I2C.h`](https://github.com/johnrickman/LiquidCrystal_I2C)

Install them via **Library Manager** in Arduino IDE.

---

##  How to Run

1. Connect components as per pin mapping.
2. Open the `.ino` code in Arduino IDE.
3. Select **Board**: Arduino Uno.
4. Select correct **COM Port**.
5. Click **Upload**.
6. Open **Serial Monitor @ 9600 baud** to see runtime values.

---

##  Tips

- Ensure motor has external power.
- Use pull-down resistors if button input is floating.
- Encoder should produce clean pulses — add hardware debounce if needed.
- Adjust `Setpoint` max value (135 RPM default) based on motor capabilities.

---

##  Output Sample

```text
Mode: ACC | SetRPM: 45 | RPM: 44.2 | PWM: 124 | Distance: 18 cm
Mode: Normal | SetRPM: 35 | RPM: 36.0 | PWM: 110 | Distance: 45 cm
