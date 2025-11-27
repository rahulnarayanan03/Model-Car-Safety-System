# Arduino-Based Car Safety System

An *Arduino-powered car safety prototype* designed to detect obstacles in real time using *ultrasonic sensors* and provide automatic alerts to improve driver safety.  
This project demonstrates concepts in *embedded systems, **sensor fusion, and **control logic design*, forming the foundation for intelligent vehicle safety applications.

---

## Overview

This project was developed as part of my Mechatronics Engineering coursework at the *University of Technology Sydney, earning a **High Distinction*.  
The system integrates multiple sensors and feedback mechanisms to detect nearby obstacles and alert the driver through both visual and auditory signals.

---

## ⚙ Features

- 🔍 *Real-time obstacle detection* using ultrasonic distance sensors  
- 💡 *Dynamic alert system* with LEDs and buzzer for hazard indication  
- 🔄 *Control logic* implemented in Arduino for decision-making and response control  
- 🧩 *Modular design* for easy expansion (e.g. braking automation or IoT integration)  
- ⚙ *Efficient signal processing* to ensure quick and reliable detection

---

## 🛠 Components Used

| Component | Description |
|------------|-------------|
| *Arduino Mega* | Main microcontroller for system control |
| *HC-SR04 Ultrasonic Sensors* | Distance measurement for obstacle detection |
| *Buzzer Module* | Audible warning output |
| *LED Indicators* | Visual proximity alerts |
| *Breadboard & Jumper Wires* | Circuit prototyping and connections |
| *Power Supply (5V)* | Power source for sensors and controller |
| *LCD Display* | To show PIN, selected gear, and distance warnings |
| *Potentiometer* | To input the pin digits to start the car |

---

## 🧩 System Architecture

[Ultrasonic Sensors] → [Arduino Microcontroller] → [Control Logic] → [LED/Buzzer Alerts]

- The *sensors* continuously measure distances to obstacles.  
- The *Arduino* processes these readings and applies threshold-based logic.  
- The *output* triggers LED or buzzer signals based on proximity severity.
