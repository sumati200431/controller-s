# Ball and Beam Control System 🎯

This project implements a control system for the **Ball and Beam setup**, where a DC motor is used to control the tilt of the beam to balance a ball at a desired position. The system is simulated in MATLAB and also implemented on hardware using an Arduino Uno, motor driver, and ultrasonic sensor.


## 🔧 Project Overview

- **System:** Ball and Beam
- **Actuator:** DC Motor (controlled via Arduino)
- **Sensor:** Ultrasonic sensor (for ball position)
- **Control Strategy:** State-space or PID controller
- **Objective:** Keep the beam level and position the ball at a reference point



## 📁 Project Files

| File | Description |
|------|-------------|
| `ball_beam_dynamics.m` | MATLAB model of the ball and beam physics |
| `controller_design.m` | MATLAB script to design and tune controller (LQR/PID) |
| `simulation.m` | Runs the complete simulation and plots results |
| `arduino_code.ino` | Arduino sketch to read sensor and control motor |
| `README.md` | Project explanation and usage |
| `plot_outputs/` | Folder to store MATLAB-generated plots |


## 🛠 Hardware Setup

- **Arduino Uno**
- **Motor driver (L298N or similar)**
- **DC gear motor**
- **Ultrasonic sensor (HC-SR04)**
- **Ball and beam structure (custom or lab setup)**


## 🚀 How to Run Simulation in MATLAB

1. Open MATLAB and navigate to this project folder.
2. Run the simulation:
   ```matlab
   run('simulation.m')

3. You will see plots showing:

Ball position vs time

Beam angle

Control input (motor voltage/speed)

🔌 How to Run on Hardware (Arduino)
1. Upload arduino_code.ino to your Arduino Uno.

2. Connect:

     Ultrasonic sensor to digital pins (e.g., Trig = D7, Echo = D6)

     Motor to motor driver (IN1, IN2, ENA)

3. Use MATLAB to send commands to Arduino (optional, via serial)

4. Observe the real-time balancing of the ball on the beam.

✅ Requirements
MATLAB :
MATLAB R2021a or later

Control System Toolbox

(Optional) Arduino Support Package


Arduino :
Arduino IDE or MATLAB Arduino Toolbox

L298N Motor Driver

HC-SR04 Ultrasonic Sensor


👤 Author
Sumati Chougule
Control Systems Internship — IIT Dharwad

📄 License
MIT License — Feel free to use, modify, and share.

