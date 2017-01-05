#Mechatronics Projects - Propeller C

This repository contains code from projects in MAE 4710 Mechatronics in the TLP

**Semi-Quadcopter**

Final project which involved controlling a 2 degrees of freedom "semi" quadcopter. [Video Link](https://drive.google.com/file/d/0B3Y5DLBJ2TJWUmxDdUU1ZF9BMjQ/view?usp=sharing)
* Uses Intertial Measurement Unit (IMU) to measure motion and determine orientation
* Implements Kalman filter to provide best estimate of orientation by "fusing" sensor data from gyroscope and accelerometer
* Uses PID to maintain stable orientation
* Uses IR remote instructions to "land" and "take-off"

**Home Entry System**

Early project that introduced students to programming the Propeller microcontroller and using Propeller C and the SimpleIDE. [Video Link](https://drive.google.com/file/d/0B3Y5DLBJ2TJWTDZTUTZ1QXgzOWs/view?usp=sharing)
* Uses a "laser system" (LED and photoresistor) to detect motion
* Plays a song when the system is armed and motion is detected
* Can be disarmed with the correct PIN typed on a IR Remote