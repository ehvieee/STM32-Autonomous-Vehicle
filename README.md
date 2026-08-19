# Project Overview
STM32 implementation of an Autonomous Vehicle System. Uses a Nucleo-L476RG development board for control and logic. Vehicle maneuvers with wheels attached to two DC Motors driven by L298N motor driver module. Object detection done through HC-SR04 Ultrasonic Sensor mounted on a SG90 Servo Motor. The entire system is powered via 2x 3.7 Volt batteries.
<p align="center">
  <img width="512" height="512" alt="image" src="https://github.com/user-attachments/assets/3ec753f2-90c9-4072-b6d3-236108341346" />
</p>

# Hardware List
- Autonomous Vehicle Chassis and Wheel Kit
- 2x DC Motors
- L298N Motor Driver Module
- HC-SR04 Ultrasonic Sensor
- SG90 Servo Motor
- 2x 3.7 Volt Batteries
- Breadboard
- 1x 1 kOhm Resistor
- 1x 2 kOhm Resistor

# Hardware Diagram
<img width="1702" height="1160" alt="image" src="https://github.com/user-attachments/assets/afbc65b5-5975-496c-8a95-941285d0a895" />

# Software Control Logic
<table>
  <tr>
    <p align="center">
      <img width="400" height="550" alt="image" src="https://github.com/user-attachments/assets/06ae1468-76ab-47fd-9ddf-bbe5dacd3052" />
      <img width="400" height="550" alt="image" src="https://github.com/user-attachments/assets/71de0fe9-e8ab-4b94-a67a-dc2b6f630e02" />
    </p>
  </tr>
</table>

The default move state of our vehicle is to continuously move forwards until encountering an obstacle. Once the ultrasonic sensor detects an object within a variable distance, the servo motor will first pivot 90 degrees counterclockwise and the sensor will scan for objects within that same distance. If the path is clear, then the vehicle will make a 90 degrees counterclockwise turn before proceeding forwards. Otherwise, if the path is blocked, the servo motor pivots in the opposite direction so that the sensor can scan the right side of the vehicle. If the path is clear, then the vehicle will make a 90 degrees clockwise turn before proceeding forwards. If both sides of the vehicle are blocked, then the vehicle will begin backing up a variable distance, before repeating the left to right scan. It will repeatedly reverse itself until there is a clear path on either side of the vehicle. The vehicle will turn itself in the direction of the clear path, before proceeding with its default forwards movement, which repeats this process all over again. 
