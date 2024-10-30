# Micromouse Robot Navigation Project

## Description
This project involves building a micromouse robot that navigates through a maze using a flood fill algorithm. The robot utilizes IR sensors and a sonar sensor to detect walls and make decisions to reach its goal efficiently.

## Table of Contents
1. [Components Used](#components-used)
2. [Setup Instructions](#setup-instructions)
3. [Code Explanation](#code-explanation)
4. [Usage](#usage)
5. [Future Improvements](#future-improvements)
6. [Acknowledgments](#acknowledgments)

## Components Used
- Arduino Uno R3
- IR Sensors (2)
- Sonar Sensor (HC-SR04)
- DC Motors (2)
- Motor Driver Module (L298N)
- Chassis for the robot
- Power Supply (Battery)

## Setup Instructions
1. Assemble the robot chassis and attach the motors.
2. Connect the IR sensors to pins 2 and 3.
3. Connect the sonar sensor to pins 4 and 5.
4. Connect the motor driver to the Arduino and the motors.
5. Upload the code to the Arduino Uno using the Arduino IDE.

## Code Explanation
The code utilizes the flood fill algorithm to navigate through the maze. It reads inputs from the IR and sonar sensors to detect walls and updates the maze representation accordingly. The robot moves based on the calculated shortest path to the goal.

- [Flood Fill Algorithm Video](https://youtu.be/Zwh-QNlsurI?si=kglIv5pRiBY5NRx-) - An informative video explaining the Flood Fill algorithm.

## Usage
Once the robot is assembled and the code is uploaded, place it at the starting point of the maze. Ensure the maze layout is correct and free of obstacles. The robot will begin navigating to the goal automatically.

## Future Improvements
- Implement advanced pathfinding algorithms (like A* or Dijkstra's).
- Add additional sensors for better wall detection.
- Improve motor control for smoother navigation.

## Acknowledgments
Special thanks to the following resources for their guidance in robotics and Arduino programming:
- [Micromouse Online](https://micromouseonline.com/) - A valuable resource for micromouse enthusiasts.
- [Veritasium YouTube Video](https://www.youtube.com/watch?app=desktop&v=ZMQbHMgK2rw&ab_channel=Veritasium) - A great introduction to the principles of robotics and automation.
