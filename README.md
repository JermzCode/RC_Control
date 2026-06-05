The goal of this project is to develop a swarm intelligence system consisting of three autonomous RC cars that can work together to track, surround, and capture a moving target. Each vehicle operates independently while communicating with the other vehicles and a central control hub, allowing the swarm to coordinate its movements and achieve a shared objective with minimal human intervention.

**Hardware Used**
ESP32 microcontrollers (one per RC car and control hub)
Modified RC car chassis
Electronic Speed Controllers (ESCs)
Steering servo motors
OV2640 camera modules
7.4V LiPo battery packs
Laptop / Orin Jetson Nano for AI processing and coordination

**Software Used**
Python for AI development and computer vision
OpenCV for target detection and tracking
C/C++ for embedded control and communication
ESP32 wireless communication framework
Simulation environment for training and testing swarm behavior

**Current Progress**
Controlled an RC car using an ESP32 and PWM signals
Established wireless communication between ESP32 devices
Developed an initial AI model capable of tracking position, direction, and a target
Created a simulation of three vehicles pursuing a moving target
Built and tested individual subsystems for AI, communication, and vehicle control
