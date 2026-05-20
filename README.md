# IE410 Robotics Projects

This repository contains the implementation, simulation, reports, and results for the IE410 Introduction to Robotics course projects completed during Winter 2026 at Dhirubhai Ambani University.

---

# Course Information

- **Course:** IE410 – Introduction to Robotics
- **Professor:** Prof. Sujay Kadam
- **Program:** B.Tech ICT
- **Semester:** 4
- **Institute:** Dhirubhai Ambani University
- **Academic Term:** Winter 2026

---

# Team Members

- Hari Patel (202401061)
- Yug Gabani (202401054)
- Kartik Shah (202401084)
- Prince Savaliya (202401191)
- Jay Vaghela (202401237)

---

# Repository Structure

```text
IE410-Robotics-Project/
│
├── Part_A/
│   ├── Code/
│   ├── Images/
│   ├── Report/
│
├── Part_B/
│   ├── MATLAB_Code/
│   ├── Images/
│   ├── Report/
│
└── README.md
```

---

# Part A — Robotic Arm Pick and Place System

## Overview

Part A focuses on robotic arm manipulation tasks using servo-controlled robotic arms and camera-assisted object detection techniques.

The project includes:

- Pre-programmed pick and place operation
- Camera-assisted object localization
- ArUco marker detection
- Multi-arm object handover and synchronization

---

## Task 1 — Pre-Programmed Pick and Place

### Description

A robotic arm was programmed using predefined servo angles to perform pick and place operations.

The arm:
- moved toward object location,
- picked the object,
- transferred it,
- and placed it at a target position.

### Features

- Servo angle-based motion control
- Sequential object manipulation
- Delay-based synchronization
- Gripper actuation

---

## Task 2 — Camera Assisted Pick and Place

### Description

A mobile camera feed using DroidCam and OpenCV was used for object detection and workspace localization.

ArUco markers were used to:
- define workspace boundaries,
- detect object position,
- estimate object coordinates.

### Technologies Used

- Python
- OpenCV
- ArUco marker detection
- DroidCam
- Arduino communication

### Challenges

Although object coordinates were successfully detected, real-time robotic arm motion toward computed coordinates was unstable due to communication and coordinate mapping issues.

The task was later completed using a hybrid approach combining:
- camera-based coordinate detection,
- and pre-programmed robotic arm motion.

---

## Task 3 — Dual Robotic Arm Synchronization

### Description

Two robotic arms were synchronized to perform object handover operations.

The process involved:
1. Robot Arm 1 picking the object
2. Moving to handover position
3. Holding object for synchronization delay
4. Robot Arm 2 grasping the object
5. Robot Arm 1 releasing the object
6. Robot Arm 2 placing the object

### Features

- Multi-arm synchronization
- Delay coordination
- Servo sequence management
- Collaborative robotic manipulation

---

# Part B — Theo Jansen Mechanism Analysis

## Overview

Part B focuses on the kinematic analysis and trajectory generation of a modified Theo Jansen walking mechanism using MATLAB simulation.

The project includes:
- mechanism modeling,
- gait trajectory generation,
- link variation analysis,
- and walking motion visualization.

---

## Objectives

- Study Theo Jansen walking mechanism
- Generate gait-like trajectories
- Analyze linkage kinematics
- Simulate walking motion
- Study effect of varying link dimensions

---

## Features

- MATLAB-based simulation
- Circle-circle intersection kinematics
- Gait trajectory generation
- Animation and visualization
- Link length variation analysis
- Gait cycle analysis

---

## Simulation Outputs

The simulation generated:
- foot trajectories,
- gait cycle plots,
- linkage animations,
- and trajectory comparisons.

---

# Software and Tools Used

## Part A
- Arduino IDE
- Python
- OpenCV
- DroidCam
- Servo motor control

## Part B
- MATLAB
- MATLAB plotting tools
- Numerical kinematic simulation

---

# Reports

The repository contains:
- LaTeX project reports
- Simulation plots
- Experimental screenshots
- Source code implementations

---

# Google Drive Link

Project videos, additional resources, and demonstration files:

https://drive.google.com/drive/folders/1GPEGhqCAhUgXQOfaM5mzqfGGLRC3UM2n

---

# Disclaimer

This repository is intended for academic and educational purposes related to the IE410 Introduction to Robotics course.

---

# License

This project is shared for educational reference and learning purposes.
