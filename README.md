# Autonomous Vehicle Planning & Tracking

[![MATLAB](https://img.shields.io/badge/MATLAB-R202x-blue?logo=mathworks)](https://www.mathworks.com/)
[![Simulink](https://img.shields.io/badge/Simulink-Enabled-orange?logo=mathworks)](https://www.mathworks.com/products/simulink.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A complete MATLAB/Simulink pipeline for **path planning**, **trajectory generation**, **feedback linearization control**, and **state estimation** of a nonholonomic car-like robot. 

Designed to demonstrate a full autonomous navigation stack, from high-level graph-based planning to low-level continuous-time control and robust localization.

This repository implements:
- Graph-based planning using motion primitives inspired by Reeds-Shepp curves
- Shortest-path search using Dijkstra
- Trajectory smoothing and velocity profiling
- Input/Output Feedback Linearization
- Odometric localization (Euler and RK2)
- Extended Kalman Filter (EKF)

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Project Structure](#project-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [How to Run](#how-to-run)
- [Simulation Modes](#simulation-modes)
- [Demo](#demo)
- [Results](#results)
- [Authors](#authors)

---

## Overview

This project solves autonomous navigation for a car-like robot over a map extracted from Google Maps.

Pipeline:

`Map → Graph Generation → Path Search → Trajectory Generation → Controller → Localization → Analysis`

---

## System Architecture

### Path Planning
- Straight, left-turn, right-turn primitives
- Occupancy grid collision checking
- Iterative graph expansion

### Path Optimization
- Dijkstra shortest path

### Trajectory Generation
- Arc-length parametrization
- Curvature estimation
- Adaptive velocity profiling
- Gaussian smoothing
- Uniform scaling

### Control
- Virtual point mapping
- I/O feedback linearization
- Decoupled tracking
- Goal damping logic

### Localization
- Ideal feedback
- Euler odometry
- RK2 odometry
- EKF with landmarks

---

## Project Structure

```text
.
├── main.m
├── CarLike_Robot.slx
├── images/
└── README.md
```

---

## Requirements

- MATLAB
- Simulink
- Image Processing Toolbox

---

## Installation

```bash
git clone https://github.com/Nicholasr567/TechnicalProject_FSR.git
cd TechnicalProject_FSR
```

---

## How to Run

1. Run `main.m`
2. Select start and goal on the map
3. Generate graph and path
4. Open `CarLike_Robot.slx`
5. Select localization mode
6. Run simulation

---

## Simulation Modes

| Mode | Description |
|---|---|
| Nominal | Ideal state feedback |
| Euler | Forward Euler odometry |
| RK2 | Second-order odometry |
| EKF | Landmark-based localization |

---

## Demo
<p align="center"> <a href="docs/full_simulation.mp4"> <img src="docs/demo.gif" alt="Simulation Demo" width="700"/> </a> </p>

<p align="center"> Click the preview to watch the full simulation. </p>


---

## Results

The framework automatically generates:

- State tracking errors
- Control signals
- Euclidean tracking error
- Trajectory comparison

Saved inside:

```text
images/<method>/
```

---

## Authors

Giuseppe Arena & Nicholas Ruggiero
