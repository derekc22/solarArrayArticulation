# Solar Array Articulation System

A dual-axis solar tracking system using Raspberry Pi, computer vision, and precision motor control to automatically orient solar panels toward the sun for maximum energy efficiency.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Usage](#usage)
- [File Structure](#file-structure)


## Overview

This system uses computer vision to detect the sun's position in real-time and automatically adjusts two solar panels using servo and stepper motors for optimal sun tracking. The system features:

- **Dual-axis tracking**: Pitch (elevation) and yaw (azimuth) control
- **Dual panel support**: Independent control of two solar panel arrays
- **Real-time sun detection**: Computer vision-based sun position tracking
- **Data logging**: Movement and position data collection
- **Safety limits**: Built-in motor angle limits and error handling

## Features

- 🌞 **Automatic Sun Tracking**: Real-time computer vision-based sun detection
- 🔄 **Dual-Axis Control**: Independent pitch and yaw movement for each panel
- 📊 **Data Logging**: CSV logging and plotting of motor movements
- 🛡️ **Safety Systems**: Angle limits and smooth movement control
- 📷 **Visual Feedback**: Live camera feed with sun detection overlay
- ⚡ **Parallel Processing**: Concurrent motor control for smooth operation

## Hardware Requirements

### Core Components
- **Raspberry Pi 4** (recommended) with GPIO access
- **Raspberry Pi Camera Module** (Pi Camera v2 or v3)
- **2x Servo Motors** (for pitch control)
- **2x Stepper Motors** (for yaw control)
- **2x Stepper Motor Drivers** (compatible with PUL/DIR/ENA interface)
- **2x Limit Switches** (for stepper homing)

### Default GPIO Pin Configuration
```
Panel 1:
- Servo: Pin 40
- Stepper PUL: Pin 35
- Stepper DIR: Pin 37
- Stepper ENA: Pin 38
- Limit Switch: Pin 11

Panel 2:
- Servo: Pin 3
- Stepper PUL: Pin 10
- Stepper DIR: Pin 12
- Stepper ENA: Pin 8
- Limit Switch: Pin 13
```

### Additional Hardware
- Power supply suitable for motors
- Mounting hardware for solar panels
- Weatherproof enclosure (for outdoor deployment)

## Software Dependencies

### Python Libraries
```bash
# Core libraries
opencv-python>=4.5.0
numpy>=1.21.0
matplotlib>=3.5.0
picamera2>=0.3.0
RPi.GPIO>=0.7.0
Pillow>=8.0.0

# System libraries
time
math
os
concurrent.futures
```

## Installation

### 1. System Setup
```bash
# Update Raspberry Pi OS
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install python3-pip python3-venv git -y

# Enable camera and GPIO
sudo raspi-config
# Navigate to Interface Options > Camera > Enable
# Navigate to Interface Options > GPIO > Enable
```

### 2. Project Installation
```bash
# Clone the repository
git clone <repository-url>
cd solarArrayArticulation

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install opencv-python numpy matplotlib picamera2 RPi.GPIO Pillow
```

### 3. Directory Setup
```bash
# Create data directory for logging
mkdir -p data
mkdir -p photos
```

## Hardware Setup

### 1. Motor Connections
Connect motors according to the GPIO pin configuration above. Ensure proper power supply connections and grounding.

### 2. Camera Setup
Mount the Raspberry Pi camera with a clear view of the sky. The camera should be positioned to capture the sun's path throughout the day.

### 3. Panel Mounting
Mount solar panels on the motor-controlled articulation system with appropriate mechanical coupling via attachments.

## Usage

### Basic Operation
```bash
# Activate virtual environment
source venv/bin/activate

# Run the main tracking system
python articulate.py
```

### Testing Individual Components

#### Test Camera
```bash
python testcam.py
```

#### Test Servo Motors
```bash
python servo.py
```

#### Test Stepper Motors
```bash
python stepper.py
```

### Control Interface
- **Spacebar**: Stop tracking and return to home position
- **Live feed**: Visual feedback with detected sun position
- **Console output**: Real-time motor position and timing data

## File Structure

```
solarArrayArticulation/
├── README.md                 # This file
├── articulate.py            # Main tracking system
├── servo.py                 # Servo motor control class
├── stepper.py              # Stepper motor control class
├── testcam.py              # Camera testing utility
├── CLINGINIT.py            # Blob detection configuration
├── data/                   # Data logging directory
│   ├── servo1_coords.csv   # Servo 1 movement log
│   ├── servo2_coords.csv   # Servo 2 movement log
│   ├── stepper1_coords.csv # Stepper 1 movement log
│   ├── stepper2_coords.csv # Stepper 2 movement log
│   └── *_coords_plt.pdf    # Generated movement plots
└── photos/                 # Captured images directory
    └── keypointsframe_*.jpg # Timestamped tracking images
```


## Data Logging

The system automatically logs:
- **Motor positions**: Real-time angle data for all motors
- **Timestamps**: Precise timing of all movements
- **Images**: Periodic captures with sun detection overlay
- **Performance plots**: Automatically generated movement visualizations

### Data Files
- `servo1_coords.csv`: Servo 1 time and angle data
- `stepper1_coords.csv`: Stepper 1 time and angle data
- `servo2_coords.csv`: Servo 2 time and angle data
- `stepper2_coords.csv`: Stepper 2 time and angle data
- `*_coords_plt.pdf`: Movement visualization plots

