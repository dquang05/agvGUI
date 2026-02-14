# AGV GUI — Development & Debugging Tool for ESP-AGV Communication

## Overview
This project is a PC-based GUI development tool designed to:

* **Establish and test** the Wi-Fi communication channel between the ESP module on the AGV and the computer (PC).
* **Monitor and debug** physical parameters such as IMU, LiDAR, and telemetry during the development and mechanical testing phases.

**The primary objectives of this repository are:**
* To verify the data flow from the ESP (via TCP/UDP) before full hardware integration.
* To support indirect debugging of AGV operational parameters, including rotation angles, acceleration, and LiDAR scan distances.

## Key Features
* **Real-time Visualization:** A `tkinter`-based GUI application featuring real-time plotting powered by `matplotlib`.
* **Data Parsing:** Logic to decode and parse IMU and LiDAR packets for immediate analysis.
* **Network Client:** Supports device discovery via UDP beacons and TCP connections for data reception and command transmission.

## Project Status
* This project is currently in the development stage.
* It is primarily built to support the [espAGV](https://github.com/dquang05/espAGV) project and is not intended as a final standalone product.

## Requirements & Quick Start

### 1. Create a Virtual Environment (Recommended)
```bash
python -m venv .venv
```
Activate on Windows:
```bash
.venv\Scripts\activate   
```
# Activate on Linux/macOS:
```bash
source .venv/bin/activate
```
2. Install Dependencies:
```bash
pip install -r requirements.txt
```

3. Run the GUI
To start the main interface:
```bash
python gui.py
```
Or to run the IMU observer specifically:
```bash
python imuObserve.py
```

Operational Notes
-----------------
- When the mechanical part of the AGV is complete, this GUI can be switched to real-time connection mode with the vehicle via Wi-Fi.
- During the development phase, it is possible to emulate beacon/TCP streams from the ESP to test the interface and parser.


