# ESP32 CYD OBD2 Bluetooth Gauge

This project turns an ESP32 Cheap Yellow Display (CYD) 2.8" board into a real-time automotive OBD2 dashboard. It connects via Bluetooth to a standard ELM327 OBD2 adapter and displays critical engine data for your vehicle (designed with a 2011 Ritz Diesel in mind).

## Hardware Requirements
- **ESP32 CYD (ESP32-2432S028R)**: A 2.8" TFT display module with built-in ESP32.
- **ELM327 Bluetooth Adapter**: Ensure it supports "Bluetooth Classic" (not BLE-only).

## Features
- Clean, flicker-free UI with dark mode styling.
- Displays Engine RPM, Vehicle Speed (KPH), Coolant Temperature, and Intake Air Temperature.
- Auto-reconnects to the Bluetooth OBD2 adapter.

## Setup Instructions

### 1. Configure Bluetooth
By default, the code looks for a Bluetooth adapter named `"OBDII"`.
If your adapter has a different name (or you want to use its MAC address for a faster/more reliable connection), edit `src/main.cpp`:
```cpp
const char* elmName = "OBDII"; // Change this if needed
```

### 2. Flashing the Code
This project uses **PlatformIO**.
1. Open this folder (`d:\obd`) in VSCode with the PlatformIO extension installed.
2. Connect your ESP32 CYD to your PC via USB.
3. Click the **Upload** arrow at the bottom of VSCode to build and flash the firmware.

### 3. Usage
1. Plug the ELM327 adapter into your car's OBD2 port.
2. Turn the car ignition to ON or start the engine.
3. Power on the ESP32 CYD.
4. The screen will display "Connecting..." and then show live data once connected to the ELM327.

### Troubleshooting
- **Cannot connect to OBD scanner:** The ESP32 is failing to pair via Bluetooth. Double-check the adapter name (`elmName`). Make sure your phone isn't already connected to the OBD scanner, as it only accepts one connection at a time.
- **Couldn't initialize ELM327:** The Bluetooth connected, but the ELM327 is not responding to standard AT initialization commands. This usually means the adapter is faulty, or the car ignition is off.
- **Values stay at 0:** The adapter might not support the specific PIDs requested, or the engine is not running.
