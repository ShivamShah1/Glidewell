# STM32 FreeRTOS Wearable Sensor Logger with BLE Control

This project is a FreeRTOS-based embedded firmware designed for an STM32x microcontroller board. It interfaces with two sensors—an **IMU** and a **PPG** via I2C —collects their data periodically, logs it into flash memory, and communicates with a mobile application via **STM's built-in BLE stack**. The user can control the device and interact with the logged data using BLE commands.

---

## 📦 Features

- 🔄 **FreeRTOS**-(priority) based task scheduling
- ⏱️ **IMU Sensor** data acquisition at **50 Hz**
- ❤️ **PPG Sensor** data acquisition at **100 Hz**
- 💾 **Flash memory logging** of synchronized sensor data
- 📡 **BLE connectivity** using STM's internal BLE stack
- 📲 **Mobile app control** with 4 supported commands:
  - `START_LOGGING` – Begin storing sensor data to flash
  - `STOP_LOGGING` – Stop logging sensor data
  - `BROWSE_LOGS` – View summary or number of logged entries
  - `DOWNLOAD_LOG` – Download complete logged data
- 🔒 **Watchdog timer** set to 1 second to ensure system reliability and auto-reset on faults or task hangs

---

## 🧠 System Architecture
## ** Diagram **
![image](https://github.com/user-attachments/assets/a4cea97e-8550-48b2-89a5-a0d27766a4d4)

## **Work Flow**
![image](https://github.com/user-attachments/assets/a84b7102-ef52-4565-a056-d3451d2765fb)



## System Design Decisions and Trade-offs

### Use of STM's BLE library
- Decision: to properly initialize and take care of request automaically.
- Trade-off: this will create overheads and can create little latency.

### Basic BLE implementation
- Decision: Use for basic communication.
- Trade-off: may not be sufficient for complex or flexible data communications.

### Memory Alignment
- Decision: Memroy alignment for faster and predictive time access.
- Trade-off: more memory space usage.

### Error Handling
-  Decision: Implemented basic error handling functionality.
-  Trade-off: this makes the code easier to manage and restarts the system when error occurs, but not a good option to reset the system for every error.

### Memory logging
- Decision: To log the data to flash single time to save clock cycle and increase CPU efficiency.
- Trade-off: Logs data when all 3 data is available, if any 1 data is not available then will not log to flash.

### CMSIS library
- Decision: Helps in writing the FreeRTOS project.
- Trade-off: consumes more memory.
