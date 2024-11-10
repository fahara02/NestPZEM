# NestPzem Library

NestPzem is a C++ library for ESP32 microcontrollers, designed to interface with PZEM power monitoring devices. This library provides a robust solution for managing multiple PZEM devices via a `MeterPool` and supports FreeRTOS-based polling to retrieve data in the background, allowing efficient and asynchronous data handling.

## Features

- **Multi-Device Management**: Support for multiple PZEM devices using a `MeterPool` for easy integration of several power meters.
- **Background Polling**: Utilizes FreeRTOS to poll each device in the background, enabling non-blocking power data collection.
- **Efficient Data Handling**: Supports asynchronous polling and data retrieval, reducing queue buildup on shared UART resources.
- **Configurable Polling Rate**: Customize polling intervals for optimized data collection.

## Requirements

- **ESP32** with the Arduino Framework and ESP-IDF integration
- **FreeRTOS** for background task management
- **PZEM-004T/AC/CT** power monitoring devices

## Installation

Clone the repository or download the zip file and add it to your PlatformIO project under the `lib` folder.

```bash
git clone https://github.com/fahara02/NestPzem.git
```

## UsingProtocolBuffer
- **STEP1** : install protoc in your system
- **STEP2** : define your proto file under proto folder
- **STEP3** : auto generate the header in terminal inside proto folder 
```bash
  protoc --nanopb_opt=-v --nanopb_out=. pData.proto