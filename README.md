# M5 Unit Cam S3 5MP Microphone WAV Recorder

This project provides an Arduino IDE example for recording audio from the **built-in microphone** of the [M5 Unit Cam S3 5MP](https://shop.m5stack.com/products/unit-cam-s3-5mp) camera module, saving it as a `.wav` file on an SD card.

> **Note:**  
> This program is developed and tested specifically for the **M5 Unit Cam S3 5MP** module.  
> It is **likely to also work on the M5 Unit Cam S3 (2MP)** model if the PDM microphone and SD card pin assignments are the same, thanks to automatic detection of PDM slot and clock polarity.  
> However, the 2MP version has not been officially tested.

## Features

- **DC offset removal** for cleaner recordings
- **Fixed gain mode** with a safety limiter
- **Automatic gain control (AGC) mode**
- **Configurable sample rate, gain, and block size**
- **Automatic file naming** (`REC0001.WAV`, `REC0002.WAV`, ...)
- **Drop-head function** to skip startup noise

## Hardware Requirements

- **M5 Unit Cam S3 5MP** (tested)  
- **MicroSD card** (formatted as FAT32)  
- Arduino IDE with ESP32 board support installed

> For the **M5 Unit Cam S3 (2MP)**:  
> - The program should work if the PDM microphone pins (`CLK=47`, `DIN=48`) and SD card pins (`CLK=39`, `MISO=40`, `MOSI=38`, `CS=9`) are identical.  
> - If the hardware uses different pins, update `mic_pins.h` and `sdcard_pins.h` accordingly.

## Pin Configuration (5MP model)

| Function   | Pin |
|------------|-----|
| PDM CLK    | 47  |
| PDM DIN    | 48  |
| SD CLK     | 39  |
| SD MISO    | 40  |
| SD MOSI    | 38  |
| SD CS      | 9   |

These are defined in `mic_pins.h` and `sdcard_pins.h`.

## Installation

1. Install the [ESP32 board support](https://github.com/espressif/arduino-esp32) in Arduino IDE.
2. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/m5-unit-cam-s3-5mp-wav-recorder.git
   ```
3. Open `WavRecorder_5MP.ino` in Arduino IDE.
4. Select **ESP32S3 Dev Module** as the board.
5. Connect your M5 Unit Cam S3 5MP via USB.
6. Compile and upload the sketch.

## Usage

### Basic Recording (Auto Gain Control)
```cpp
micInit();  // Initialize PDM microphone
recordingAuto(5);  // Record 5 seconds with AGC
```

### Fixed Gain Recording
```cpp
FixedGainConfig gain;
gain.gainDb = 20.0f; // 20 dB gain
recordingFixedEx(5, nullptr, &gain, nullptr, nullptr);
```

The recordings will be saved in `/audio` on the SD card.

## File Structure

```
/
├── src/
│   ├── mic.cpp
│   ├── mic.h
│   ├── mic_pins.h
│   ├── sdcard_pins.h
├── examples/
│   └── WavRecorder_5MP/
│       └── WavRecorder_5MP.ino
├── README.md
└── .gitignore
```

- **src/** — Core recording library
- **examples/** — Example sketch for Arduino IDE

## License

This project is released under the MIT License.  
See [LICENSE](LICENSE) for details.
