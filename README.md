# M5 Unit Cam S3 / M5 Unit Cam S3 5MP Microphone WAV Recorder

This project provides an Arduino IDE example for recording audio from the **built-in microphone** of both the **M5 Unit Cam S3 5MP** and the **M5 Unit Cam S3 (2MP)** camera modules, saving it as `.wav` files on an SD card.

- ✅ **Tested and working on M5 Unit Cam S3 5MP**
- ✅ **Tested and working on M5 Unit Cam S3 (2MP)**

The PDM microphone and SD card pin mappings are compatible across these modules, and the code auto-detects the PDM LEFT/RIGHT slot and clock polarity, so it runs on either model without source changes.

## Features

- **DC offset removal** for cleaner recordings
- **Fixed gain mode** with a safety limiter
- **Automatic gain control (AGC) mode**
- **Auto-detection of PDM slot and clock polarity**
- **Configurable sample rate, gain, and block size**
- **Automatic file naming** (`REC0001.WAV`, `REC0002.WAV`, ...)
- **Drop-head function** to skip startup noise
- **WAV header written with correct sizes** at the end of recording

_Defaults_: 16 kHz, 16‑bit PCM, mono, `/audio` directory, 1024‑sample I/O blocks.

## Hardware Requirements

- **M5 Unit Cam S3 5MP** or **M5 Unit Cam S3 (2MP)**
- **MicroSD card** (FAT32)
- Arduino IDE with ESP32 board support installed

## Pin Configuration (both models)

| Function | Pin |
|---|---|
| PDM CLK | 47 |
| PDM DIN | 48 |
| SD CLK  | 39 |
| SD MISO | 40 |
| SD MOSI | 38 |
| SD CS   | 9  |

Pins are defined in `mic_pins.h` and `sdcard_pins.h` and can be adjusted if your hardware differs.

## Installation

1. Install the [ESP32 board support](https://github.com/espressif/arduino-esp32) in Arduino IDE.
2. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/m5-unit-cam-s3-wav-recorder.git
   ```
3. Open one of the examples in Arduino IDE:
   - `examples/WavRecorder/WavRecorder.ino` (generic example for both models)
   - `examples/WavRecorder_5MP/WavRecorder_5MP.ino` (legacy example kept for 5MP)
4. Select **ESP32S3 Dev Module** as the board.
5. Connect your device via USB.
6. Compile and upload the sketch.

## Usage

### Basic Recording (Auto Gain Control)
```cpp
micInit();            // Initialize PDM microphone
recordingAuto(5);     // Record 5 seconds with AGC
```

### Fixed Gain Recording
```cpp
FixedGainConfig gain;
gain.gainDb = 20.0f;  // 20 dB gain
recordingFixedEx(5, nullptr, &gain, nullptr, nullptr);
```

Recordings are saved under `/audio` on the SD card with sequential file names.

## Repository Structure

```
/
├── src/
│   ├── mic.cpp            # updated implementation (filenames unchanged)
│   ├── mic.h
│   ├── mic_pins.h
│   ├── sdcard_pins.h
├── examples/
│   ├── WavRecorder/
│   │   └── WavRecorder.ino
│   └── WavRecorder_5MP/
│       └── WavRecorder_5MP.ino
├── README.md
└── .gitignore
```

> **Note:** The `src/` sources have been **updated** while keeping the same filenames for compatibility.

## License

This project is released under the MIT License.  
See [LICENSE](LICENSE) for details.
