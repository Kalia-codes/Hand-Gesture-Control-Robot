# Raspberry Pi Gesture Controlled Robot

This project uses a Raspberry Pi, OpenCV, and Mediapipe to control a robot's movement via hand gestures detected from a camera. The robot's motors are operated using GPIO pins and PWM signals.

## Features

- **Move Forward:** Show a closed fist (no fingers up) to move forward for 1 meter.
- **Move Backward:** Show index and middle fingers (2 fingers up) to move backward for 1 meter.
- **Turn Right:** Show only your thumb (1 finger up).
- **Turn Left:** Show any 3 fingers up.
- **Stop:** Show 4 or 5 fingers up.

## Hardware Requirements

- Raspberry Pi (with GPIO support)
- Motor Driver (connected to GPIO pins)
- USB Camera
- Motors for robot movement

## Software Requirements

- Python 3
- OpenCV (`cv2`)
- Mediapipe
- RPi.GPIO

## Installation

1. **Install dependencies:**
    ```bash
    pip install opencv-python mediapipe RPi.GPIO
    ```

2. **Connect your hardware:**
    - Make sure your motors are connected to the following GPIO pins:
      - Left Motor Forward: GPIO 17
      - Left Motor Backward: GPIO 18
      - Right Motor Forward: GPIO 22
      - Right Motor Backward: GPIO 23
    - Connect your camera to the Raspberry Pi.

3. **Clone or copy the code into `main.py`.**

## Usage

Run the script on your Raspberry Pi:
```bash
python main.py
```

A window named "Gesture Control" will open, showing the camera feed and detected hand landmarks. Use the following gestures in front of the camera to control the robot:

| Gesture             | Action         |
|---------------------|---------------|
| Fist (0 fingers up) | Move Forward  |
| Index + Middle      | Move Backward |
| Thumb only          | Turn Right    |
| Any 3 fingers up    | Turn Left     |
| 4 or 5 fingers up   | Stop          |

**Press `'q'` to quit the program.**

## How It Works

- The program uses OpenCV to capture video frames from the camera.
- Mediapipe detects hand landmarks and counts the number of fingers held up.
- Based on the gesture, the corresponding movement function is called, controlling the motors via PWM signals on the specified GPIO pins.
- Forward and backward movements are time-limited to approximate 1 meter.

## Code Structure Overview

- **GPIO & PWM Setup:** Initializes the pins and PWM channels for motor control.
- **Movement Functions:** Handles the logic for moving forward, backward, turning, and stopping.
- **Gesture Detection:** Uses Mediapipe to count fingers and map gestures to robot actions.
- **Main Loop:** Captures frames, processes gestures, and controls motors accordingly.

## Safety & Precautions

- Ensure your robot is on a safe surface before running the program.
- Disconnect power before making hardware modifications.


---

**Happy Robot Building!**
