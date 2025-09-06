# Hand_Eye_Calibration
Master thesis dissertation project. Does an Hand Eye Calibration between a robotic arm and a camera mounted on the robot.

# Hand-Eye Calibration with OPC UA and OpenCV

This project implements **hand-eye calibration** between a robot and a camera using:
- **OpenCV** for camera calibration & hand-eye calibration
- **OPC UA** for communication with a PLC/robot
- **Python (NumPy, SciPy)** for math & transformations

---

## Features
- Collects robot poses via **OPC UA**
- Performs **camera intrinsic calibration** using a chessboard grid
- Computes **hand-eye calibration** with the **Tsai method**
- Sends calibration results back to the PLC
- Calculates and prints reprojection error

---

## Requirements
- Python 3.8+
- Libraries:
  ```bash
  pip install numpy opencv-python scipy opcua
