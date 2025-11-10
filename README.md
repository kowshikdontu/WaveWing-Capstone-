# WaveWing-Capstone-
# 🛰️ WaveWing - wave your hands to control wings

**WaveWing** is an interactive system that allows users to control a drone using **hand gestures**.  
It bridges **MediaPipe-based gesture recognition (Windows)** and **DroneKit-based drone control (Ubuntu)** via a **WebSocket communication layer**.

---

## 🎯 Project Overview

**Workflow:**
```
Camera → Gesture Detection (MediaPipe + CNN)
       → WebSocket Client (Windows)
       → Command Queue & Ack Buffer
       → WebSocket Server (Ubuntu)
       → DroneKit (MAVLink API)
       → ArduPilot SITL → Gazebo Simulation
```

Our system recognizes real-time gestures through a webcam and translates them into drone commands like  
**TAKEOFF**, **LAND**, **FORWARD**, **LEFT**, **RIGHT**, **UP**, **DOWN**, and **YAW ROTATION**.

---

## ⚙️ Components

### 🖐 Gesture Detection (Windows Frontend)
- Built with **MediaPipe** and **OpenCV** for 3D hand-landmark tracking.  
- A **5-layer CNN classifier** trained on our custom dataset interprets keypoint coordinates into labeled gestures.  
- Real-time FPS optimized at ~30 fps.  
- Outputs a JSON payload to WebSocket:
  ```json
  {"gesture": "TAKEOFF"}
  ```

### 🌐 WebSocket Bridge
- Uses Python’s `asyncio` + `websockets` for low-latency data transmission.  
- Handles acknowledgments, buffering, and command queuing to ensure smooth control.  
- Enables seamless **Windows → Ubuntu** communication.

### 🚁 Drone Control (Ubuntu Backend)
- Powered by **DroneKit Python** connected to **ArduPilot SITL** or a physical flight controller.  
- Commands interpreted through **MAVLink protocol**.  
- Simulated in **Gazebo 7** using **Iris Gimbal** model and custom runway world.  
- Each received gesture triggers precise drone actions with velocity-vector control.

---

## 🧠 Model Training

1. **Data Collection:**  
   - Capture hand-landmark coordinates from MediaPipe.  
   - Normalize and label data (e.g., `Takeoff`, `Land`, `Hover`).

2. **Training:**  
   - Run `keypoint_training.ipynb` to train the CNN classifier.  
   - Architecture: Dense-ReLU layers → Softmax output.  
   - Framework: TensorFlow 2.x.

3. **Deployment:**  
   - Export model as `.tflite` for lightweight inference.  
   - Integrated with `gesture_app.py` for real-time prediction.

---

## 🧩 System Requirements

| Component | Version |
|------------|----------|
| Python | ≥ 3.8 |
| MediaPipe | ≥ 0.10 |
| OpenCV | ≥ 4.5 |
| TensorFlow | ≥ 2.8 |
| DroneKit | Latest |
| ArduPilot SITL | 4.3 + |
| Gazebo | ≥ 7 |

---

## ▶️ Running the Project

### 1️⃣ Launch Gesture Detection (Windows)
```bash
python gesture_app.py
```
- Opens webcam stream and starts sending gestures via WebSocket.

### 2️⃣ Start WebSocket Server (Ubuntu)
```bash
python server_ws.py
```
- Connects to the drone through DroneKit (`127.0.0.1:14550`).  
- Receives gesture commands and converts them into movement instructions.

### 3️⃣ Run Drone Simulation
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```
- Launches ArduPilot SITL in Gazebo environment.

---

## 🔢 Repository Structure
```
│  gesture_app.py
│  server_ws.py
│  keypoint_training.ipynb
│
├─ model/
│   ├─ keypoint_classifier/
│   │   ├─ keypoint_classifier.tflite
│   │   ├─ keypoint_classifier_label.csv
│   │   └─ keypoint.csv
│   └─ gesture_logs/
│       └─ collected_datasets/
│
└─ utils/
    ├─ cvfpscalc.py
    └─ command_buffer.py
```

---

## System Architecture
<img width="774" height="434" alt="image" src="https://github.com/user-attachments/assets/57d9c325-d2bd-4597-8222-9221de2a97c2" />

<img width="997" height="586" alt="image" src="https://github.com/user-attachments/assets/a9ef5d09-c536-4862-838b-d551d905e81c" />
<img width="646" height="585" alt="image" src="https://github.com/user-attachments/assets/b5b0785f-8645-4079-93cb-3dfb9918c1da" />



---

## 📊 Tech Stack
- **Python** (OpenCV, TensorFlow, DroneKit)
- **MediaPipe** for hand landmark detection
- **WebSocket (asyncio + websockets)** for real-time messaging
- **Gazebo 7 + ArduPilot SITL** for simulation
- **MAVLink Protocol** for flight communication


---

## 🔖 Credits & Acknowledgements
This project was **inspired by the open-source work** [*hand-gesture-recognition-using-mediapipe*](https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe) by **Kazuhito Takahashi**.  
We extended and adapted that foundational gesture model into a **complete drone control interface** integrating WebSocket communication, DroneKit API, and ArduPilot Gazebo simulation.

---

## 📄 License
This project is released under the **Apache License 2.0**.
