# Isaac Sim ROS2 Camera Publisher Extension

This Extension publishes **camera images from Isaac Sim** to a **ROS2 topic** using the ROS2 Bridge. It also supports `rosbridge_websocket`, allowing real-time access from web or browser clients.

---

## Features
- Publish Isaac Sim camera images to a **ROS2 topic** (default: `/ROBOTNAME/camera/image_raw`)
- Built-in UI with:
  - Start/Stop publishing
  - Editable topic name, publish frequency, and rosbridge URL
- Supports **rosbridge_websocket** for browser-based or external subscribers

---

## Installation & Usage

### 1. Place the Extension
- Copy `extension.py` into your Isaac Sim extensions directory, for example:
  ```
  <your_isaac_sim_extensions>/isaac.camera.ros2.publisher/
  ```
- Ensure `extension.toml` is properly configured (refer to other Isaac Sim extensions if needed).

### 2. Enable the Extension
- Open Isaac Sim → `Window > Extension Manager`
- Search for **ROS2 Camera Publisher** → Enable it

### 3. Start the Simulation
- Load your scene
- Ensure the camera exists at the correct prim path (see **Modify Camera Path** below)
- Press **Play**

### 4. Start Publishing
- Open `Window > ROS2 Camera Publisher`
- Click **Start Publishing**
- Images will be published to the configured ROS2 topic

### 5. Verification
Using ROS2:
```bash
ros2 topic list
ros2 topic echo /ROBOTNAME/camera/image_raw
```

For `rosbridge_websocket`, subscribe using a browser or WebSocket client.

---

## Code Sections Users May Need to Modify

You may need to modify the settings in the **TODO** sections depending on your scene or network setup.

### 1. Camera Path
```python
self.target_camera_path = "/World/Demo_8F/_R05D00002_only_bottom_sim_/tn__7R05D00002_only_bottom_sim_/Camera_ROBOTNAME"
```
- Replace with your own camera prim path.
- You can copy the prim path from the **Stage** panel.

---

### 2. ROS2 Topic
```python
self.ros2_topic = "/ROBOTNAME/camera/image_raw"
```
- Replace with your desired ROS2 topic name, e.g.:
  - `/camera/front/image_raw`
  - `/robot1/camera/color`
- This can also be edited directly in the UI.

---

### 3. Publishing Frequency (Hz)
```python
self.ros2_publishing_frequency = 10
```
- Default: **10 Hz**
- Can be increased (e.g., 30 Hz or 60 Hz)
- Also editable via the UI

---

### 4. Frame ID
```python
self.ros2_camera_frame_id = "camera_ROBOTNAME"
```
- Used in the `Header.frame_id` field of ROS2 Image messages
- Modify when publishing from multiple cameras (e.g. `"camera_front"`, `"camera_left"`)
- Currently must be edited in code (no UI field yet)

---

### 5. rosbridge URL
```python
self.rosbridge_url = "ws://localhost:9090"
```
- Change this if rosbridge is on another machine, e.g.:
  - `"ws://192.168.1.10:9090"`

---

## Troubleshooting

### Camera not found
- Ensure `self.target_camera_path` matches the camera prim path in the scene.

### No ROS2 topic is published
- Check that `omni.isaac.ros2_bridge` is enabled.
- Verify the ROS2 Modules status in the UI shows **Available**.

### rosbridge connection issues
- Ensure rosbridge is running:
  ```bash
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  ```
- Confirm the port (default: 9090) and update `rosbridge_url` if needed.

---

## Quick Workflow Summary
1. Open Isaac Sim and load your scene  
2. Verify the camera prim path  
3. Adjust topic/frame ID/frequency as needed  
4. Enable **ROS2 Camera Publisher**  
5. Press **Play**  
6. Click **Start Publishing**  
7. Verify messages via ROS2 or rosbridge  

---
