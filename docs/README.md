# Isaac Sim ROS2 Camera Publisher Extension

這個 Extension 會將 **Isaac Sim 中的 Camera 畫面** 透過 **ROS2 Bridge** 發布到指定的 ROS2 Topic，並支援 `rosbridge_websocket`，方便 Web/Browser 端即時存取。

---

## 📦 功能
- 將 Isaac Sim 相機影像發布到 **ROS2 Topic**（預設：`/ROBOTNAME/camera/image_raw`）  
- 內建 UI，可視化控制：
  - 開始/停止發布  
  - 修改 Topic 名稱、頻率、rosbridge URL  
- 支援 **rosbridge_websocket**，方便瀏覽器或外部系統存取  

---

## 🚀 安裝與使用

1. **放置 Extension**
   - 將 `extension.py` 放到 Isaac Sim 的 extension 專案目錄，例如：  
     ```
     <your_isaac_sim_extensions>/isaac.camera.ros2.publisher/
     ```
   - 確保 `extension.toml` 已正確配置（可參考其他 Isaac Sim extension 的寫法）。

2. **啟用 Extension**
   - 打開 Isaac Sim → `Window > Extension Manager`  
   - 搜尋 `ROS2 Camera Publisher` → 啟用  

3. **啟動模擬**
   - 載入場景（scene）  
   - 確保相機已存在於指定路徑（詳見下方 **修改相機路徑**）  
   - 點擊 **▶️ Play**  

4. **開始發布**
   - 開啟 `ROS2 Camera Publisher` 視窗（在 `Window > ROS2 Camera Publisher`）  
   - 點擊 **🚀 Start Publishing**  
   - 影像會發布到指定 ROS2 Topic  

5. **驗證**
   - 在 ROS2 環境中輸入：  
     ```bash
     ros2 topic list
     ros2 topic echo /ROBOTNAME/camera/image_raw
     ```
   - 若使用 `rosbridge_websocket`，可從瀏覽器或其他 WebSocket 客戶端訂閱  

---

## ⚙️ 使用者需要修改的程式碼

不同使用者可能需要根據場景或網路配置修改以下程式碼設定：
在 # TODO 的地方更改成自己的相機路徑/ros camera的frame id/topic名稱

### 1. 相機路徑
```python
self.target_camera_path = "/World/Demo_8F/_R05D00002_only_bottom_sim_/tn__7R05D00002_only_bottom_sim_/Camera_ROBOTNAME"
```
- 這是 Isaac Sim 場景中 **Camera 的路徑**。  
- 請改成你自己的 Camera prim 的路徑。  
- 可從 **Stage 視窗** 複製 prim path。

---

### 2. ROS2 Topic
```python
self.ros2_topic = "/ROBOTNAME/camera/image_raw"
```
- 修改成你要發布的 topic 名稱，例如：  
  - `/camera/front/image_raw`  
  - `/robot1/camera/color`  

在 UI 介面中也可即時更改。

---

### 3. 頻率 (Hz)
```python
self.ros2_publishing_frequency = 10
```
- 預設為 **10Hz**。  
- 可依需求調整，例如 **30Hz 或 60Hz**。  
- UI 裡可直接修改。

---

### 4. Frame ID
```python
self.ros2_camera_frame_id = "camera_ROBOTNAME"
```
- 每個 ROS2 影像訊息都會帶有 `std_msgs/Header.frame_id`，下游模組會用來識別資料來源。  
- 如果有多個相機，請修改為不同的名稱，例如：  
  - `"camera_front"`  
  - `"camera_left"`  
  - `"camera_right"`  

目前程式碼中沒有 UI 欄位修改 `frame_id`，若有需要可直接改程式碼。

---

### 5. rosbridge URL
```python
self.rosbridge_url = "ws://localhost:9090"
```
- 若 rosbridge 不是在本機，請改成對應 IP 與 Port，例如：  
  - `"ws://192.168.1.10:9090"`  

---

## 🛠️ 除錯指南
- **相機找不到**  
  - 確認 `self.target_camera_path` 與場景中的 Camera prim 路徑一致  
- **沒看到 ROS2 topic**  
  - 確認 Isaac Sim 中已啟動 `omni.isaac.ros2_bridge`  
  - 在 UI 中檢查 ROS2 Modules 狀態是否為 **Available**  
- **rosbridge 無法連線**  
  - 確認 rosbridge 已啟動，例如：  
    ```bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```  
  - 檢查 port 是否為 9090，若不同需修改 `rosbridge_url`  

---

## 📖 使用流程簡要
1. 開啟 Isaac Sim 並載入場景  
2. 確認相機路徑正確  
3. 修改程式碼中的 Topic/Frame ID/頻率（如有需要）  
4. 啟用 `ROS2 Camera Publisher` Extension  
5. 按 **▶️ Play**  
6. 點擊 **🚀 Start Publishing**  
7. 在 ROS2 或 rosbridge 客戶端驗證資料  
