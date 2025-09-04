# Isaac Sim ROS2 Camera Publisher Extension (Pure Publisher)
# 職責：將 Isaac Sim ROS camera 發布到 ROS2 話題 /ROBOTNAME/camera/image_raw
# 支援 rosbridge_websocket 即時串流

import omni.ext
import omni.ui as ui
import omni.usd
import omni.timeline
import carb
from omni.isaac.sensor import Camera
from omni.isaac.core import World
from pxr import UsdGeom, Gf
import threading
import time
import requests
import traceback

# ROS2 發布相關導入
try:
    import omni.replicator.core as rep
    import omni.syntheticdata._syntheticdata as sd
    import omni.syntheticdata
    import omni.graph.core as og
    from omni.isaac.core.utils import extensions
    ROS2_AVAILABLE = True
    print("✅ ROS2 publishing modules available")
except ImportError as e:
    ROS2_AVAILABLE = False
    print(f"⚠️ ROS2 publishing modules not available: {e}")

class Extension(omni.ext.IExt):
    """
    Isaac Sim ROS2 Camera Publisher Extension (Pure Publisher)
    只負責將 Isaac Sim 中的 ROS camera 發布到 ROS2 話題
    """
    
    def on_startup(self, ext_id):
        print("[isaac.camera.ros2.publisher] Isaac Sim ROS2 Camera Publisher Extension")
        print("🎯 Mode: Pure Publisher - Isaac Sim Camera → ROS2 Topic")
        print("📡 Target Topic: /ROBOTNAME/camera/image_raw")
        print("🌐 Compatible with rosbridge_websocket")
        
        # Initialize variables
        self._window = None
        self._camera_sensor = None
        self._world = None
        
        # ROS2 Publishing variables
        self._is_publishing = False
        self._publishing_thread = None
        self._stop_publishing_flag = False
        self.ros2_publishers = {}
        
        # Settings
        self.ros2_publishing_frequency = 10  # Hz
        self.ros2_camera_frame_id = "camera_ROBOTNAME" # TODO
        self.ros2_topic = "/ROBOTNAME/camera/image_raw" # TODO
        
        # Target camera path in Isaac Sim
        self.target_camera_path = "/World/Demo_8F/_R05D00002_only_bottom_sim_/tn__7R05D00002_only_bottom_sim_/Camera_ROBOTNAME" # TODO
        
        # rosbridge status
        self.rosbridge_url = "ws://localhost:9090"
        self.rosbridge_status = "Unknown"
        
        # Create UI
        self._create_publisher_ui()
        
        # Check rosbridge status
        self._check_rosbridge_status()
    
    def on_shutdown(self):
        print("[isaac.camera.ros2.publisher] Publisher Extension Shutdown")
        self._stop_publishing_safely()
        
        if self._window:
            self._window.destroy()
            self._window = None
        
        print("✅ ROS2 Publisher extension shutdown completed")
    
    def _create_publisher_ui(self):
        """創建純發布者UI"""
        try:
            self._window = ui.Window("ROS2 Camera Publisher", width=420, height=380)
            
            with self._window.frame:
                with ui.VStack():
                    # Header
                    ui.Label("ROS2 Camera Publisher", style={"font_size": 16})
                    ui.Label("Pure Publisher Mode - Isaac Sim → ROS2", style={"color": 0x888888})
                    
                    ui.Spacer(height=5)
                    
                    # System Status
                    with ui.HStack():
                        ui.Label("ROS2 Modules:", width=100)
                        ros2_status = "Available" if ROS2_AVAILABLE else "Not Available"
                        ui.Label(ros2_status, style={"color": 0x00FF00 if ROS2_AVAILABLE else 0xFF0000})
                    
                    with ui.HStack():
                        ui.Label("rosbridge:", width=100)
                        self._rosbridge_status_label = ui.Label("Checking...")
                    
                    ui.Spacer(height=10)
                    
                    # Camera Configuration
                    ui.Label("=== Camera Configuration ===")
                    
                    ui.Label(f"Camera Path: {self.target_camera_path}", style={"color": 0x888888})
                    ui.Label(f"Frame ID: {self.ros2_camera_frame_id}", style={"color": 0x888888})
                    
                    # Topic Configuration
                    with ui.HStack():
                        ui.Label("ROS2 Topic:", width=100)
                        self._topic_field = ui.StringField(width=250)
                        self._topic_field.model.set_value(self.ros2_topic)
                    
                    # Publishing Frequency
                    with ui.HStack():
                        ui.Label("Frequency:", width=100)
                        self._freq_field = ui.FloatField(width=80)
                        self._freq_field.model.set_value(self.ros2_publishing_frequency)
                        ui.Label("Hz")
                    
                    ui.Spacer(height=10)
                    
                    # Publishing Controls
                    ui.Label("=== Publishing Controls ===")
                    
                    with ui.HStack():
                        self._start_btn = ui.Button("Start Publishing", width=150)
                        self._start_btn.set_mouse_pressed_fn(lambda x, y, b, m: self._start_publishing())
                        
                        self._stop_btn = ui.Button("Stop Publishing", width=150)
                        self._stop_btn.set_mouse_pressed_fn(lambda x, y, b, m: self._stop_publishing())
                    
                    # Status Display
                    self._status_label = ui.Label("Status: Ready")
                    self._info_label = ui.Label("Press Play in Isaac Sim before publishing")
                    
                    ui.Spacer(height=10)
                    
                    # rosbridge Support
                    ui.Label("=== rosbridge_websocket Support ===")
                    
                    with ui.HStack():
                        ui.Label("rosbridge URL:", width=100)
                        self._rosbridge_field = ui.StringField(width=200)
                        self._rosbridge_field.model.set_value(self.rosbridge_url)
                    
                    with ui.HStack():
                        self._check_rosbridge_btn = ui.Button("Check rosbridge", width=150)
                        self._check_rosbridge_btn.set_mouse_pressed_fn(lambda x, y, b, m: self._check_rosbridge_status())
                    
                    self._rosbridge_info = ui.Label("rosbridge enables web browser access to ROS2 topics")
                    
                    ui.Spacer(height=10)
                    
                    # Usage Instructions
                    # ui.Label("=== Usage Instructions ===")
                    # ui.Label("1. Ensure Isaac Sim scene is loaded", style={"color": 0x888888})
                    # ui.Label("2. Press Play in Isaac Sim", style={"color": 0x888888})  
                    # ui.Label("3. Click 'Start Publishing'", style={"color": 0x888888})
                    # ui.Label("4. Use external script to subscribe and process", style={"color": 0x888888})
                    
                    # Callbacks for field changes
                    def on_topic_change():
                        new_topic = self._topic_field.model.get_value_as_string()
                        if new_topic.strip():
                            self.ros2_topic = new_topic.strip()
                            print(f"🔄 Topic updated: {self.ros2_topic}")
                    
                    def on_freq_change():
                        new_freq = self._freq_field.model.get_value_as_float()
                        if new_freq > 0:
                            self.ros2_publishing_frequency = new_freq
                            print(f"🔄 Frequency updated: {self.ros2_publishing_frequency} Hz")
                    
                    def on_rosbridge_url_change():
                        new_url = self._rosbridge_field.model.get_value_as_string()
                        if new_url.strip():
                            self.rosbridge_url = new_url.strip()
                            print(f"🔄 rosbridge URL updated: {self.rosbridge_url}")
                    
                    self._topic_field.model.add_value_changed_fn(lambda model: on_topic_change())
                    self._freq_field.model.add_value_changed_fn(lambda model: on_freq_change())
                    self._rosbridge_field.model.add_value_changed_fn(lambda model: on_rosbridge_url_change())
            
            print("✅ Publisher UI created successfully")
            
        except Exception as e:
            carb.log_error(f"UI creation failed: {e}")
            print(f"❌ UI creation failed: {e}")
    
    def _check_rosbridge_status(self):
        """檢查 rosbridge_websocket 狀態"""
        def check_in_background():
            try:
                # 使用簡單的 socket 測試代替 WebSocket
                import socket
                
                # 解析 rosbridge URL
                url_parts = self.rosbridge_url.replace('ws://', '').split(':')
                host = url_parts[0]
                port = int(url_parts[1]) if len(url_parts) > 1 else 9090
                
                # 測試 TCP 連接
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3)
                
                try:
                    result = sock.connect_ex((host, port))
                    if result == 0:
                        self.rosbridge_status = "Connected"
                        self._update_rosbridge_status("✅ Connected")
                    else:
                        self.rosbridge_status = "Connection refused"
                        self._update_rosbridge_status("❌ Connection refused")
                except socket.timeout:
                    self.rosbridge_status = "Timeout"
                    self._update_rosbridge_status("⏱️ Connection timeout")
                except Exception as e:
                    self.rosbridge_status = "Error"
                    self._update_rosbridge_status(f"❌ Error: {str(e)[:20]}")
                finally:
                    sock.close()
                
            except ImportError:
                # 如果沒有 websocket 模組，嘗試簡單的 HTTP 檢查
                try:
                    import urllib.request
                    # 檢查是否有 rosbridge 在運行 (通常會有 HTTP 服務)
                    http_url = self.rosbridge_url.replace('ws://', 'http://').replace(':9090', ':8080')
                    response = urllib.request.urlopen(http_url, timeout=3)
                    self.rosbridge_status = "HTTP OK"
                    self._update_rosbridge_status("🌐 HTTP service detected")
                except:
                    self.rosbridge_status = "Not found"
                    self._update_rosbridge_status("❌ Not running")
                    
            except Exception as e:
                self.rosbridge_status = "Error"
                self._update_rosbridge_status(f"❌ Error: {str(e)[:20]}")
                print(f"rosbridge check error: {e}")
        
        threading.Thread(target=check_in_background, daemon=True).start()
    
    def _update_rosbridge_status(self, status: str):
        """更新 rosbridge 狀態顯示"""
        try:
            if hasattr(self, '_rosbridge_status_label'):
                self._rosbridge_status_label.text = status
        except:
            pass
    
    def _start_publishing(self):
        """開始 ROS2 發布"""
        try:
            print("=== Starting ROS2 Publishing ===")
            
            if self._is_publishing:
                print("⚠️ Publishing already active")
                self._update_status("Already active")
                return
            
            if not ROS2_AVAILABLE:
                print("❌ ROS2 publishing modules not available")
                self._update_status("ROS2 not available")
                return
            
            # 檢查模擬狀態
            timeline = omni.timeline.get_timeline_interface()
            if not timeline.is_playing():
                print("❌ Simulation not running - Press Play in Isaac Sim")
                self._update_status("Simulation stopped - Press Play")
                self._update_info("⚠️ Isaac Sim must be playing before publishing")
                return
            
            print("✓ Simulation is running")
            
            # 更新設定
            self.ros2_topic = self._topic_field.model.get_value_as_string()
            self.ros2_publishing_frequency = self._freq_field.model.get_value_as_float()
            
            print(f"📡 Publishing to: {self.ros2_topic}")
            print(f"📊 Frequency: {self.ros2_publishing_frequency} Hz")
            print(f"🏷️ Frame ID: {self.ros2_camera_frame_id}")
            
            # 初始化 World
            self._ensure_world()
            
            # 初始化相機
            success = self._initialize_camera()
            if not success:
                self._update_status("Camera initialization failed")
                return
            
            # 設置 ROS2 發布器
            success = self._setup_ros2_publisher()
            if not success:
                self._update_status("Publisher setup failed")
                return
            
            # 啟動發布監控
            self._stop_publishing_flag = False
            self._publishing_thread = threading.Thread(target=self._publishing_monitor_loop, daemon=True)
            self._publishing_thread.start()
            
            self._is_publishing = True
            self._update_status(f"✅ Publishing at {self.ros2_publishing_frequency} Hz")
            self._update_info(f"Publishing to {self.ros2_topic}")
            
            print(f"✅ ROS2 publishing started successfully")
            
        except Exception as e:
            error_msg = f"❌ Publishing error: {str(e)}"
            self._update_status(error_msg)
            print(f"❌ Publishing error: {e}")
            traceback.print_exc()
    
    def _ensure_world(self):
        """確保 World 實例存在"""
        try:
            if self._world is None:
                try:
                    from omni.isaac.core import World
                    world_instance = World.instance()
                except:
                    world_instance = None
                    
                if world_instance is None:
                    from omni.isaac.core import World
                    self._world = World(stage_units_in_meters=1.0)
                    print("✅ World created")
                else:
                    self._world = world_instance
                    print("✅ Using existing World")
        except Exception as e:
            print(f"❌ World setup error: {e}")
            self._world = None
    
    def _initialize_camera(self):
        """初始化相機感測器"""
        try:
            print(f"🎥 Initializing camera: {self.target_camera_path}")
            
            # 檢查相機路徑是否存在
            stage = omni.usd.get_context().get_stage()
            camera_prim = stage.GetPrimAtPath(self.target_camera_path)
            if not camera_prim.IsValid():
                print(f"❌ Camera not found at path: {self.target_camera_path}")
                return False
            
            # 創建 Camera sensor
            self._camera_sensor = Camera(
                prim_path=self.target_camera_path,
                frequency=60,  # 使用高頻率，通過 step_size 控制發布頻率
                resolution=(640, 480)
            )
            
            self._camera_sensor.initialize()
            print(f"✅ Camera sensor initialized")
            
            # 等待相機準備
            if self._world is not None:
                try:
                    for i in range(5):
                        self._world.step(render=True)
                        time.sleep(0.1)
                except Exception as e:
                    print(f"World step warning: {e}")
                    time.sleep(1.0)
            else:
                time.sleep(1.0)
            
            # 測試相機
            for attempt in range(5):
                try:
                    rgb_data = self._camera_sensor.get_rgb()
                    if rgb_data is not None and rgb_data.size > 0:
                        print(f"✅ Camera test successful! Shape: {rgb_data.shape}")
                        return True
                except:
                    pass
                time.sleep(0.2)
            
            print("❌ Camera test failed")
            return False
            
        except Exception as e:
            print(f"❌ Camera initialization error: {e}")
            return False
    
    def _setup_ros2_publisher(self):
        """設置 ROS2 發布器"""
        if not ROS2_AVAILABLE:
            print("❌ ROS2 modules not available")
            return False
            
        try:
            # 啟用 ROS2 橋接擴展
            extensions.enable_extension("omni.isaac.ros2_bridge")
            
            print(f"🔧 Setting up ROS2 publisher:")
            print(f"   📡 Topic: {self.ros2_topic}")
            print(f"   🏷️ Frame ID: {self.ros2_camera_frame_id}")
            print(f"   📊 Frequency: {self.ros2_publishing_frequency} Hz")
            
            # 獲取相機的 render product
            if not self._camera_sensor:
                print("❌ Camera sensor not available")
                return False
                
            render_product = self._camera_sensor._render_product_path
            print(f"✅ Render product: {render_product}")
            
            # 計算發布步長
            step_size = int(60 / self.ros2_publishing_frequency)
            print(f"✅ Step size: {step_size}")
            
            # 創建 RGB 影像發布器
            rv = sd.SensorType.Rgb.name
            render_var = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(rv)
            
            writer = rep.writers.get(render_var + "ROS2PublishImage")
            writer.initialize(
                frameId=self.ros2_camera_frame_id,
                nodeNamespace="",
                queueSize=1,
                topicName=self.ros2_topic
            )
            writer.attach([render_product])
            print(f"✅ Writer attached to render product")
            
            # 設置發布頻率
            try:
                gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                    render_var + "IsaacSimulationGate", render_product
                )
                og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
                print(f"✅ Publishing frequency set: {self.ros2_publishing_frequency} Hz")
            except Exception as gate_error:
                print(f"⚠️ Gate setup warning: {gate_error}")
            
            # 儲存發布器資訊
            self.ros2_publishers[self.target_camera_path] = {
                "topic_name": self.ros2_topic,
                "frame_id": self.ros2_camera_frame_id,
                "writer": writer,
                "render_product": render_product,
                "step_size": step_size
            }
            
            print(f"✅ ROS2 publisher setup complete")
            print(f"📡 Check with: ros2 topic echo {self.ros2_topic}")
            print(f"🌐 rosbridge users can access at: {self.rosbridge_url}")
            return True
            
        except Exception as e:
            print(f"❌ ROS2 publisher setup failed: {e}")
            traceback.print_exc()
            return False
    
    def _publishing_monitor_loop(self):
        """發布監控循環"""
        try:
            print("🔄 Starting publishing monitor...")
            cycle_count = 0
            
            while not self._stop_publishing_flag:
                try:
                    cycle_count += 1
                    
                    # 每 30 個循環顯示一次狀態
                    if cycle_count % 30 == 0:
                        print(f"📡 Publishing monitor - {cycle_count} cycles")
                        # 更新狀態顯示
                        if self._is_publishing:
                            self._update_status(f"✅ Active - {cycle_count} cycles")
                    
                    # 檢查模擬狀態
                    timeline = omni.timeline.get_timeline_interface()
                    if not timeline.is_playing():
                        print("⚠️ Simulation stopped, pausing publishing")
                        self._update_status("⏸️ Paused - Simulation stopped")
                        self._update_info("Press Play in Isaac Sim to resume")
                    
                    time.sleep(1.0)
                        
                except Exception as monitor_error:
                    print(f"❌ Monitor error: {monitor_error}")
                    time.sleep(1.0)
                    
            print(f"📡 Publishing monitor ended. Total cycles: {cycle_count}")
                    
        except Exception as e:
            print(f"❌ Monitor thread error: {e}")
        finally:
            if self._is_publishing:
                self._update_status("Stopped")
    
    def _stop_publishing(self):
        """停止 ROS2 發布"""
        print("🛑 Stopping ROS2 publishing...")
        self._stop_publishing_safely()
    
    def _stop_publishing_safely(self):
        """安全停止 ROS2 發布"""
        try:
            if not self._is_publishing:
                return
            
            print("🛑 Setting stop flag...")
            self._stop_publishing_flag = True
            
            if self._publishing_thread and self._publishing_thread.is_alive():
                print("⏳ Waiting for publishing thread...")
                self._publishing_thread.join(timeout=3.0)
                if self._publishing_thread.is_alive():
                    print("⚠️ Thread did not stop gracefully")
                else:
                    print("✅ Publishing thread stopped")
            
            # 清理發布器
            self.ros2_publishers.clear()
            if hasattr(self, '_camera_sensor'):
                self._camera_sensor = None
            
            self._is_publishing = False
            self._update_status("Stopped")
            self._update_info("Ready to publish")
            print("✅ Publishing stopped completely")
            
        except Exception as e:
            print(f"❌ Stop error: {e}")
            self._is_publishing = False
            self._update_status("Error")
    
    def _update_status(self, status: str):
        """更新狀態顯示"""
        try:
            if hasattr(self, '_status_label'):
                self._status_label.text = f"Status: {status}"
        except:
            pass
    
    def _update_info(self, info: str):
        """更新信息顯示"""
        try:
            if hasattr(self, '_info_label'):
                self._info_label.text = info
        except:
            pass
