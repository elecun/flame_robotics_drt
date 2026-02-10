"""
3D Visualizer using Vedo
@note
- Vedo is a Python library for 3D visualization based on VTK (Visualization Toolkit).
- VedoVisualizer is a class that renders 3D geometries
"""

import threading
from collections import deque
import time
import numpy as np
import vedo
import open3d as o3d
from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe

from common import zapi
from common.graphic_device import GraphicDevice

class VedoVisualizer:
    def __init__(self, config:dict=None, zpipe:ZPipe=None):
        if config is None:
            config = {}
    
        self.__console = ConsoleLogger.get_logger()
        
        # Initialize message queue and stats
        self._message_lock = threading.Lock()
        self._pending_messages = deque(maxlen=100)
        self._message_stats = {'received': 0, 'dropped': 0}

        # Device Detection (Reusing GraphicDevice from common)
        self.gdevice = GraphicDevice()
        self.__console.info(f"Graphic Device: Running on {self.gdevice.get_device_name()}")
        
        # GPU Acceleration check for Vedo (VTK)
        # Vedo uses VTK which can use GPU. We can check if we want to enforce any specific settings.
        # For this prototype, we rely on VTK's default behavior but log the preference.
        if "cuda" in self.gdevice.get_device_name().lower() or "mps" in self.gdevice.get_device_name().lower():
             self.__console.info("GPU Acceleration enabled for Vedo/VTK (if available via drivers)")
             vedo.settings.use_depth_peeling = True # Better transparency on GPU
        else:
             self.__console.info("Running on CPU mode for Vedo/VTK")

        # create & join asynczsocket
        # 1. Subscriber Socket (Recv data + System commands from SimTool)
        self.__socket = AsyncZSocket("VedoVisualizer", "subscribe")
        if self.__socket.create(pipeline=zpipe):
            transport = config.get("transport", "tcp")
            port = config.get("port", 9001)
            host = config.get("host", "localhost")
            if self.__socket.join(transport, host, port):
                self.__socket.subscribe("call")
                self.__socket.subscribe(zapi.TOPIC_SYSTEM)
                self.__socket.set_message_callback(self.__on_data_received)
                self.__console.debug(f"Socket created and joined: {transport}://{host}:{port}")
            else:
                self.__console.error("Failed to join socket")
        else:
            self.__console.error("Failed to create socket")
            
        # 2. Publisher Socket (Send System commands)
        self.sys_pub_port = config.get("sys_pub_port", 9003)
        self.__sys_socket = AsyncZSocket("VedoVisualizerSys", "publish")
        if self.__sys_socket.create(pipeline=zpipe):
            if self.__sys_socket.join("tcp", "*", self.sys_pub_port):
                self.__console.debug(f"System Publisher bound to: *: {self.sys_pub_port}")
            else:
                 self.__console.error("Failed to bind System Publisher")

        # Initialize Vedo Plotter
        window_title = config.get('window_title', f'Vedo Viewer (Optimized - {self.gdevice.get_device_name()})')
        window_size = config.get('window_size', [1920, 1080])
        bg_color = config.get('background_color', [1.0, 1.0, 1.0])
        
        self.plotter = vedo.Plotter(title=window_title, size=window_size, bg=bg_color, interactive=False) # We will control the loop
                

        # Add C-Space Axes/Box if requested
        if config.get("show_axes", False):
             self.c_bounds = config.get("c_space_bound", [5.0, 8.0, 5.0])
             c_bounds = self.c_bounds # keep local var for below usage
             self.c_center = [self.c_bounds[0]/2, self.c_bounds[1]/2, self.c_bounds[2]/2]

             c_space_box = vedo.Box(pos=(c_bounds[0]/2, c_bounds[1]/2, c_bounds[2]/2), length=c_bounds[0], width=c_bounds[1], height=c_bounds[2])
             c_space_box.wireframe().c('gray').alpha(0.3)
             
             # Create custom axes with 1-unit intervals
             x_ticks = [(i, str(i)) for i in range(int(c_bounds[0]) + 1)]
             y_ticks = [(i, str(i)) for i in range(int(c_bounds[1]) + 1)]
             z_ticks = [(i, str(i)) for i in range(int(c_bounds[2]) + 1)]
             
             axes_config = dict(xtitle='X', x_values_and_labels=x_ticks, ytitle='Y', y_values_and_labels=y_ticks, ztitle='Z', z_values_and_labels=z_ticks, c='black')
             c_space_axes = vedo.Axes(c_space_box, **axes_config)

             self.plotter.add(c_space_box, c_space_axes)

        
        # Flag for external termination
        self._should_close = False

        self.loop_count = 0
        self.last_log_time = time.time()
        self.last_frame_time = time.time()
        self.target_frequency_hz = 60
        self.fps_text = None

        if config.get("show_fps", False):
            self.fps_text = vedo.Text2D("FPS: 0.0", pos='top-left', s=1.0, c="black", bg="white", alpha=0.5)
            self.plotter.add(self.fps_text)

        # Register key callback
        self.plotter.add_callback("KeyPress", self._on_key_press)

    def _on_key_press(self, event):
        """Handle key press events for camera control"""
        if not event.keypress:
            return

        key = event.keypress
        
        if not hasattr(self, 'c_bounds'):
            return

        cx, cy, cz = self.c_center
        lx, ly, lz = self.c_bounds
        max_dim = max(lx, ly, lz)
        dist = max_dim * 2.0 # distance factor

        if key == '1': # XY Plane (Top View)
            self.plotter.camera.SetPosition(cx, cy, cz + dist)
            self.plotter.camera.SetFocalPoint(cx, cy, cz)
            self.plotter.camera.SetViewUp(0, 1, 0)
            self.plotter.renderer.ResetCameraClippingRange()
            self.__console.info("Camera set to XY Plane (Top View)")
            self.plotter.render()
        
        elif key == '2': # YZ Plane (Side View)
            self.plotter.camera.SetPosition(cx + dist, cy, cz)
            self.plotter.camera.SetFocalPoint(cx, cy, cz)
            self.plotter.camera.SetViewUp(0, 0, 1)
            self.plotter.renderer.ResetCameraClippingRange()
            self.__console.info("Camera set to YZ Plane (Side View)")
            self.plotter.render()

        elif key == '3': # XZ Plane (Front View)
            self.plotter.camera.SetPosition(cx, cy - dist, cz) # Look from -Y
            self.plotter.camera.SetFocalPoint(cx, cy, cz)
            self.plotter.camera.SetViewUp(0, 0, 1)
            self.plotter.renderer.ResetCameraClippingRange()
            self.__console.info("Camera set to XZ Plane (Front View)")
            self.plotter.render()

    def _convert_to_vedo(self, name, geometry):
        """Convert Open3D geometry (Tensor or Legacy) to Vedo Actor"""
        try:
            # Handle Tensor Geometry (convert to legacy/numpy first)
            if isinstance(geometry, o3d.t.geometry.TriangleMesh):
                # Ensure CPU device
                mesh_cpu = geometry.cpu()
                vertices = mesh_cpu.vertex.positions.numpy()
                faces = mesh_cpu.triangle.indices.numpy()
                
                # Check for colors
                colors = None
                if 'colors' in mesh_cpu.vertex:
                    colors = mesh_cpu.vertex.colors.numpy() * 255 # Vedo expects 0-255 or 0-1? 
                    # Vedo mesh.c() usually takes name or hex. mesh.pointdata['RGBA'] can be used.
                
                actor = vedo.Mesh([vertices, faces])
                
                # If it's the coordinate frame, color it appropriately if possible, 
                # but standard O3D coord frame is mesh. 
                # For simplicity, apply a default color or try to transfer colors if complex.
                if "origin_coordinate" in name:
                     # Re-create axes using vedo for better look
                     return vedo.Axes(xrange=[0,1], yrange=[0,1], zrange=[0,1]) # Simplified
                
                 # If ground, color it
                if "ground" in name:
                     actor.c("gray").alpha(0.5)

                return actor

            elif isinstance(geometry, o3d.geometry.TriangleMesh):
                 # Legacy Mesh
                 vertices = np.asarray(geometry.vertices)
                 triangles = np.asarray(geometry.triangles)
                 actor = vedo.Mesh([vertices, triangles])
                 return actor
            
            # Add other types (PointCloud, etc) if needed
            return None
        except Exception as e:
            self.__console.error(f"Failed to convert geometry {name}: {e}")
            return None

    def run(self, frequency_hz: int):
        self.target_frequency_hz = frequency_hz
        self.__console.debug(f"Starting Vedo GUI loop (target: {frequency_hz} Hz)")
        
        # shape initial view to 3D perspective if C-Space is defined
        if hasattr(self, 'c_bounds'):
             cx, cy, cz = self.c_center
             max_dim = max(self.c_bounds)
             # Isometric-like view
             self.plotter.show(interactive=False)
             # Manually set camera after show (or before, but sometimes show resets it if no objects)
             if self.plotter.camera:
                 self.plotter.camera.SetPosition(cx + max_dim*2.0, cy - max_dim*2.0, cz + max_dim*2.0)
                 self.plotter.camera.SetFocalPoint(cx, cy, cz)
                 self.plotter.camera.SetViewUp(0, 0, 1)
                 self.plotter.renderer.ResetCameraClippingRange()
                 self.plotter.render()
        else:
             self.plotter.show(interactive=False)
        
        while not self._should_close:
            if not self.plotter.interactor or self.plotter.interactor.GetDone():
                break
            start_time = time.time()
            
            # Logic step
            if not self._on_tick():
                break
                
            # Render step
            self.plotter.render()
            
            # Event processing (interactor)
            if self.plotter.interactor:
                self.plotter.interactor.ProcessEvents()
            
            # Timing control
            elapsed = time.time() - start_time
            sleep_time = (1.0 / self.target_frequency_hz) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
        self.on_close()
        self.plotter.close()
        self.__console.info("Visualizer closed")
    
    def _on_tick(self) -> bool:
        """Called every frame by the GUI event loop."""
        
        # 1. Log Frequency
        self.loop_count, self.last_log_time = self._log_rendering_frequency(self.loop_count, self.last_log_time)
        
        # 2. Poll ZMQ Messages
        processed_count = 0
        while True:
            try:
                with self._message_lock:
                    if not self._pending_messages:
                        break
                    multipart_data = self._pending_messages.popleft()
                
                self._process_data(multipart_data)
                processed_count += 1
                if processed_count > 10: 
                    break
            except Exception as e:
                self.__console.error(f"Error processing message: {e}")
                break
        
        return True

    def _process_data(self, multipart_data):
         # Placeholder for data processing
         pass

    def _log_rendering_frequency(self, loop_count, last_log_time):
        current_time = time.time()
        
        # 1. Calculate Instantaneous FPS for Text Overlay
        frame_duration = current_time - self.last_frame_time
        if frame_duration > 0:
            inst_fps = 1.0 / frame_duration
            if self.fps_text:
                self.fps_text.text(f"FPS: {inst_fps:.1f}")
        
        self.last_frame_time = current_time
            
        return loop_count, last_log_time

    def on_close(self):
        # Send termination signal
        if hasattr(self, '_VedoVisualizer__sys_socket') and self.__sys_socket:
             zapi.zapi_destroy(self.__sys_socket)
             time.sleep(0.1)
             self.__sys_socket.destroy_socket()

        # Clean up subscriber socket
        if hasattr(self, '_VedoVisualizer__socket') and self.__socket:
            self.__socket.destroy_socket()
            self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")
        
    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
        try:
            if len(multipart_data) >= 2:
                topic = multipart_data[0]
                msg = multipart_data[1]
                
                # CHECK FOR SYSTEM TERMINATION
                if zapi.zapi_check_system_message(topic, msg):
                    self.__console.warning("Received TERMINATION signal")
                    self._should_close = True
                    return

                with self._message_lock:
                    if len(self._pending_messages) >= self._pending_messages.maxlen:
                        self._message_stats['dropped'] += 1
                    
                    self._pending_messages.append(multipart_data)
                    self._message_stats['received'] += 1
                    
        except Exception as e:
            self.__console.error(f"({self.__class__.__name__}) Error processing received data: {e}")
