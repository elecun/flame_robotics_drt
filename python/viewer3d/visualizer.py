"""
3D Visualizer using Open3D
"""
import os
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import trimesh
import time
import zmq
import json
import numpy as np
import threading

from util.logger.console import ConsoleLogger
from urdf_parser import URDF
from viewer3d.geomerty import geometryAPI
from math import pi, cos, sin
from pytransform3d import rotations, transformations
import math
from pytransform3d.transformations import transform_from
from pytransform3d.rotations import matrix_from_euler
from common.zpipe import AsyncZSocket, ZPipe

class Open3DVisualizer(geometryAPI):
    _geometry_container = {}

    def __init__(self, config:dict, zpipe:ZPipe):
        super().__init__()

        # initialize
        self.__config = config
        self.__console = ConsoleLogger.get_logger()
        gui.Application.instance.initialize()

        # create window and scene
        self._window = gui.Application.instance.create_window(title=config["window_title"], 
                                                              width=config.get("window_size", [1280, 720])[0],
                                                              height=config.get("window_size", [1280, 720])[1])
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self._window.renderer)
        self._scene.scene.set_background(config.get("background-color", [1.0, 1.0, 1.0, 1.0])) # RGBA
        self._scene.scene.scene.set_sun_light([-1, -1, -1], [1, 1, 1], 100000)
        self._scene.scene.scene.enable_sun_light(True)        

        # camera parameter
        intrinsics = o3d.camera.PinholeCameraIntrinsic(640, 480, 525, 525, 320, 240)
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = rotations.active_matrix_from_extrinsic_euler_xyz([math.radians(90), 0, 0])
        extrinsic[0:3, 3] = [-2.5, 1.0, 4.0]

        # viewpoint 
        view_bbox = o3d.geometry.AxisAlignedBoundingBox([-3, -3, -3], [3, 3, 3])
        self._scene.setup_camera(intrinsics, extrinsic, view_bbox)
        # self._scene.setup_camera(60, view_bbox, [2.5, 2.5, 2])
        self._window.add_child(self._scene)

        # show initial geometry
        self.show_initial_geometry(config=config)

        # close event
        self._window.set_on_close(self.on_close)

        # key event callbacks
        self._window.set_on_key(self.on_key_event)

        # create & join asynczsocket
        self.__socket = AsyncZSocket("Open3DVisualizer", "subscribe")
        if self.__socket.create(pipeline=zpipe):
            transport = config.get("transport", "tcp")
            port = config.get("port", 9001)
            host = config.get("host", "localhost")
            if self.__socket.join(transport, host, port):
                self.__socket.subscribe("call")
                self.__socket.set_message_callback(self.__on_data_received)
                self.__console.debug(f"Socket created and joined: {transport}://{host}:{port}")
            else:
                self.__console.error("Failed to join socket")
        else:
            self.__console.error("Failed to create socket")
    
        
        # flags
        self.__show_origin_coord = False
        
        # Message processing queue for GUI thread
        from collections import deque
        self._pending_messages = deque(maxlen=100)  # Limit queue size to prevent memory issues
        self._message_lock = threading.Lock()
        self._message_stats = {'received': 0, 'processed': 0, 'dropped': 0}
        
        # GUI update timer with adaptive frequency
        self._gui_timer = None
        self._base_update_interval = 33  # 30 FPS base rate
        self._current_update_interval = self._base_update_interval
        self._setup_gui_timer()
    
    def _setup_gui_timer(self):
        """Setup GUI timer for periodic updates"""
        try:
            # Create timer that runs in GUI thread
            # Update at 30 FPS (33.33ms interval)
            update_interval_ms = 33  # 30 FPS
            
            # Schedule periodic updates
            def timer_callback():
                try:
                    self._process_pending_messages()
                    # Schedule next update
                    gui.Application.instance.post_to_main_thread(self._window, timer_callback)
                except Exception as e:
                    self.__console.error(f"Error in timer callback: {e}")
            
            # Start the timer
            gui.Application.instance.post_to_main_thread(self._window, timer_callback)
            self.__console.debug("Setup GUI timer for periodic updates")
            
        except Exception as e:
            self.__console.error(f"Failed to setup GUI timer: {e}")
            # Fallback to old thread method
            self._start_fallback_refresh_thread()
    
    def _process_pending_messages(self):
        """Process pending ZMQ messages in GUI thread"""
        try:
            messages_to_process = []
            with self._message_lock:
                # Process up to 10 messages per cycle to avoid blocking GUI too long
                max_process_per_cycle = min(10, len(self._pending_messages))
                
                for _ in range(max_process_per_cycle):
                    if self._pending_messages:
                        messages_to_process.append(self._pending_messages.popleft())
            
            # Process messages outside the lock
            for multipart_data in messages_to_process:
                self.__zpipe_msg_process(multipart_data)
                self._message_stats['processed'] += 1
                
            # Log stats periodically for monitoring
            # if self._message_stats['processed'] % 100 == 0:
            #     self.__console.debug(f"Message stats - Received: {self._message_stats['received']}, "
            #                        f"Processed: {self._message_stats['processed']}, "
            #                        f"Dropped: {self._message_stats['dropped']}, "
            #                        f"Pending: {len(self._pending_messages)}")
                
        except Exception as e:
            self.__console.error(f"Error processing pending messages: {e}")
    
    def _start_fallback_refresh_thread(self):
        """Fallback refresh thread if GUI timer fails"""
        self._refresh_running = True
        self._refresh_thread = threading.Thread(target=self._refresh_loop, daemon=True)
        self._refresh_thread.start()
        self.__console.debug("Started fallback rendering refresh thread")
    
    def _refresh_loop(self):
        """Fallback periodic refresh loop"""
        refresh_interval = 1.0 / 30.0  # 30 FPS refresh rate
        
        while self._refresh_running:
            try:
                # Process pending messages
                self._process_pending_messages()
                
                # Force redraw
                if hasattr(self, '_window') and self._window:
                    self._window.post_redraw()
                
                time.sleep(refresh_interval)
                
            except Exception as e:
                self.__console.error(f"Error in refresh loop: {e}")
                time.sleep(0.1)
    
    def _stop_refresh_thread(self):
        """Stop the rendering refresh thread"""
        if hasattr(self, '_refresh_running'):
            self._refresh_running = False
        if hasattr(self, '_refresh_thread') and self._refresh_thread and self._refresh_thread.is_alive():
            self._refresh_thread.join(timeout=1.0)
        self.__console.debug("Stopped rendering refresh thread")

    def show_initial_geometry(self, config:dict):
        """ Show initial geometry """
        self.on_show_origin_coord(show=config.get("show_origin", False))
        self.on_show_urdf(show=config.get("show_robot", False))
        self.on_show_floor(show=config.get("show_floor", False))

    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
        try:
            if len(multipart_data) >= 2:
                with self._message_lock:
                    # Check if queue is full
                    if len(self._pending_messages) >= self._pending_messages.maxlen:
                        self._message_stats['dropped'] += 1
                        # Optionally log when dropping messages frequently
                        if self._message_stats['dropped'] % 50 == 0:
                            self.__console.warning(f"Dropped {self._message_stats['dropped']} messages due to processing lag")
                    
                    # Add to queue (deque will automatically drop oldest if at maxlen)
                    self._pending_messages.append(multipart_data)
                    self._message_stats['received'] += 1
                    
        except Exception as e:
            self.__console.error(f"({self.__class__.__name__}) Error processing received data: {e}")
    
    def __process_pending_data(self):
        """Process pending data in the main GUI thread"""
        try:
            if hasattr(self, '__pending_data') and self.__pending_data:
                self.__zpipe_msg_process(self.__pending_data)
                self._window.post_redraw()  # Redraw the GUI after processing the message
                self.__pending_data = None
        except Exception as e:
            self.__console.error(f"({self.__class__.__name__}) Error processing pending data: {e}")
    
    def __on_status_received(self, status:dict):
        """Callback function for zpipe status reception"""
        self.__console.debug(f"({self.__class__.__name__}) Received status: {status}")

    def __zpipe_msg_process(self, multipart_data):
        if len(multipart_data) < 2:
            self.__console.error(f"({self.__class__.__name__}) Invalid multipart data received")
            return

        topic = multipart_data[0]
        msg = multipart_data[1]

        if topic.decode() == "call":
            msg_decoded = json.loads(msg.decode('utf8').replace("'", '"'))
            try:
                function_name = msg_decoded["function"]
                function = getattr(super(), function_name)
                kwargs = msg_decoded["kwargs"]
                function(self._scene, **kwargs)
                
                # Force immediate redraw after geometry update
                self._window.post_redraw()
                
                # Also force scene update if possible
                if hasattr(self._scene, 'force_redraw'):
                    self._scene.force_redraw()
            except json.JSONDecodeError as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")
            except Exception as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")

    def on_close(self):
        """ close window and stop all """
        # Stop refresh thread and clear pending messages
        self._stop_refresh_thread()
        
        # Clear pending messages
        if hasattr(self, '_message_lock') and hasattr(self, '_pending_messages'):
            with self._message_lock:
                self._pending_messages.clear()
        
        # Clean up subscriber socket
        if hasattr(self, '_Open3DVisualizer__socket') and self.__socket:
            self.__socket.destroy_socket()
            self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")
        
        # Clean up Open3D resources before quitting
        try:
            # Clear scene geometry
            if hasattr(self, '_scene') and self._scene:
                self._scene.scene.clear_geometry()
            
            # Properly shutdown GUI application
            gui.Application.instance.quit()
            
            # Force cleanup of Open3D resources
            import gc
            gc.collect()
            
        except Exception as e:
            self.__console.error(f"({self.__class__.__name__}) Error during cleanup: {e}")
        
        self.__console.debug(f"({self.__class__.__name__}) Closed Window")
        return True
    
    def on_key_event(self, event):
        """ key event"""
        pass
        # if event.type == gui.KeyEvent.DOWN and event.key == gui.KeyName.O:
        #    self.__show_origin_coord = not self.__show_origin_coord
        #    self.on_show_origin_coord()
        #    return True
        # elif event.type == gui.KeyEvent.DOWN and event.key == gui.KeyName.U:
        #     self.on_show_urdf()
        #     return True
        # return False

    def run(self):
        gui.Application.instance.run()

    def on_show_origin_coord(self, show:bool):
        """ Show origin coordinate """
        if show:
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
            material = rendering.MaterialRecord()
            material.shader = "defaultLit"
            self._scene.scene.add_geometry("origin_frame", frame, material)
        else:
            self._scene.scene.remove_geometry("origin_frame")

    def on_show_urdf(self, show:bool):
        """ show robot URDF model"""
        if show:
            for urdf in self.__config["urdf"]:
                name = urdf["name"]
                urdf_file = os.path.join(self.__config["root_path"], urdf["path"])
                self.__console.debug(f"Loading URDF {name} from {urdf_file}")

                # load Robot URDF Model
                robot = URDF.load(urdf_file, lazy_load_meshes=True)

                # Get base position and orientation from config
                base_pos = urdf.get("base", [0.0, 0.0, 0.0])[0:3]
                base_ori = np.deg2rad(np.array(urdf.get("base", [0.0, 0.0, 0.0])[3:6]))

                # Use API_add_urdf to add the robot to the scene
                self.API_add_urdf(self._scene, name, robot, base_pos, base_ori)
        else:
            pass

    def on_show_floor(self, show:bool):
        if show:
            config_floor = self.__config.get("floor", {"name":"floor", "size":[5.0, 5.0, 0.0]})
            width = config_floor.get("size", [5.0, 5.0, 0.0])[0]
            depth = config_floor.get("size", [5.0, 5.0, 0.0])[1]
            height = config_floor.get("size", [5.0, 5.0, 0.0])[2]
            
            # draw floor(box)
            floor = o3d.geometry.TriangleMesh.create_box(width, depth, height)
            floor.compute_vertex_normals()

            # move floor to z=0
            floor.translate((0, 0, -height))

            # set color
            floor.paint_uniform_color(config_floor.get("color", [0.1, 0.1, 0.1]))

            # material
            material = rendering.MaterialRecord()
            material.shader = "defaultLit"
            self._scene.scene.add_geometry(config_floor.get("name", "floor"), floor, material)

        else:
            pass
            # self._scene.scene.remove_geometry(config_floor.get("name", "floor"))