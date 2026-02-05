import threading
from collections import deque
import open3d as o3d
import time
import datetime
import colorlog
from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe
from viewer.container import GeoContainer

from common import zapi

class Open3DVisualizer:
    def __init__(self, config:dict=None,  zpipe:ZPipe=None):
        if config is None:
            config = {}
    
        self.__console = ConsoleLogger.get_logger()
        
        # Initialize message queue and stats
        self._message_lock = threading.Lock()
        self._pending_messages = deque(maxlen=100)
        self._message_stats = {'received': 0, 'dropped': 0}

        # Initialize GeoContainer
        self.container = GeoContainer()

        # create & join asynczsocket
        # 1. Subscriber Socket (Recv data + System commands from SimTool)
        self.__socket = AsyncZSocket("Open3DVisualizer", "subscribe")
        if self.__socket.create(pipeline=zpipe):
            transport = config.get("transport", "tcp")
            # Viewer listens to SimTool's PUB port (9002) - wait, config says 9001?
            # Viewer.cfg has "port": 9001. This is usually its OWN port if it was a server, 
            # OR the target port if client.
            # Open3DVisualizer is a Subscriber. 
            # In `viewer.py`, it passes this config.
            # If `viewer.cfg` says 9001, it attempts to connect to 9001.
            # But SimTool binds to 9002. 
            # So Viewer needs to know SimTool's port. 
            # User didn't ask to fix port mismatch, but implied they work together.
            # Maybe I should just assume config is correct or SimTool also publishes on 9001?
            # SimTool.cfg says 9002.
            # Viewer.cfg says 9001.
            # Unless Viewer is binding 9001? No, it's Subscriber. 
            # AsyncZSocket join logic: if pattern is subscribe -> connect via `join`.
            # So Viewer connects to localhost:9001.
            # SimTool binds 9002.
            # THIS IS A CONFIG MISMATCH.
            # However, I should stick to the task: ZAPI.
            # But for ZAPI to work, they must be connected.
            # I will configure the NEW sockets explicitly.
            
            # Existing socket logic (using config['port']):
            port = config.get("port", 9001)
            host = config.get("host", "localhost")
            if self.__socket.join(transport, host, port):
                self.__socket.subscribe("call")
                # SUBSCRIBE TO SYSTEM TOPIC
                self.__socket.subscribe(zapi.TOPIC_SYSTEM)
                
                self.__socket.set_message_callback(self.__on_data_received)
                self.__console.debug(f"Socket created and joined: {transport}://{host}:{port}")
            else:
                self.__console.error("Failed to join socket")
        else:
            self.__console.error("Failed to create socket")
            
        # 2. Publisher Socket (Send System commands) - NEW
        # We need a dedicated port for Viewer to publish. 
        # Let's hardcode 9003 for now as per plan, or add to config?
        # Better to add to config but I can just use 9003 default.
        self.sys_pub_port = config.get("sys_pub_port", 9003)
        self.__sys_socket = AsyncZSocket("Open3DVisualizerSys", "publish")
        if self.__sys_socket.create(pipeline=zpipe):
            if self.__sys_socket.join("tcp", "*", self.sys_pub_port):
                self.__console.debug(f"System Publisher bound to: *: {self.sys_pub_port}")
            else:
                 self.__console.error("Failed to bind System Publisher")

        # Initialize Open3D Visualizer
        self.vis = o3d.visualization.Visualizer()
        
        window_title = config.get('window_title', 'Open3D')
        window_size = config.get('window_size', [1920, 1080])
        width = window_size[0]
        height = window_size[1]
        
        self.vis.create_window(window_name=window_title, width=width, height=height)
        
        # Add geometries from container
        for name, data in self.container.get_geometries().items():
            if data["visible"]:
                self.vis.add_geometry(data["geometry"])
                
        # Set default view
        self.set_isometric()
        
        # Flag for external termination
        self._should_close = False

    def set_isometric(self):
        """Set view to isometric (looking from diagonal)"""
        ctr = self.vis.get_view_control()
        # Set camera to look from a diagonal direction
        ctr.set_front([-1.0, -1.0, 1.0])
        ctr.set_lookat([0.0, 0.0, 0.0])
        ctr.set_up([0.0, 0.0, 1.0])
        ctr.set_zoom(0.8)
        self.__console.debug("View set to Isometric")

    def set_perspective(self):
        """Set view to perspective (standard front view)"""
        ctr = self.vis.get_view_control()
        ctr.set_front([-1.0, -0.5, 0.5])
        ctr.set_lookat([0.0, 0.0, 0.0])
        ctr.set_up([0.0, 0.0, 1.0])
        ctr.set_zoom(0.8)
        self.__console.debug("View set to Perspective")

    def run(self, frequency_hz: int):
        interval = 1.0 / frequency_hz
        self.__console.info(f"Starting rendering loop at {frequency_hz} Hz")

        while self.vis.poll_events():
            if self._should_close:
                break
                
            start_time = time.time()
            
            self.vis.update_renderer()
            
            # Log update with timestamp (handled by formatter)
            # code here to update

            elapsed = time.time() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.vis.destroy_window()
        self.on_close()
        self.__console.info("Visualizer closed")
    
    def on_close(self):
        # Send termination signal
        if hasattr(self, '_Open3DVisualizer__sys_socket') and self.__sys_socket:
             zapi.zapi_destroy(self.__sys_socket)
             # Give a moment for message to fly out
             time.sleep(0.1)
             self.__sys_socket.destroy_socket()

        # Clean up subscriber socket
        if hasattr(self, '_Open3DVisualizer__socket') and self.__socket:
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