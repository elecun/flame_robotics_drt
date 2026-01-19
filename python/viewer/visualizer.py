import threading
from collections import deque
import open3d as o3d
import time
import datetime
import colorlog
from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe
from viewer.container import GeoContainer

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
            start_time = time.time()
            
            self.vis.update_renderer()
            
            # Log update with timestamp (handled by formatter)
            self.__console.info("Main loop updated")

            elapsed = time.time() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.vis.destroy_window()
        self.on_close()
        self.__console.info("Visualizer closed")
    
    def on_close(self):
        # Clean up subscriber socket
        if hasattr(self, '_Open3DVisualizer__socket') and self.__socket:
            self.__socket.destroy_socket()
            self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")
        

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