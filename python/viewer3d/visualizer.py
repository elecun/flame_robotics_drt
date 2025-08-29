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
# from pykin.robots.robot import Robot

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
        self.__console = ConsoleLogger.get_logger() # logger
        gui.Application.instance.initialize() # gui

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

    def show_initial_geometry(self, config:dict):
        """ Show initial geometry """
        self.on_show_origin_coord(show=config.get("show_origin", False))
        self.on_show_urdf(show=config.get("show_robot", False))
        self.on_show_floor(show=config.get("show_floor", False))

    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
        try:
            if len(multipart_data) >= 2:
                self.__zpipe_msg_process(multipart_data)
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
                self._window.post_redraw()
            except json.JSONDecodeError as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")
            except Exception as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")

    def on_close(self):
        """ close window and stop all """
        # Clean up subscriber socket first
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