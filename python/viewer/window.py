"""
3D Visualizer using pyvistaqt
author Byunghun Hwang <bh.hwang@iae.re.kr>
"""

try:
    # using PyQt6
    from PyQt6.QtGui import QImage, QPixmap, QCloseEvent, QStandardItem, QStandardItemModel
    from PyQt6.QtWidgets import QApplication, QFrame, QMainWindow, QLabel, QPushButton, QCheckBox, QComboBox, QDialog
    from PyQt6.QtWidgets import QMessageBox, QProgressBar, QFileDialog, QComboBox, QLineEdit, QSlider, QVBoxLayout
    from PyQt6.uic import loadUi
    from PyQt6.QtCore import QObject, Qt, QTimer, QThread, pyqtSignal, QRegularExpression
except ImportError:
    print("PyQt6 is required to run this application.")

import os
import pathlib
import numpy as np
from functools import partial
import trimesh
import time
import zmq
import json
import numpy as np
import threading
from pyvistaqt import QtInteractor

from util.logger.console import ConsoleLogger
from urdf_parser import URDF
from .geometry import geometryAPI
from math import pi, cos, sin
from pytransform3d import rotations, transformations
from pytransform3d.transformations import transform_from
from pytransform3d.rotations import matrix_from_euler
from common.zpipe import AsyncZSocket, ZPipe

class RenderWindow(QMainWindow, geometryAPI):

    def __init__(self, config:dict, zpipe:ZPipe):
        super().__init__()

        # initialize
        self.__config = config
        self.__console = ConsoleLogger.get_logger()
        self.__labels_visible = True  # Track label visibility state

        try:            
            if "gui" in config:
                # load UI
                ui_path = pathlib.Path(config["app_path"]) / config["gui"]
                if os.path.isfile(ui_path):
                    loadUi(ui_path, self)
                    self.setWindowTitle(config.get("window_title", "DRT 3D View Window"))

                    # using pyvistaqt
                    self.plotter = QtInteractor(self.widget_frame)
                    self.plotter.background_color = config.get("background-color", [0.9, 0.9, 0.9])
                    layout = self.widget_frame.layout()
                    if layout is None:
                        layout = QVBoxLayout(self.widget_frame)
                        layout.setContentsMargins(0, 0, 0, 0) # zero margin
                        layout.setSpacing(0)  # zero spacing
                        self.widget_frame.setLayout(layout)
                    else:
                        layout.setContentsMargins(0, 0, 0, 0)
                        layout.setSpacing(0)
                    layout.addWidget(self.plotter.interactor)

                    # Initialize geometry API
                    self.geometry_api = geometryAPI()
                    self.geometry_api.API_add_coord_frame(self.plotter, "origin", pos=[0,0,0], ori=[0,0,0], size=0.1)
                    self.geometry_api.API_add_ground(self.plotter, "ground", end_pos=config.get("ground", [5,5,0]), thickness=0.01)

                    # add urdf
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
                        self.API_add_urdf(self.plotter, name, robot, base_pos, base_ori)
                    
                    # Setup keyboard event handlers
                    self.plotter.add_key_event('1', self._on_key_1_pressed)
                    self.plotter.add_key_event('l', self._on_key_l_pressed)
                    
                    # Reset camera to show the coordinate frame
                    self.plotter.reset_camera()

                    # create & join asynczsocket
                    self.__socket = AsyncZSocket("RenderWindow", "subscribe")
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


                else:
                    raise Exception(f"Cannot found UI file : {ui_path}")
                
        except Exception as e:
            self.__console.error(f"{e}")

    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
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
                function(self.plotter, **kwargs)
                
            except json.JSONDecodeError as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")
            except Exception as e:
                self.__console.error(f"({self.__class__.__name__}) {e}")
    
    def __call(self, socket, function:str, kwargs:dict):
        """ call function via zpipe to others"""
        try:
            topic = "call"
            message = { "function":function,"kwargs":kwargs}
            if socket:
                socket.dispatch([topic, json.dumps(message)])
            else:
                self.__console.warning(f"Failed send")
            
        except zmq.ZMQError as e:
            self.__console.error(f"[Main Window] {e}")
        except json.JSONDecodeError as e:
            self.__console.error(f"[Main Window] JSON Decode Error: {e}")


    def closeEvent(self, event:QCloseEvent) -> None:
        """ Handle close event """
        try:
            # Clean up subscriber socket first
            if hasattr(self, '_RenderWindow__socket') and self.__socket:
                self.__socket.destroy_socket()
                self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")

        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            self.__console.info("Successfully Closed")
            return super().closeEvent(event)

    def _on_key_1_pressed(self):
        """Handle '1' key press - focus camera on ground center"""
        try:
            # Get ground center position (assuming ground extends from origin to [5,5,0])
            ground_center = [2.5, 2.5, 0]  # Center of 5x5 ground
            
            # Set camera to look at ground center from an elevated position
            self.plotter.camera_position = [
                (ground_center[0], ground_center[1] - 3, 3),  # Camera position (elevated and back)
                ground_center,  # Focal point (ground center)
                (0, 0, 1)  # Up vector (z-axis up)
            ]
            
            self.__console.info("Camera focused on ground center")
            
        except Exception as e:
            self.__console.error(f"Failed to focus camera on ground center: {e}")

    def _on_key_l_pressed(self):
        """Handle 'l' key press - toggle all geometry labels visibility"""
        try:
            self.__labels_visible = not self.__labels_visible
            
            # Get all geometry names
            geometry_names = self.geometry_api.API_list_geometries()
            
            for name in geometry_names:
                # Get actors for this geometry
                if name in self.geometry_api.geometry_container:
                    actors = self.geometry_api.geometry_container[name]
                    
                    # Find and toggle label actors (those with '_label' in name)
                    for actor in actors:
                        if hasattr(actor, 'name') and '_label' in str(actor.name):
                            actor.SetVisibility(self.__labels_visible)
                        elif hasattr(actor, 'GetMapper') and hasattr(actor.GetMapper(), 'GetInput'):
                            # Check if this is a text actor by looking for point labels
                            try:
                                mapper = actor.GetMapper()
                                if hasattr(mapper, 'GetInput'):
                                    input_data = mapper.GetInput()
                                    if hasattr(input_data, 'GetPointData'):
                                        point_data = input_data.GetPointData()
                                        if point_data.GetArrayName(0) and 'labels' in str(point_data.GetArrayName(0)).lower():
                                            actor.SetVisibility(self.__labels_visible)
                            except:
                                pass
            
            # Also toggle actor names that contain 'label'
            for actor_name in self.plotter.renderer.actors:
                if 'label' in actor_name.lower():
                    actor = self.plotter.renderer.actors[actor_name]
                    actor.SetVisibility(self.__labels_visible)
            
            status = "visible" if self.__labels_visible else "hidden"
            self.__console.info(f"All geometry labels are now {status}")
            
        except Exception as e:
            self.__console.error(f"Failed to toggle labels visibility: {e}")