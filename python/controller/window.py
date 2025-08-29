'''
DRT Monitor Application Window class
@Author Byunghun Hwang<bh.hwang@iae.re.kr>
'''

try:
    # using PyQt6
    from PyQt6.QtGui import QImage, QPixmap, QCloseEvent, QStandardItem, QStandardItemModel
    from PyQt6.QtWidgets import QApplication, QFrame, QMainWindow, QLabel, QPushButton, QCheckBox, QComboBox, QDialog
    from PyQt6.QtWidgets import QMessageBox, QProgressBar, QFileDialog, QComboBox, QLineEdit, QSlider, QVBoxLayout
    from PyQt6.uic import loadUi
    from PyQt6.QtCore import QObject, Qt, QTimer, QThread, pyqtSignal, QRegularExpression
except ImportError:
    print("PyQt6 is required to run this application.")

import zmq
import os, sys
import pathlib
import json
import threading
import re
import numpy as np
import math
from functools import partial

from common.zpipe import AsyncZSocket, ZPipe
from common.urdf_parser import URDFParser
from util.logger.console import ConsoleLogger
from .geometry_model import GeometryTableModel  # geometry model handling
from .tcpview_model import TCPViewTableModel  # tcp view model handling

""" Global variables """
# DO not change this value unless necessary
JOINT_ANGLE_SCALE_FACTOR = 1000


class AppWindow(QMainWindow):
    def __init__(self, config:dict, zpipe:ZPipe):
        """ initialization """
        super().__init__()
        
        self.__console = ConsoleLogger.get_logger()
        self.__config = config

        # Initialize table models
        self.__geometry_model = GeometryTableModel()
        self.__tcpview_model = TCPViewTableModel()

        try:            
            if "gui" in config:
                # load UI
                ui_path = pathlib.Path(config["app_path"]) / config["gui"]
                if os.path.isfile(ui_path):
                    loadUi(ui_path, self)
                    self.setWindowTitle(config.get("window_title", "DRT Control Window"))

                    # Create socket for communication (as server)
                    self.__socket = AsyncZSocket("Controller", "publish")
                    if self.__socket.create(pipeline=zpipe):
                        transport = config.get("transport", "tcp")
                        port = config.get("port", 9001)
                        host = config.get("host", "localhost")
                        if self.__socket.join(transport, host, port):
                            self.__socket.set_message_callback(self.__on_data_received)
                            self.__console.debug(f"Socket created : {transport}://{host}:{port}")
                        else:
                            self.__console.error("Failed to join AsyncZsocket")
                    else:
                        self.__console.error("Failed to create AsyncZsocket")

                    # Initialize URDF parser and joint limits
                    self.urdf_parser = None
                    _rt_joint_limits = {}
                    _dda_joint_limits = {}
                    
                    # Load URDF and parse joint limits
                    if "urdf" in self.__config and self.__config["urdf"]:
                        for urdf_info in self.__config["urdf"]:
                            urdf_file = os.path.join(self.__config["root_path"], urdf_info["path"])
                            if os.path.exists(urdf_file):
                                parser = URDFParser(urdf_file)
                                if "dda" in urdf_info["name"]:
                                    _dda_joint_limits = parser.get_all_joint_limits()
                                    self.__console.info(f"Found {len(_dda_joint_limits)} joints with limits for DDA-side Manipulator")
                                elif "rt" in urdf_info["name"]:
                                    _rt_joint_limits = parser.get_all_joint_limits()
                                    self.__console.info(f"Found {len(_rt_joint_limits)} joints with limits for RT-side Manipulator")
                            else:
                                self.__console.warning(f"URDF file not found: {urdf_file}")

                    # menu actions
                    self.actionUtilGeneratePCD.triggered.connect(self.on_select_generate_pcd)

                    # Initialize sliders with URDF joint limits
                    self._setup_sliders(dda_side_joint_limits=_dda_joint_limits, rt_side_joint_limits=_rt_joint_limits)

                    # Setup geometry table
                    self.table_geometry.setModel(self.__geometry_model)
                    self.__geometry_model.geometryTransformChanged.connect(self.on_geometry_transform_changed)

                    # Setup TCP view table
                    self.table_tcpview.setModel(self.__tcpview_model)

                    
                    # Enable delete key functionality for geometry table
                    self.table_geometry.keyPressEvent = self.on_geometry_table_key_press
                    
                    # Button events
                    self.btn_open_pcd.clicked.connect(self.on_open_pcd)
                    self.btn_load_pcd.clicked.connect(self.on_load_pcd)
                    self.btn_run_simulation.clicked.connect(self.on_run_simulation)
                    self.btn_stop_simulation.clicked.connect(self.on_stop_simulation)
                    self.btn_geometry_remove_all.clicked.connect(self.on_btn_geometry_remove_all)

                else:
                    raise Exception(f"Cannot found UI file : {ui_path}")
                
        except Exception as e:
            self.__console.error(f"{e}")
    
    def closeEvent(self, event):
        """Handle window close event"""
        try:
            # Clear all geometry in viewer3d before closing
            self.__console.info("Clearing all geometry before closing controller window")
            self.__call(socket=self.__socket, function="API_clear_all_geometry", kwargs={})
            
            # Clear geometry table model
            self.__geometry_model.clear_all_geometry()
            
            # Close socket connection
            if hasattr(self, '_AppWindow__socket') and self.__socket:
                self.__socket.destroy_socket()
                self.__console.debug("Controller socket destroyed")
                
        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            # Accept the close event
            event.accept()
            self.__console.info("Controller window closed")

    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
        try:
            if len(multipart_data) >= 2:
                topic = multipart_data[0]
                msg = multipart_data[1]
                self.__call(topic, msg)
        except Exception as e:
            self.__console.error(f"({self.__class__.__name__}) Error processing received data: {e}")
    
    def closeEvent(self, event:QCloseEvent) -> None:
        """ Handle close event """
        try:
            # Clear all geometry in viewer3d before closing
            self.__console.info("Clearing all geometry before closing controller window")
            self.__call(socket=self.__socket, function="API_clear_all_geometry", kwargs={})

            # Clear geometry table model
            self.__geometry_model.clear_all_geometry()

            # Clean up subscriber socket first
            if hasattr(self, '_AppWindow__socket') and self.__socket:
                self.__socket.destroy_socket()
                self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")

        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            self.__console.info("Successfully Closed")
            return super().closeEvent(event)
    
    def on_open_pcd(self):
        """ Open PCD file dialog """
        pcd_file, _ = QFileDialog.getOpenFileName(self, "Open PCD File", "", "PCD Files (*.pcd);PLY Files (*.ply);All Files (*)")
        if pcd_file:
            self.__console.info(f"Selected PCD file: {pcd_file}")
            self.edit_pcd_file.setText(pcd_file)
        else:
            self.__console.warning("No PCD file selected.")
    
    def on_load_pcd(self):
        """ Load PCD file """
        self.__console.info(f"({self.__class__.__name__}) Loading PCD file")
        pcd_file = self.edit_pcd_file.text()
        pcd_name = os.path.basename(pcd_file)
        
        # Add to geometry table
        self.__geometry_model.add_geometry(pcd_name, pos=[0.0, 0.0, 0.0], ori=[0.0, 0.0, 0.0], geometry_type="pcd")
        
        # Send to visualizer
        self.__call(socket=self.__socket, function="API_add_pcd", kwargs={"name":pcd_name, "path":pcd_file, "pos":[0.0, 0.0, 0.0], "point_size":0.2})
    

    def on_run_simulation(self):
        """ Run Simulation """
        self.__console.info(f"({self.__class__.__name__}) Running Simulation")
        

    def on_stop_simulation(self):
        """ Stop Simulation """
        self.__console.info(f"({self.__class__.__name__}) Stop Simulation")

    def on_btn_geometry_remove_all(self):
        """ Clear all geometry """ 
        
        self.__console.info(f"({self.__class__.__name__}) Clear all geometry")
        
        # Clear geometry table
        self.__geometry_model.clear_all_geometry()
        
        # Send to visualizer
        self.__call(socket=self.__socket, function="API_clear_all_geometry", kwargs={})

    def on_slide_control_update(self, value, joint:str):
        slider = self.sender()
        self.findChild(QLineEdit, f"edit_{slider.objectName()}").setText(str(value/JOINT_ANGLE_SCALE_FACTOR))
        self.__call(socket=self.__socket, function="API_set_joint_angle", kwargs={"joint": joint, "value": value/JOINT_ANGLE_SCALE_FACTOR*3.14/180})
        self.__console.info(f"({self.__class__.__name__}) Joint {joint} value changed to {value/JOINT_ANGLE_SCALE_FACTOR*3.14/180}")

    def on_geometry_transform_changed(self, name: str, position: list, orientation: list):
        """Handle geometry transform changes from table"""
        self.__console.info(f"Geometry {name} transform changed: pos={position}, ori={orientation}")
        self.__call(socket=self.__socket, function="API_update_geometry_transform", kwargs={"name": name, "pos": position, "ori": orientation})

    def on_geometry_table_key_press(self, event):
        """Handle key press events for geometry table"""
        from PyQt6.QtCore import Qt
        
        if event.key() == Qt.Key.Key_Delete:
            # Get selected indexes
            selected_indexes = self.table_geometry.selectionModel().selectedIndexes()
            if selected_indexes:
                # Delete from model and get deleted names
                deleted_names = self.__geometry_model.delete_selected_geometry(selected_indexes)
                for name in deleted_names:
                    self.__call(socket=self.__socket, function="API_remove_geometry", kwargs={"name": name})
                
                self.__console.info(f"Deleted {len(deleted_names)} geometry objects")
        else:
            # Call the original keyPressEvent for other keys
            super(type(self.table_geometry), self.table_geometry).keyPressEvent(event)

    def _setup_sliders(self, dda_side_joint_limits:dict, rt_side_joint_limits:dict):
        """Setup sliders with URDF joint limits"""
        try:
            # RT-side Manipulator joint control
            rt_labels = self.findChildren(QLabel, QRegularExpression(r"label_rt_\d+"))
            rt_edits = self.findChildren(QLineEdit, QRegularExpression(r"edit_slide_rt_\d+"))
            rt_sliders = self.findChildren(QSlider, QRegularExpression(r"slide_rt_\d+"))
            rt_labels.sort(key=lambda lbl: int(re.search(r"\d+", lbl.objectName()).group()))
            rt_edits.sort(key=lambda lbl: int(re.search(r"\d+", lbl.objectName()).group()))
            rt_sliders.sort(key=lambda lbl: int(re.search(r"\d+", lbl.objectName()).group()))

            for label, key in zip(rt_labels, rt_side_joint_limits.keys()):
                label.setText(key)
            for label, slider, key in zip(rt_labels, rt_sliders, rt_side_joint_limits.keys()):
                # slider.valueChanged.connect(lambda v: self.on_slide_control_update(v, key))
                slider.valueChanged.connect(partial(self.on_slide_control_update, joint=key))
                

                limits = rt_side_joint_limits[label.text()]
                lower_deg = int(limits['lower'] * 180 / 3.14)*JOINT_ANGLE_SCALE_FACTOR
                upper_deg = int(limits['upper'] * 180 / 3.14)*JOINT_ANGLE_SCALE_FACTOR
                slider.setRange(lower_deg, upper_deg)
                slider.setValue(0)
                self.findChild(QLineEdit, f"edit_{slider.objectName()}").setText(str(0))
        except Exception as e:
            self.__console.warning(f"{e}")

        try:
            # DDA-side Manipulator joint control
            dda_labels = self.findChildren(QLabel, QRegularExpression(r"label_dda_\d+"))
            dda_edits = self.findChildren(QLineEdit, QRegularExpression(r"edit_slide_dda_\d+"))
            dda_sliders = self.findChildren(QSlider, QRegularExpression(r"slide_dda_\d+"))
            dda_labels.sort(key=lambda lbl: int(re.search(r"\d+", lbl.objectName()).group()))
            dda_edits.sort(key=lambda lbl: int(re.search(r"\d+", lbl.objectName()).group()))
            dda_sliders.sort(key=lambda lbl: int(re.search(r"\d+", lbl.objectName()).group()))

            for label, key in zip(dda_labels, dda_side_joint_limits.keys()):
                label.setText(key)
            for label, slider, key in zip(dda_labels, dda_sliders, dda_side_joint_limits.keys()):
                # slider.valueChanged.connect(lambda v: self.on_slide_control_update(v, key))
                slider.valueChanged.connect(partial(self.on_slide_control_update, joint=key))

                limits = dda_side_joint_limits[label.text()]
                lower_deg = int(limits['lower'] * 180 / 3.14)*JOINT_ANGLE_SCALE_FACTOR
                upper_deg = int(limits['upper'] * 180 / 3.14)*JOINT_ANGLE_SCALE_FACTOR
                slider.setRange(lower_deg, upper_deg)
                slider.setValue(0)
                self.findChild(QLineEdit, f"edit_{slider.objectName()}").setText(str(0))
        except Exception as e:
            self.__console.warning(f"{e}")

        

    def on_select_generate_pcd(self):
        """ 3D Model to PCD file generation """
        from python.manager.util_gen_pcd import AppWindow as GenPCDWindow
        model_file, _ = QFileDialog.getOpenFileName(self, "Open STL File", "", "STL Files (*.stl);;All Files (*)")
        if model_file:
            wnd = GenPCDWindow(config=self.__config, target=model_file)
            wnd.show()

            self.__console.info(f"Selected 3D model file: {model_file}")
        
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

    


        


