'''
DRT Tool Window
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
import importlib
import inspect
import pkgutil
from common import zapi
from functools import partial

from common.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger
from plugins.pluginbase.plannerbase import PlannerBase
from plugins.pluginbase.optimizerbase import OptimizerBase


class AppWindow(QMainWindow):
    def __init__(self, config:dict, zpipe:ZPipe):
        """ initialization """
        super().__init__()
        
        self.__console = ConsoleLogger.get_logger()
        self.__config = config

        try:            
            if "gui" in config:
                # load UI
                ui_path = pathlib.Path(config["app_path"]) / config["gui"]
                if os.path.isfile(ui_path):
                    loadUi(ui_path, self)
                    self.setWindowTitle(config.get("window_title", "DRT Tool Window"))

                    # Load plugins and samples
                    self.load_plugins()

                    # Create socket for communication (as server)
                    self.__socket = AsyncZSocket("SimTool", "publish")
                    if self.__socket.create(pipeline=zpipe):
                        transport = config.get("transport", "tcp")
                        port = config.get("port", 9002) # SimTool binds 9002
                        host = config.get("host", "*")  # Binds to all
                        if self.__socket.join(transport, host, port):
                            self.__socket.set_message_callback(self.__on_data_received) # PUB usually doesn't recv but fine
                            self.__console.debug(f"Socket created : {transport}://{host}:{port}")
                        else:
                            self.__console.error("Failed to join AsyncZsocket")
                    else:
                        self.__console.error("Failed to create AsyncZsocket")
                        
                    # Create System Subscriber socket (Listen to Viewer on 9003)
                    self.__sys_sub = AsyncZSocket("SimToolSysSub", "subscribe")
                    if self.__sys_sub.create(pipeline=zpipe):
                        # Connect to Viewer
                        # Viewer binds 9003 on *
                        # We connect to localhost:9003 (assuming local)
                        sys_port = 9003
                        if self.__sys_sub.join("tcp", "localhost", sys_port):
                            self.__sys_sub.subscribe(zapi.TOPIC_SYSTEM)
                            # We use the SAME callback or a new one?
                            # Use same for simplicity, handle in __on_data_received
                            self.__sys_sub.set_message_callback(self.__on_data_received)
                            self.__console.debug(f"Connected to System Bus on port {sys_port}")
                        else:
                             self.__console.error("Failed to connect to System Bus")

                else:
                    raise Exception(f"Cannot found UI file : {ui_path}")
                
        except Exception as e:
            self.__console.error(f"{e}")

    def load_plugins(self):
        """
        Load plugins and populate comboboxes
        """
        # 1. Load Path Planners
        if hasattr(self, 'cbx_plugin_pathplanner'):
            self.cbx_plugin_pathplanner.clear()
            planner_path = pathlib.Path(self.__config.get("root_path", "")) / "python/plugins/pathplanner"
            if planner_path.exists():
                for file_path in planner_path.glob("*.py"):
                    if file_path.name == "__init__.py":
                        continue
                    module_name = f"plugins.pathplanner.{file_path.stem}"
                    try:
                        module = importlib.import_module(module_name)
                        for name, obj in inspect.getmembers(module):
                            if inspect.isclass(obj) and issubclass(obj, PlannerBase) and obj is not PlannerBase:
                                self.cbx_plugin_pathplanner.addItem(obj.__name__)
                                self.__console.debug(f"Loaded PathPlanner: {obj.__name__}")
                    except Exception as e:
                        self.__console.error(f"Failed to load planner plugin {module_name}: {e}")
            else:
                self.__console.warning(f"PathPlanner plugin directory not found: {planner_path}")

        # 2. Load Optimizers
        if hasattr(self, 'cbx_plugin_optimizer'):
            self.cbx_plugin_optimizer.clear()
            optimizer_path = pathlib.Path(self.__config.get("root_path", "")) / "python/plugins/optimizer"
            if optimizer_path.exists():
                for file_path in optimizer_path.glob("*.py"):
                    if file_path.name == "__init__.py":
                        continue
                    module_name = f"plugins.optimizer.{file_path.stem}"
                    try:
                        module = importlib.import_module(module_name)
                        for name, obj in inspect.getmembers(module):
                            if inspect.isclass(obj) and issubclass(obj, OptimizerBase) and obj is not OptimizerBase:
                                self.cbx_plugin_optimizer.addItem(obj.__name__)
                                self.__console.debug(f"Loaded Optimizer: {obj.__name__}")
                    except Exception as e:
                        self.__console.error(f"Failed to load optimizer plugin {module_name}: {e}")
            else:
                 self.__console.warning(f"Optimizer plugin directory not found: {optimizer_path}")

        # 3. Load Pipe Spool Samples
        if hasattr(self, 'cbx_pipe_spool'):
            self.cbx_pipe_spool.clear()
            sample_path = pathlib.Path(self.__config.get("root_path", "")) / "sample"
            if sample_path.exists():
                for file_path in sample_path.iterdir():
                    if file_path.suffix.lower() in ['.pcd', '.ply']:
                        self.cbx_pipe_spool.addItem(file_path.name)
                        self.__console.debug(f"Loaded Sample: {file_path.name}")
            else:
                self.__console.warning(f"Sample directory not found: {sample_path}")

    def __on_data_received(self, multipart_data):
        """Callback function for zpipe data reception"""
        try:
            if len(multipart_data) >= 2:
                topic = multipart_data[0]
                msg = multipart_data[1]
                
                # Check for Termination Signal
                if zapi.zapi_check_system_message(topic, msg):
                     self.__console.warning("Received TERMINATION signal - Exiting...")
                     QApplication.quit() # This will close windows and trigger cleanup
                     return
                     
                self.__call(topic, msg)
        except Exception as e:
            self.__console.error(f"({self.__class__.__name__}) Error processing received data: {e}")
    
    def closeEvent(self, event:QCloseEvent) -> None:
        """ Handle close event """
        try:
            # Broadcast termination
            if hasattr(self, '_AppWindow__socket') and self.__socket:
                zapi.zapi_destroy(self.__socket)
                # Give time for msg to be sent
                # zmq might handle it asynchronously, but a small process sleep helps ensure network flush
                import time
                time.sleep(0.1)

            # Clean up subscriber socket first
            if hasattr(self, '_AppWindow__socket') and self.__socket:
                self.__socket.destroy_socket()
                self.__console.debug(f"({self.__class__.__name__}) Destroyed socket")
            
            if hasattr(self, '_AppWindow__sys_sub') and self.__sys_sub:
                self.__sys_sub.destroy_socket()

        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            self.__console.info("Successfully Closed")
            super().closeEvent(event)