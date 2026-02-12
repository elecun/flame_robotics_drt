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
    def __init__(self, config:dict, zpipe):
        """ initialization """
        super().__init__()
        
        self.__console = ConsoleLogger.get_logger()
        self.__config = config
        self.zpipe = zpipe
        self.zapi = None

        try:            
            if "gui" in config:
                # load UI
                ui_path = pathlib.Path(config["app_path"]) / config["gui"]
                if os.path.isfile(ui_path):
                    loadUi(ui_path, self)
                    self.setWindowTitle(config.get("window_title", "DRT Simulation Tool"))

                    # Load plugins and samples
                    self.load_plugins()

                    # connect UI componens signals
                    self._connect_signals()

                else:
                    raise Exception(f"Cannot found UI file : {ui_path}")
            
            # Initialize ZAPI if not provided (though simtool.py passes zpipe instance, we need SimToolZApi instance)
            if self.zpipe:
                from simtool.zapi import ZAPI
                self.zapi = ZAPI(config, self.zpipe)
                self.zapi.set_message_callback(self._handle_message)
                self.zapi.run()
                self.__console.info("ZAPI started")
            else:
                 self.__console.error("ZPipe instance missing, ZAPI not started")
                
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
            else:
                self.__console.warning(f"Sample directory not found: {sample_path}")


    def _connect_signals(self):
        """Connect UI signals"""
        if hasattr(self, 'btn_load_spool'):
            self.btn_load_spool.clicked.connect(self.on_btn_load_spool_clicked)

    def on_btn_load_spool_clicked(self):
        """Handle Load Spool button click"""
        try:
            if hasattr(self, 'cbx_pipe_spool'):
                current_text = self.cbx_pipe_spool.currentText()
                if not current_text:
                    self.__console.warning("No spool file selected")
                    return
                
                # Resolve full path
                sample_path = pathlib.Path(self.__config.get("root_path", "")) / "sample" / current_text
                if sample_path.exists():
                    if self.zapi:
                        self.zapi.load_spool(str(sample_path.absolute()))
                        self.__console.info(f"Requested to load spool: {current_text}")
                    else:
                        self.__console.error("ZApi instance not available")
                else:
                    self.__console.error(f"Spool file not found: {sample_path}")
        except Exception as e:
            self.__console.error(f"Error loading spool: {e}")

    def _handle_message(self, topic, msg):
        """Handle incoming ZMQ messages"""
        try:
            # Decode if bytes
            if isinstance(topic, bytes):
                topic = topic.decode('utf-8')
            if isinstance(msg, bytes):
                msg = msg.decode('utf-8')
            
            # Check termination
            if zapi.zapi_check_system_message(topic, msg):
                 self.__console.warning("Received TERMINATION signal - Exiting...")
                 QApplication.quit()
                 return

            if topic == "call":
                try:
                    payload = json.loads(msg)
                    command = payload.get("command")
                    if command == "reply_load_spool":
                        path = payload.get("path")
                        status = payload.get("status")
                        if status == "success":
                            self.__console.info(f"Viewer successfully loaded spool: {path}")
                            QMessageBox.information(self, "Load Spool", f"Successfully loaded:\n{os.path.basename(path)}")
                        else:
                            self.__console.error(f"Viewer failed to load spool: {path}")
                            QMessageBox.warning(self, "Load Spool", f"Failed to load:\n{os.path.basename(path)}")
                except json.JSONDecodeError:
                    pass
        except Exception as e:
            self.__console.error(f"Error handling message: {e}")
    
    def closeEvent(self, event:QCloseEvent) -> None:
        """ Handle close event """
        try:
            # ZAPI cleanup
            if hasattr(self, 'zapi') and self.zapi:
                self.zapi.stop()
                self.__console.info("ZAPI stopped")

        except Exception as e:
            self.__console.error(f"Error during window close: {e}")
        finally:
            self.__console.info("Successfully Closed")
            super().closeEvent(event)