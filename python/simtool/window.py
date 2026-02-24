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

import os, sys
import pathlib
import json
import importlib
import inspect

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
                    self.__load_plugins()

                    # connect UI componens signals
                    self.__connect_signals()

                else:
                    raise Exception(f"Cannot found UI file : {ui_path}")
            
            # Initialize ZAPI and Start ZAPI
            if self.zpipe:
                from simtool.zapi import ZAPI
                zapi_config = self.__config.get("zapi", {})
                transport = zapi_config.get("transport", "ipc")
                channel = zapi_config.get("channel", "/tmp/viewervedo")
                
                self.zapi = ZAPI(zpipe=self.zpipe, transport=transport, channel=channel)
                self.zapi.signal_message_received.connect(self._handle_message)
                self.zapi.run()
                self.__console.info("Now ZAPI is running for SimTool")
            else:
                 self.__console.error("ZPipe instance missing, ZAPI not started")
                
        except Exception as e:
            self.__console.error(f"{e}")

    def __load_plugins(self):
        """
        Load plugins and populate comboboxes
        """
        
        # Load Path Planners and Optimizers
        plugin_categories = [
            {
                "name": "PathPlanner",
                "path": "python/plugins/pathplanner",
                "package_prefix": "plugins.pathplanner",
                "base_class": PlannerBase,
                "combobox": getattr(self, 'cbx_plugin_pathplanner', None)
            },
            {
                "name": "Optimizer",
                "path": "python/plugins/optimizer",
                "package_prefix": "plugins.optimizer",
                "base_class": OptimizerBase,
                "combobox": getattr(self, 'cbx_plugin_optimizer', None)
            }
        ]

        for category in plugin_categories:
            combobox = category["combobox"]
            
            if combobox is not None:
                try:
                    combobox.clear()
                    
                    root_path = self.__config.get("root_path", "")
                    plugin_path = pathlib.Path(root_path) / category["path"]
                    
                    if plugin_path.exists():
                        files = list(plugin_path.glob("*.py"))
                        
                        for file_path in files:
                            if file_path.name == "__init__.py":
                                continue
                            module_name = f"{category['package_prefix']}.{file_path.stem}"
                            try:
                                module = importlib.import_module(module_name)
                                for name, obj in inspect.getmembers(module):
                                    if inspect.isclass(obj) and issubclass(obj, category["base_class"]): 
                                        if obj is not category["base_class"]:
                                            self.__console.info(f"Found plugin class: {obj.__name__}") # LOG INFO
                                            combobox.addItem(obj.__name__)
                            except Exception as e:
                                self.__console.error(f"Failed to load {category['name']} plugin {module_name}: {e}")
                    else:
                        self.__console.warning(f"{category['name']} plugin directory not found: {plugin_path}")

                except Exception as e:
                    import traceback
                    self.__console.error(f"Error processing category {category['name']}: {e}")
                    self.__console.error(traceback.format_exc())

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


    def __connect_signals(self):
        """Connect UI signals"""
        if hasattr(self, 'btn_load_spool'):
            self.btn_load_spool.clicked.connect(self.__on_btn_load_spool_clicked)
        if hasattr(self, 'btn_test_async_zapi_request'):
            self.btn_test_async_zapi_request.clicked.connect(self.on_btn_test_async_zapi_request_clicked)

    def on_btn_test_async_zapi_request_clicked(self):
        """Handle async ZAPI test request button click"""
        pass

    def __on_btn_load_spool_clicked(self):
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
                        self.zapi._ZAPI_request_load_spool(str(sample_path.absolute()))
                        self.__console.info(f"Requested to load spool: {current_text}")
                    else:
                        self.__console.error("ZAPI instance not available")
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