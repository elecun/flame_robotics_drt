import threading
import json
import time
from collections import deque
from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe
from common import zapi as common_zapi
from common.zapi import ZapiBase

class ZAPI(ZapiBase):
    """ZMQ communication manager for SimTool.
    Uses DEALER socket to communicate with Viewer (Router).
    """

    def __init__(self, config: dict = None, zpipe: ZPipe = None):
        super().__init__()
        if config is None:
            config = {}

        self.__console = ConsoleLogger.get_logger()
        self._zpipe = zpipe
        self._running = False
        self._thread = None
        self._config = config
        
        # Callback for handling received messages from ZAPI -> Window
        self._message_callback = None

        # --- Dealer Socket ---
        self.__dealer_socket = AsyncZSocket("ZAPI_Dealer", "dealer")
        if not self.__dealer_socket.create(pipeline=zpipe):
            self.__console.error("[ZAPI] Failed to create dealer socket")

    def set_message_callback(self, callback):
        """Set callback to handle messages in the main application."""
        self._message_callback = callback

    def run(self):
        """Start the ZApi communication and connection loop."""
        if self._running:
             return
        self._running = True
        self.__console.debug("[ZAPI] Started")
        
        # Start connection monitor/retry thread
        self._thread = threading.Thread(target=self._connection_loop, daemon=True)
        self._thread.start()

    def _connection_loop(self):
        """Persistent connection loop."""
        transport = self._config.get("transport", "ipc")
        host = self._config.get("host", "/tmp/drt_zapi")
        port = self._config.get("port", 0) # Port ignored for IPC usually, or part of address
        
        while self._running:
            if not self.__dealer_socket.is_joined:
                # Try to join
                if self.__dealer_socket.join(transport, host, port):
                    self.__dealer_socket.set_message_callback(self._on_message_received)
                    self.__console.info(f"[ZAPI] Connected to {transport}://{host}")
                else:
                    self.__console.debug("[ZAPI] Connection attempt failed, retrying...")
                    time.sleep(1.0) # Retry interval
            else:
                # Already joined, just monitor or sleep
                # If IPC file deleted, ZMQ might not explicitly 'unjoin' in our wrapper state, 
                # but AsyncZSocket doesn't check liveness. 
                # We assume ZMQ handles reconnection.
                # If we wanted to detect disconnection, we'd need heartbeats.
                time.sleep(1.0)

    def stop(self):
        """Stop and cleanup."""
        self._running = False
        
        if self.__dealer_socket:
            self.__dealer_socket.destroy_socket()
        
        self.__console.debug("[ZAPI] Stopped")

    def _on_message_received(self, multipart_data):
        """Callback from sockets."""
        try:
            # Dealer receives: [socket_name, function, json_kwargs]
            # (Sent by Router via ZapiBase.call style reply or manual dispatch)
            
            if len(multipart_data) < 3:
                return

            socket_name = multipart_data[0]
            function_name = multipart_data[1]
            json_kwargs = multipart_data[2]
            
            # Decode components
            socket_str = socket_name.decode('utf-8') if isinstance(socket_name, bytes) else socket_name
            function_str = function_name.decode('utf-8') if isinstance(function_name, bytes) else function_name
            kwargs_str = json_kwargs.decode('utf-8') if isinstance(json_kwargs, bytes) else json_kwargs
            
            # Application Callback
            if self._message_callback:
                self._message_callback(function_str, kwargs_str)

        except Exception as e:
            self.__console.error(f"[ZAPI] Error receiving message: {e}")

    # ----------------------------------------------------------------
    # Commands
    # ----------------------------------------------------------------
    def load_spool(self, file_path: str):
        """Sends command to load spool."""
        if self.__dealer_socket and self.__dealer_socket.is_joined:
            kwargs = {
                "path": file_path
            }
            # Use ZapiBase.call
            self.call(self.__dealer_socket, "load_spool", kwargs)
            self.__console.info(f"[ZAPI] Sent load_spool request: {file_path}")
        else:
            self.__console.warning("[ZAPI] Cannot send load_spool: Socket not connected")
