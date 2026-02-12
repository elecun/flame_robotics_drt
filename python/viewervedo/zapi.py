"""
Zapi - ZMQ Communication Module for Visualizer
@note
- Manages all ZMQ socket communication in a separate thread
- Uses ROUTER pattern for async request/reply
- Dispatches incoming messages to internal zapi_* handler methods
"""

import threading
import json
from collections import deque
from typing import Optional
from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe
from common.zapi import ZapiBase


class Zapi(ZapiBase):
    """ZMQ communication manager for Visualizer.
    
    Runs a receiver thread that listens for incoming messages and
    dispatches them to zapi_* handler methods. Provides a request_queue
    for the visualizer to consume rendering commands from.
    """

    def __init__(self, config: dict = None, zpipe: ZPipe = None, visualizer=None):
        super().__init__()
        if config is None:
            config = {}

        self.__console = ConsoleLogger.get_logger()
        self._visualizer = visualizer
        self._zpipe = zpipe
        self._running = False
        self._thread = None

        # Thread-safe queue: zapi handlers push commands here,
        # visualizer polls from it each frame
        self.request_queue = deque(maxlen=100)
        self._queue_lock = threading.Lock()

        # --- Router Socket (receive data + system commands) ---
        self.__router_socket = AsyncZSocket("Zapi_Router", "router")
        if self.__router_socket.create(pipeline=zpipe):
            transport = config.get("transport", "tcp")
            port = config.get("port", 9001)
            # ROUTER is a server, so it should bind. 
            # If transport is tcp, host should be "*" or "0.0.0.0" to bind to all interfaces.
            # If user config provides a specific host (e.g. localhost), it binds to that.
            # Ensure we bind to valid address. If config has 'localhost', bind to 127.0.0.1 or localhost.
            host = config.get("host", "*") 
            # If config has '0.0.0.0' or '*', bind to all.
            if host == "*": host = "0.0.0.0" # Some systems prefer 0.0.0.0 for binding all
            
            if self.__router_socket.join(transport, host, port):
                self.__router_socket.set_message_callback(self._on_message_received)
                self.__console.debug(f"[ZApi] Router bound: {transport}://{host}:{port}")
            else:
                self.__console.error("[ZApi] Failed to bind router socket")
        else:
            self.__console.error("[ZApi] Failed to create router socket")

        # --- Publisher Socket (send system commands) ---
        self._sys_pub_port = config.get("sys_pub_port", 9003)
        self.__pub_socket = AsyncZSocket("VedoZApi_Pub", "publish")
        if self.__pub_socket.create(pipeline=zpipe):
            if self.__pub_socket.join("tcp", "*", self._sys_pub_port):
                self.__console.debug(f"[ZApi] System Publisher bound to port {self._sys_pub_port}")
            else:
                self.__console.error("[ZApi] Failed to bind System Publisher")

    def run(self):
        """Start the ZApi communication thread."""
        if self._running:
            self.__console.warning("[ZApi] Already running")
            return

        self._running = True
        self.__console.debug("[ZApi] Communication thread started")

    def stop(self):
        """Stop the ZApi communication thread and cleanup sockets."""
        self._running = False

        # Send termination signal to other processes
        # common_zapi.zapi_destroy(self.__pub_socket) - function removed.
        # Use call? System messages might need to be adapted or just commented out for now as requested "delete unused code".
        pass

        # Destroy sockets
        if self.__router_socket:
            self.__router_socket.destroy_socket()
        if self.__pub_socket:
            self.__pub_socket.destroy_socket()

        self.__console.debug("[ZApi] Stopped and cleaned up")

    # ----------------------------------------------------------------
    # Message reception (called from AsyncZSocket's receiver thread)
    # ----------------------------------------------------------------
    def _on_message_received(self, multipart_data):
        """Callback from AsyncZSocket receiver thread.
        ROUTER socket receives: [identity, socket_name, function, json_kwargs]
        Why?
        Sender (DEALER) calls ZapiBase.call -> sends [socket_name, function, json_kwargs]
        ROUTER receives -> [identity, socket_name, function, json_kwargs]
        """
        try:
            if len(multipart_data) < 4:
                # Minimum expected for ROUTER receiving from ZapiBase.call:
                # [identity, socket_name, function, json_kwargs] = 4 parts
                self.__console.warning(f"[ZApi] Received incomplete message: {len(multipart_data)} parts")
                return

            identity = multipart_data[0]
            socket_name = multipart_data[1]
            function_name = multipart_data[2]
            json_kwargs = multipart_data[3]

            # Decode
            socket_name_str = socket_name.decode('utf-8') if isinstance(socket_name, bytes) else socket_name
            function_name_str = function_name.decode('utf-8') if isinstance(function_name, bytes) else function_name
            
            # Dispatch
            self._dispatch_message(identity, function_name_str, json_kwargs)

        except Exception as e:
            self.__console.error(f"[ZApi] Error processing message: {e}")

    def _dispatch_message(self, identity, function_name, json_kwargs):
        """Route incoming messages to the appropriate zapi_* handler.
        
        Args:
            identity: Sender identity (bytes)
            function_name: Function name (str)
            json_kwargs: JSON encoded kwargs (bytes/str)
        """
        try:
            # Parse kwargs
            try:
                kwargs_str = json_kwargs.decode('utf-8') if isinstance(json_kwargs, bytes) else json_kwargs
                kwargs = json.loads(kwargs_str)
                
                if not isinstance(kwargs, dict):
                    kwargs = {"payload": kwargs} # Fallback if not dict

                # Inject identity into kwargs so we know who to reply to
                kwargs["_identity"] = identity

                # Look up zapi_<function> handler
                handler_name = f"zapi_{function_name}"
                handler = getattr(self, handler_name, None)
                
                if handler and callable(handler):
                    handler(kwargs)
                    return
                else:
                    self.__console.warning(f"[ZApi] Unknown function: {function_name}")
                    
            except (json.JSONDecodeError, UnicodeDecodeError):
                self.__console.error(f"[ZApi] Failed to parse kwargs: {json_kwargs}")

        except Exception as e:
            self.__console.error(f"[ZApi] Dispatch error: {e}")

    # ----------------------------------------------------------------
    # zapi_* handler functions
    # ----------------------------------------------------------------
    def zapi_terminate(self, payload=None):
        """Handle termination request."""
        if self._visualizer:
            self._visualizer._should_close = True
        self.__console.info("[ZApi] Termination requested")

    def zapi_ping(self, kwargs=None):
        """Handle ping request — reply with pong via router."""
        self.__console.debug("[ZApi] Received ping, sending pong")
        if self.__router_socket and self.__router_socket.is_joined:
            identity = kwargs.get("_identity") if kwargs else None
            # Standardize reply format? Or keep 'command':'pong'?
            # Let's use new format for reply too: function='pong', kwargs={}
            if identity:
                 # Manually dispatch for ROUTER reply because ZapiBase.call sends [socket_name, func, kwargs]
                 # But ROUTER needs [identity, socket_name, func, kwargs]
                 
                 # Construct payload
                 socket_name = self.__router_socket.socket_id
                 function = "pong"
                 kwargs_reply = {}
                 
                 reply_parts = [
                     identity,
                     socket_name.encode('utf-8'),
                     function.encode('utf-8'),
                     json.dumps(kwargs_reply).encode('utf-8')
                 ]
                 self.__router_socket.dispatch(reply_parts)

    def zapi_load_spool(self, kwargs=None):
        """Handle load_spool request."""
        if kwargs and "path" in kwargs:
            self.__console.info(f"[ZApi] Received load_spool request: {kwargs['path']}")
            self.push_to_queue(kwargs)
        else:
            self.__console.warning("[ZApi] Received load_spool request without path")

    def reply_load_spool(self, path: str, success: bool, identity=None):
        """Send a reply for load_spool request."""
        if self.__router_socket and self.__router_socket.is_joined and identity:
            kwargs = {
                "path": path,
                "status": "success" if success else "failed"
            }
            
            # Manual dispatch for ROUTER reply
            reply_parts = [
                identity,
                self.__router_socket.socket_id.encode('utf-8'),
                "reply_load_spool".encode('utf-8'),
                json.dumps(kwargs).encode('utf-8')
            ]
            self.__router_socket.dispatch(reply_parts)
            self.__console.info(f"[ZApi] Sent reply for {path}: {success}")

    # ----------------------------------------------------------------
    # Utility
    # ----------------------------------------------------------------
    def push_to_queue(self, data):
        """Push data to the request queue (thread-safe)."""
        with self._queue_lock:
            self.request_queue.append(data)

    def pop_from_queue(self):
        """Pop data from the request queue (thread-safe). Returns None if empty."""
        with self._queue_lock:
            if self.request_queue:
                return self.request_queue.popleft()
        return None
