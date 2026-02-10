"""
VedoZApi - ZMQ Communication Module for VedoVisualizer
@note
- Manages all ZMQ socket communication in a separate thread
- Uses DEALER pattern for async request/reply
- Uses PUB pattern for system broadcast (terminate signals)
- Dispatches incoming messages to internal zapi_* handler methods
"""

import threading
import json
from collections import deque
from typing import Optional
from util.logger.console import ConsoleLogger
from common.zpipe import AsyncZSocket, ZPipe
from common import zapi as common_zapi


class VedoZApi:
    """ZMQ communication manager for VedoVisualizer.
    
    Runs a receiver thread that listens for incoming messages and
    dispatches them to zapi_* handler methods. Provides a request_queue
    for the visualizer to consume rendering commands from.
    """

    def __init__(self, config: dict = None, zpipe: ZPipe = None, visualizer=None):
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

        # --- Subscriber Socket (receive data + system commands) ---
        self.__sub_socket = AsyncZSocket("VedoZApi_Sub", "subscribe")
        if self.__sub_socket.create(pipeline=zpipe):
            transport = config.get("transport", "tcp")
            port = config.get("port", 9001)
            host = config.get("host", "localhost")
            if self.__sub_socket.join(transport, host, port):
                self.__sub_socket.subscribe("call")
                self.__sub_socket.subscribe(common_zapi.TOPIC_SYSTEM)
                self.__sub_socket.set_message_callback(self._on_message_received)
                self.__console.debug(f"[ZApi] Subscriber connected: {transport}://{host}:{port}")
            else:
                self.__console.error("[ZApi] Failed to join subscriber socket")
        else:
            self.__console.error("[ZApi] Failed to create subscriber socket")

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
        if self.__pub_socket and self.__pub_socket.is_joined:
            common_zapi.zapi_destroy(self.__pub_socket)

        # Destroy sockets
        if self.__sub_socket:
            self.__sub_socket.destroy_socket()
        if self.__pub_socket:
            self.__pub_socket.destroy_socket()

        self.__console.debug("[ZApi] Stopped and cleaned up")

    # ----------------------------------------------------------------
    # Message reception (called from AsyncZSocket's receiver thread)
    # ----------------------------------------------------------------
    def _on_message_received(self, multipart_data):
        """Callback from AsyncZSocket receiver thread."""
        try:
            if len(multipart_data) < 2:
                return

            topic = multipart_data[0]
            msg = multipart_data[1]

            # Check for system termination
            if common_zapi.zapi_check_system_message(topic, msg):
                self.__console.warning("[ZApi] Received TERMINATION signal")
                self.zapi_terminate()
                return

            # Dispatch to handler based on topic
            self._dispatch_message(topic, msg, multipart_data)

        except Exception as e:
            self.__console.error(f"[ZApi] Error processing message: {e}")

    def _dispatch_message(self, topic, msg, multipart_data):
        """Route incoming messages to the appropriate zapi_* handler.
        
        Message format expected: topic=b'call', msg=JSON with 'command' field.
        Falls back to queuing raw data if no specific handler matched.
        """
        try:
            # Decode topic
            topic_str = topic.decode('utf-8') if isinstance(topic, bytes) else topic

            if topic_str == "call":
                # Try to parse JSON command
                try:
                    msg_str = msg.decode('utf-8') if isinstance(msg, bytes) else msg
                    payload = json.loads(msg_str)
                    command = payload.get("command", "")

                    # Look up zapi_<command> handler
                    handler_name = f"zapi_{command}"
                    handler = getattr(self, handler_name, None)
                    if handler and callable(handler):
                        handler(payload)
                        return
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass  # Not JSON, fall through to raw queue

            # Default: push raw data to request_queue for visualizer
            with self._queue_lock:
                self.request_queue.append(multipart_data)

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

    def zapi_ping(self, payload=None):
        """Handle ping request — reply with pong via publisher."""
        self.__console.debug("[ZApi] Received ping, sending pong")
        if self.__pub_socket and self.__pub_socket.is_joined:
            self.__pub_socket.dispatch(["call", json.dumps({"response": "pong"})])

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
