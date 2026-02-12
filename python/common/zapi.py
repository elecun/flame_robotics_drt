"""
ZAPI (ZeroMQ API) Base Class
Provides common definitions and helper functions for system-level signaling
"""

import json
from common.zpipe import AsyncZSocket

class ZapiBase:
    def __init__(self):
        pass

    def call(self, socket: AsyncZSocket, function: str, kwargs: dict) -> bool:
        """
        Call a remote function via Zpipe socket using standardized multipart message format.
        Format: [socket_name, function, json_kwargs]
        
        Args:
            socket: AsyncZSocket instance
            function: Name of the function to call (str)
            kwargs: Dictionary of arguments
            
        Returns:
            bool: True if dispatched successfully
        """
        try:
            if socket:
                # Construct multipart message: [socket_name, function, json_kwargs]
                # socket_name is used to identify the sender or context if needed, 
                # but typically the receiver might need to know who sent it or just the function.
                # The user request said: "socket_name, function 이름, kwargs로 되는 dictionary를 multipart로 보내는 역할"
                
                socket_name = socket.socket_id
                
                parts = [
                    socket_name.encode('utf-8'),
                    function.encode('utf-8'),
                    json.dumps(kwargs).encode('utf-8')
                ]
                
                socket.dispatch(parts)
                return True
            else:
                return False
        except Exception:
            return False
