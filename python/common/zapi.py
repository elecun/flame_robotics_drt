"""
ZAPI (ZeroMQ API) for System Communications
Provides common definitions and helper functions for system-level signaling
"""

TOPIC_SYSTEM = "sys"
CMD_TERMINATE = "terminate"

def zapi_destroy(pub_socket):
    """
    Sends a system termination signal via the provided publisher socket.
    
    Args:
        pub_socket: AsyncZSocket instance with 'publish' pattern
    """
    if pub_socket and pub_socket.is_joined:
        pub_socket.dispatch([TOPIC_SYSTEM, CMD_TERMINATE])

def zapi_check_system_message(topic, msg):
    """
    Checks if a received message is a system termination signal.
    
    Args:
        topic: Received topic (bytes or str)
        msg: Received message part (bytes or str or list)
        
    Returns:
        bool: True if it is a termination signal
    """
    # Normalize inputs to strings if possible
    try:
        if isinstance(topic, bytes):
            topic = topic.decode('utf-8')
        if isinstance(msg, bytes):
            msg = msg.decode('utf-8')
        elif isinstance(msg, list):
            # If msg is a list (multipart), usually the first part is the payload if topic was separated
            # Check if any part matches CMD_TERMINATE
            for m in msg:
                if isinstance(m, bytes):
                     if m.decode('utf-8') == CMD_TERMINATE:
                         return True
                elif isinstance(m, str):
                     if m == CMD_TERMINATE:
                         return True
            return False
            
        if topic == TOPIC_SYSTEM and msg == CMD_TERMINATE:
            return True
            
    except Exception:
        pass
        
    return False
