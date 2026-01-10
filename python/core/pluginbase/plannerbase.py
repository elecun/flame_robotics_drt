from abc import ABC, abstractmethod
import numpy as np
from typing import List, Union, Optional

class PlannerBase(ABC):
    """
    Abstract base class for path planning algorithms.
    """

    def __init__(self):
        self.static_objects = []

    @abstractmethod
    def generate(self, current_pose: Union[List[float], np.ndarray], target_pose: Union[List[float], np.ndarray]) -> List[np.ndarray]:
        """
        Generate a path from current_pose to target_pose.
        
        Args:
            current_pose: Current end-effector pose [x, y, z, roll, pitch, yaw]
            target_pose: Target end-effector pose [x, y, z, roll, pitch, yaw]
                         Orientation components can be NaN (don't care).
                         
        Returns:
            List of waypoints (numpy arrays).
        """
        pass

    @abstractmethod
    def add_static_object(self, object_model):
        """
        Add a static object to the planning environment.
        
        Args:
            object_model: The static object model (e.g., Open3D geometry).
        """
        pass
