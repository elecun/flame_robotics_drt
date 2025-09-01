"""
Robot Manipulation Module for Forward Kinematics Calculation
Provides forward kinematics computation for robot manipulators based on URDF configuration
@author: Byunghun Hwang (bh.hwang@iae.re.kr)
"""

import os
import numpy as np
from scipy.spatial import transform as rotations
try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
    print(f"Using pinocchio version {pin.__version__} for kinematics calculations")
except ImportError:
    print("pinocchio not available, falling back to URDF parser")
    print("To install pinocchio: pip install pin")
    PINOCCHIO_AVAILABLE = False

if not PINOCCHIO_AVAILABLE:
    from urdf_parser import URDF
    
from util.logger.console import ConsoleLogger


class Kinematics:
    """Module for robot manipulator forward kinematics calculations"""
    
    def __init__(self, config: dict):
        """
        Initialize manipulation module with configuration
        
        Args:
            config: Configuration dictionary containing URDF robot definitions
        """
        self.__console = ConsoleLogger.get_logger()
        self.__config = config
        self.__robots = {}  # Dict to store loaded robot models
        self.__joint_configs = {}  # Dict to store current joint configurations
        
        # Load robot models from config
        self._load_robots_from_config()
        
    def _load_robots_from_config(self):
        """Load robot models from configuration file"""
        if "urdf" not in self.__config:
            self.__console.warning("No URDF configuration found in config")
            return
            
        for urdf_config in self.__config["urdf"]:
            try:
                robot_name = urdf_config["name"]
                urdf_path = urdf_config["path"]
                base_transform = urdf_config.get("base", [0, 0, 0, 0, 0, 0])
                
                # Get absolute path
                if not os.path.isabs(urdf_path):
                    # Try multiple possible root paths
                    possible_roots = [
                        self.__config.get("root_path", ""),
                        os.getcwd(),
                        os.path.join(os.getcwd(), ".."),  # Go up one directory
                        os.path.dirname(os.path.dirname(os.path.dirname(__file__)))  # Go to project root
                    ]
                    
                    for root_path in possible_roots:
                        if root_path:
                            test_path = os.path.join(root_path, urdf_path)
                            if os.path.isfile(test_path):
                                urdf_path = test_path
                                break
                    
                    # If still not found, just use the original path
                    if not os.path.isfile(urdf_path):
                        urdf_path = os.path.join(os.getcwd(), urdf_path)
                
                # Create base transformation matrix
                base_pos = np.array(base_transform[0:3])
                base_ori = np.deg2rad(np.array(base_transform[3:6]))  # Convert degrees to radians
                
                base_R = rotations.Rotation.from_euler('xyz', base_ori).as_matrix()
                base_T = np.eye(4)
                base_T[:3, :3] = base_R
                base_T[:3, 3] = base_pos
                
                # Try to load with pinocchio first if available
                pin_success = False
                if PINOCCHIO_AVAILABLE:
                    try:
                        # Load robot model with pinocchio
                        pin_model = pin.buildModelFromUrdf(urdf_path)
                        pin_data = pin_model.createData()
                        
                        # Get joint names (excluding universe joint)
                        joint_names = []
                        for i in range(1, pin_model.njoints):  # Skip universe joint (index 0)
                            joint_name = pin_model.names[i]
                            joint_names.append(joint_name)
                        
                        # Store robot information with pinocchio
                        self.__robots[robot_name] = {
                            'pin_model': pin_model,
                            'pin_data': pin_data,
                            'type': 'pinocchio',
                            'base_transform': base_T,
                            'joint_names': joint_names,
                            'urdf_path': urdf_path
                        }
                        
                        self.__console.debug(f"Loaded robot {robot_name} with pinocchio - {len(joint_names)} joints: {joint_names}")
                        pin_success = True
                        
                    except Exception as e:
                        self.__console.error(f"Failed to load {robot_name} with pinocchio: {e}")
                        self.__console.info(f"Falling back to URDF parser for {robot_name}")
                
                # Fallback to URDF parser if pinocchio failed or not available
                if not pin_success:
                    robot = URDF.load(urdf_path, lazy_load_meshes=True)
                    
                    # Store robot information with URDF parser
                    self.__robots[robot_name] = {
                        'robot': robot,
                        'type': 'urdf',
                        'base_transform': base_T,
                        'joint_names': list(robot.actuated_joint_names),
                        'urdf_path': urdf_path
                    }
                    
                    self.__console.debug(f"Loaded robot {robot_name} with URDF parser - {len(robot.actuated_joint_names)} joints")
                
                # Initialize joint configuration to zero
                robot_info = self.__robots[robot_name]
                self.__joint_configs[robot_name] = {
                    joint_name: 0.0 for joint_name in robot_info['joint_names']
                }
                
                self.__console.debug(f"Loaded robot {robot_name} with {len(self.__joint_configs[robot_name])} joints")
                
            except Exception as e:
                self.__console.error(f"Failed to load robot {urdf_config.get('name', 'unknown')}: {e}")
    
    def get_robot_names(self):
        """Get list of available robot names"""
        return list(self.__robots.keys())
    
    def get_joint_names(self, robot_name: str):
        """Get joint names for a specific robot"""
        if robot_name not in self.__robots:
            self.__console.error(f"Robot {robot_name} not found")
            return []
        return self.__robots[robot_name]['joint_names'].copy()
    
    def set_joint_angles(self, robot_name: str, joint_angles: dict):
        """
        Set joint angles for a robot
        
        Args:
            robot_name: Name of the robot
            joint_angles: Dictionary with joint_name: angle_in_radians pairs
        """
        if robot_name not in self.__robots:
            self.__console.error(f"Robot {robot_name} not found")
            return False
            
        try:
            # Update joint configuration
            for joint_name, angle in joint_angles.items():
                if joint_name in self.__joint_configs[robot_name]:
                    self.__joint_configs[robot_name][joint_name] = float(angle)
                else:
                    self.__console.warning(f"Joint {joint_name} not found in robot {robot_name}")
            
            return True
            
        except Exception as e:
            self.__console.error(f"Failed to set joint angles for {robot_name}: {e}")
            return False
    
    def set_joint_angle(self, robot_name: str, joint_name: str, angle: float):
        """
        Set angle for a specific joint
        
        Args:
            robot_name: Name of the robot
            joint_name: Name of the joint
            angle: Joint angle in radians
        """
        if robot_name not in self.__robots:
            self.__console.error(f"Robot {robot_name} not found. Available robots: {list(self.__robots.keys())}")
            return False
            
        if joint_name not in self.__joint_configs[robot_name]:
            available_joints = list(self.__joint_configs[robot_name].keys())
            self.__console.error(f"Joint {joint_name} not found in robot {robot_name}. Available joints: {available_joints}")
            return False
            
        try:
            self.__joint_configs[robot_name][joint_name] = float(angle)
            self.__console.debug(f"Set {robot_name}.{joint_name} = {angle:.4f} rad")
            return True
        except Exception as e:
            self.__console.error(f"Failed to set joint angle for {robot_name}.{joint_name}: {e}")
            return False
    
    def get_joint_angles(self, robot_name: str):
        """Get current joint angles for a robot"""
        if robot_name not in self.__robots:
            self.__console.error(f"Robot {robot_name} not found")
            return {}
        return self.__joint_configs[robot_name].copy()
    
    def compute_fk(self, robot_name: str, joint_config: dict = None):
        """
        Compute forward kinematics for end-effector position and orientation
        
        Args:
            robot_name: Name of the robot
            joint_config: Optional joint configuration. If None, uses current stored configuration
            
        Returns:
            dict: {'position': [x, y, z], 'orientation': [rx, ry, rz], 'transform': 4x4_matrix}
            Returns None if computation fails
        """
        if robot_name not in self.__robots:
            self.__console.error(f"Robot {robot_name} not found")
            return None
            
        try:
            robot_info = self.__robots[robot_name]
            base_T = robot_info['base_transform']
            
            # Use provided joint config or current stored config
            if joint_config is None:
                joint_config = self.__joint_configs[robot_name]
            
            if robot_info['type'] == 'pinocchio':
                # Use pinocchio for forward kinematics
                pin_model = robot_info['pin_model']
                pin_data = robot_info['pin_data']
                
                # Create joint angle vector for pinocchio
                q = np.zeros(pin_model.nq)  # Configuration vector
                joint_names = robot_info['joint_names']
                
                for i, joint_name in enumerate(joint_names):
                    if i + 1 < len(q):  # Skip universe joint (index 0)
                        q[i + 1] = joint_config.get(joint_name, 0.0)
                
                # Compute forward kinematics
                pin.forwardKinematics(pin_model, pin_data, q)
                pin.updateFramePlacements(pin_model, pin_data)
                
                # Get end-effector transform (last frame)
                end_effector_frame_id = pin_model.nframes - 1
                end_effector_T = pin_data.oMf[end_effector_frame_id].homogeneous
                
            else:
                # Use URDF parser for forward kinematics
                robot = robot_info['robot']
                
                # Ensure all joints have values
                cfg = {}
                for joint_name in robot_info['joint_names']:
                    cfg[joint_name] = joint_config.get(joint_name, 0.0)
                
                # Compute forward kinematics using URDF
                fk_result = robot.link_fk(cfg=cfg)
                
                # Get end-effector transform (typically the last link)
                end_effector_links = list(fk_result.keys())
                if not end_effector_links:
                    self.__console.error(f"No links found for robot {robot_name}")
                    return None
                    
                # Use the last link as end-effector
                end_effector_link = end_effector_links[-1]
                end_effector_T = fk_result[end_effector_link]
            
            # Apply base transformation
            final_T = base_T @ end_effector_T
            
            # Extract position and orientation
            position = final_T[:3, 3].tolist()
            rotation_matrix = final_T[:3, :3]
            
            # Convert rotation matrix to Euler angles (XYZ convention)
            orientation_rad = rotations.Rotation.from_matrix(rotation_matrix).as_euler('xyz')
            orientation = orientation_rad.tolist()
            
            result = {
                'position': position,
                'orientation': orientation,  # in radians
                'orientation_deg': np.rad2deg(orientation_rad).tolist(),  # in degrees for convenience
                'transform': final_T.tolist()
            }
            
            self.__console.debug(f"FK for {robot_name}: pos={position}, ori_deg={np.rad2deg(orientation_rad)}")
            return result
            
        except Exception as e:
            self.__console.error(f"Failed to compute forward kinematics for {robot_name}: {e}")
            return None
    
    def compute_ik(self, robot_name: str, target_position: list, target_orientation: list = None, 
                                 initial_joint_config: dict = None, max_iterations: int = 1000, tolerance: float = 1e-6):
        """
        Compute inverse kinematics to reach target pose
        
        Args:
            robot_name: Name of the robot
            target_position: Target position [x, y, z]
            target_orientation: Target orientation [rx, ry, rz] in radians (optional)
            initial_joint_config: Initial joint configuration for IK solver
            max_iterations: Maximum iterations for IK solver
            tolerance: Convergence tolerance
            
        Returns:
            dict: {'joint_angles': {joint_name: angle}, 'success': bool, 'error': float}
            Returns None if robot not found or IK fails
        """
        if robot_name not in self.__robots:
            self.__console.error(f"Robot {robot_name} not found")
            return None
            
        robot_info = self.__robots[robot_name]
        
        # Only pinocchio supports IK directly
        if robot_info['type'] != 'pinocchio':
            self.__console.warning(f"Inverse kinematics only available with pinocchio backend")
            return None
            
        try:
            pin_model = robot_info['pin_model']
            pin_data = robot_info['pin_data']
            base_T = robot_info['base_transform']
            
            # Create target transform matrix
            target_T = np.eye(4)
            target_T[:3, 3] = target_position
            
            if target_orientation is not None:
                target_R = rotations.Rotation.from_euler('xyz', target_orientation).as_matrix()
                target_T[:3, :3] = target_R
            else:
                # If no orientation specified, use current orientation
                current_fk = self.compute_fk(robot_name)
                if current_fk:
                    current_T = np.array(current_fk['transform'])
                    target_T[:3, :3] = current_T[:3, :3]
            
            # Apply inverse base transform
            target_T_local = np.linalg.inv(base_T) @ target_T
            
            # Initial joint configuration
            if initial_joint_config is None:
                initial_joint_config = self.__joint_configs[robot_name]
            
            # Create initial configuration vector
            q_init = np.zeros(pin_model.nq)
            joint_names = robot_info['joint_names']
            
            for i, joint_name in enumerate(joint_names):
                if i + 1 < len(q_init):
                    q_init[i + 1] = initial_joint_config.get(joint_name, 0.0)
            
            # Get end-effector frame ID
            end_effector_frame_id = pin_model.nframes - 1
            
            # Create target SE3 object
            target_se3 = pin.SE3(target_T_local[:3, :3], target_T_local[:3, 3])
            
            # Solve IK using CLIK (Closed-Loop Inverse Kinematics)
            q_result = q_init.copy()
            error = float('inf')
            
            for iteration in range(max_iterations):
                # Compute forward kinematics
                pin.forwardKinematics(pin_model, pin_data, q_result)
                pin.updateFramePlacements(pin_model, pin_data)
                
                # Get current end-effector pose
                current_pose = pin_data.oMf[end_effector_frame_id]
                
                # Compute error
                pose_error = pin.log6(current_pose.inverse() * target_se3)
                error = np.linalg.norm(pose_error.vector)
                
                if error < tolerance:
                    break
                
                # Compute Jacobian
                pin.computeFrameJacobian(pin_model, pin_data, q_result, end_effector_frame_id)
                J = pin_data.J
                
                # Damped least squares solution
                lambda_damping = 0.01  # Damping parameter
                J_damped = J.T @ np.linalg.inv(J @ J.T + lambda_damping**2 * np.eye(J.shape[0]))
                dq = J_damped @ pose_error.vector
                
                # Update configuration
                q_result = pin.integrate(pin_model, q_result, dq)
            
            # Extract joint angles from result
            joint_angles = {}
            for i, joint_name in enumerate(joint_names):
                if i + 1 < len(q_result):
                    joint_angles[joint_name] = q_result[i + 1]
            
            success = error < tolerance
            
            self.__console.debug(f"IK for {robot_name}: iterations={iteration+1}, error={error:.6f}, success={success}")
            
            return {
                'joint_angles': joint_angles,
                'success': success,
                'error': error,
                'iterations': iteration + 1
            }
            
        except Exception as e:
            self.__console.error(f"Failed to compute inverse kinematics for {robot_name}: {e}")
            return None
    
    def get_end_effector_pose(self, robot_name: str):
        """
        Get current end-effector pose using stored joint configuration
        
        Args:
            robot_name: Name of the robot
            
        Returns:
            dict: {'position': [x, y, z], 'orientation': [rx, ry, rz]} or None if failed
        """
        fk_result = self.compute_fk(robot_name)
        if fk_result:
            return {
                'position': fk_result['position'],
                'orientation': fk_result['orientation']
            }
        return None
    
    def reset_joint_angles(self, robot_name: str = None):
        """
        Reset joint angles to zero
        
        Args:
            robot_name: Name of the robot. If None, resets all robots
        """
        if robot_name:
            if robot_name in self.__joint_configs:
                for joint_name in self.__joint_configs[robot_name]:
                    self.__joint_configs[robot_name][joint_name] = 0.0
                self.__console.debug(f"Reset joint angles for robot {robot_name}")
            else:
                self.__console.error(f"Robot {robot_name} not found")
        else:
            for name in self.__joint_configs:
                for joint_name in self.__joint_configs[name]:
                    self.__joint_configs[name][joint_name] = 0.0
            self.__console.debug("Reset joint angles for all robots")
    
    def get_robot_info(self, robot_name: str):
        """
        Get information about a robot
        
        Returns:
            dict: Robot information including joint names, base transform, etc.
        """
        if robot_name not in self.__robots:
            return None
            
        robot_info = self.__robots[robot_name].copy()
        # Don't return the actual robot object for safety
        result = {
            'joint_names': robot_info['joint_names'],
            'base_transform': robot_info['base_transform'].tolist(),
            'urdf_path': robot_info['urdf_path'],
            'current_joint_config': self.__joint_configs[robot_name].copy()
        }
        return result
    
    def compute_all_robots_fk(self):
        """
        Compute forward kinematics for all robots
        
        Returns:
            dict: {robot_name: fk_result} for all robots
        """
        results = {}
        for robot_name in self.__robots:
            fk_result = self.compute_fk(robot_name)
            if fk_result:
                results[robot_name] = fk_result
        return results