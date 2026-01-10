import numpy as np
import json
import os
import open3d as o3d
from typing import List, Union
import sys

# Adjust path to import PlannerBase
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from plannerbase import PlannerBase

class TaskSpaceRRT(PlannerBase):
    def __init__(self, config_path: str = None):
        super().__init__()
        if config_path is None:
            config_path = os.path.splitext(__file__)[0] + '.json'
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
            
        self.step_size = self.config.get("step_size", 1.0)
        self.max_iter = self.config.get("max_iter", 1000)
        self.goal_bias = self.config.get("goal_bias", 0.1)
        self.weights = self.config.get("weights", {"pos": 1.0, "orient": 0.5})
        self.bounds = self.config.get("workspace_bounds", {
            "x_min": -10.0, "x_max": 10.0,
            "y_min": -10.0, "y_max": 10.0,
            "z_min": -10.0, "z_max": 10.0,
            "roll_min": -np.pi, "roll_max": np.pi,
            "pitch_min": -np.pi, "pitch_max": np.pi,
            "yaw_min": -np.pi, "yaw_max": np.pi
        })
        
        self.scene = None

    def add_static_object(self, object_model):
        self.static_objects.append(object_model)
        if self.scene is None:
             self.scene = o3d.t.geometry.RaycastingScene()
        try:
            t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_model)
            self.scene.add_triangles(t_mesh)
        except Exception as e:
            print(f"Error adding object to scene: {e}")

    def generate(self, current_pose: Union[List[float], np.ndarray], target_pose: Union[List[float], np.ndarray]) -> List[np.ndarray]:
        current_pose = np.array(current_pose, dtype=float)
        target_pose = np.array(target_pose, dtype=float)
        
        # 6D Nodes
        nodes = [current_pose]
        parents = {0: None}
        
        path_found = False
        goal_node_idx = -1
        
        # Identify Don't Care in Goal
        # If NaN, we don't try to converge to it, effectively.
        # But for RRT to work in 6D, we need a target.
        # If goal has NaNs, we sample those dimensions freely? 
        # Or we treat distance as 0 for those dims?
        # Let's treat distance as 0 for NaN dimensions in Goal sampling.
        
        for i in range(self.max_iter):
            # Sample
            if np.random.random() < self.goal_bias:
                # Sample Goal
                # If NaN, we can pick random value or keep current?
                # RRT bias means we want to pull towards goal.
                # If goal is don't care, we don't pull? 
                # Let's use target_pose but replace NaNs with random or zero.
                # Actually, if NaN, we just don't care where it is.
                # We can sample random for NaN dims.
                rnd_point = np.copy(target_pose)
                mask = np.isnan(rnd_point)
                rnd_point[mask] = np.random.uniform(-np.pi, np.pi, size=np.sum(mask)) # Assuming angle dims mostly
                # For Position NaNs? (Unlikely but possible).
                
            else:
                rnd_point = np.zeros(6)
                rnd_point[0] = np.random.uniform(self.bounds['x_min'], self.bounds['x_max'])
                rnd_point[1] = np.random.uniform(self.bounds['y_min'], self.bounds['y_max'])
                rnd_point[2] = np.random.uniform(self.bounds['z_min'], self.bounds['z_max'])
                rnd_point[3] = np.random.uniform(self.bounds['roll_min'], self.bounds['roll_max'])
                rnd_point[4] = np.random.uniform(self.bounds['pitch_min'], self.bounds['pitch_max'])
                rnd_point[5] = np.random.uniform(self.bounds['yaw_min'], self.bounds['yaw_max'])
                
            # Nearest
            # Weighted Distance
            diffs = np.array(nodes) - rnd_point
            # Weighted Metric: Pos diff + Orient diff (scaled)
            # Simple Euclidean on 6D with weights
            pos_diff = diffs[:, :3]
            orient_diff = diffs[:, 3:]
            
            # Handle orientation periodicity? standard RRT often ignores it, 
            # but for correctness we should use angular diff. Simple absolute diff for now.
            
            w_pos = self.weights['pos']
            w_ori = self.weights['orient']
            
            weighted_sq_dists = w_pos * np.sum(pos_diff**2, axis=1) + w_ori * np.sum(orient_diff**2, axis=1)
            nearest_idx = np.argmin(weighted_sq_dists)
            nearest_node = nodes[nearest_idx]
            
            # Steer
            direction = rnd_point - nearest_node
            # Normalize direction? 
            # We need a metric length
            length = np.sqrt(w_pos * np.sum(direction[:3]**2) + w_ori * np.sum(direction[3:]**2))
            
            if length == 0:
                continue
            
            direction = direction / length # Normalized in Weighted Space?
            # Actually, standard steering moves fixed step size.
            # Let's move in the raw direction but clamped.
            
            # Re-calculating raw length for step size limiting
            # Usually step size applies to position. Orientation changes accordingly.
            raw_pos_len = np.linalg.norm(direction[:3])
            
            # If we enforce step size on position mainly:
            # factor = step_size / raw_pos_len
            # But we are in 6D. Let's use the weighted length limit.
            
            factor = min(self.step_size, length)
            # new_point = nearest + dir * factor? 
            # If dir is normalized in weighted space, this is tricky.
            # Simple approach: interpolate linearly.
            
            ratio = min(1.0, self.step_size / length)
            new_point = nearest_node + (rnd_point - nearest_node) * ratio
            
            if not self._check_collision(nearest_node, new_point):
                nodes.append(new_point)
                new_idx = len(nodes) - 1
                parents[new_idx] = nearest_idx
                
                # Check Goal
                # Compute distance to goal (respecting ignore NaNs)
                delta = new_point - target_pose
                # Mask NaNs
                mask = np.isnan(target_pose)
                delta[mask] = 0.0
                
                dist_to_goal = np.sqrt(w_pos * np.sum(delta[:3]**2) + w_ori * np.sum(delta[3:]**2))
                
                if dist_to_goal < self.step_size:
                    # Reachable?
                    # Check collision to goal (with NaNs replaced by current vals)
                    final_node = np.copy(target_pose)
                    final_node[mask] = new_point[mask]
                    
                    if not self._check_collision(new_point, final_node):
                        nodes.append(final_node)
                        goal_node_idx = len(nodes) - 1
                        parents[goal_node_idx] = new_idx
                        path_found = True
                        break
                        
        if path_found:
            path = []
            curr_idx = goal_node_idx
            while curr_idx is not None:
                path.append(nodes[curr_idx])
                curr_idx = parents[curr_idx]
            return path[::-1]
            
        print("Task-space RRT failed.")
        return []

    def _check_collision(self, p1, p2):
        if self.scene is None:
            return False
            
        # We only check collision for Position (3D)
        # Orientation collision is not checked against environment here (point robot assumption for collision)
        # If user wanted swept volume collision, that requires robot kinematics.
        # User prompt implies "endeffector position" and "static object".
        # So we just check line segment p1[:3] to p2[:3]
        
        pos1 = p1[:3]
        pos2 = p2[:3]
        
        direction = pos2 - pos1
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return False
        direction /= length
        
        rays = o3d.core.Tensor([[pos1[0], pos1[1], pos1[2], direction[0], direction[1], direction[2]]], dtype=o3d.core.Dtype.Float32)
        ans = self.scene.cast_rays(rays)
        t_hit = ans['t_hit'][0].item()
        
        if np.isfinite(t_hit) and t_hit < length:
            return True
        return False
