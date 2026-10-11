import numpy as np
import json
import os
import open3d as o3d
from typing import List, Union, Optional
import sys
import logging

# Adjust path to import PlannerBase
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from plugins.pluginbase.plannerbase import PlannerBase

class TaskSpaceRRTStar(PlannerBase):
    def __init__(self, config_path: str = None):
        super().__init__()
        if config_path is None:
            config_path = os.path.splitext(__file__)[0] + '.json'
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
            
        self.step_size = self.config.get("step_size", 1.0)
        self.max_iter = self.config.get("max_iter", 1000)
        self.search_radius = self.config.get("search_radius", 5.0)
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
        self.tool_mesh = None

    def add_static_object(self, object_model):
        self.static_objects.append(object_model)
        if self.scene is None:
             self.scene = o3d.t.geometry.RaycastingScene()
        try:
            t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_model)
            self.scene.add_triangles(t_mesh)
        except Exception as e:
            print(f"Error adding object to scene: {e}")

    # ... params ...

    def _check_collision(self, p1, p2):
        if self.scene is None:
            return False
            
        pos1 = p1[:3]
        pos2 = p2[:3]
        direction = pos2 - pos1
        length = np.linalg.norm(direction)
        if length > 1e-6: # Ray Check
            dir_norm = direction / length
            rays = o3d.core.Tensor([[pos1[0], pos1[1], pos1[2], dir_norm[0], dir_norm[1], dir_norm[2]]], dtype=o3d.core.Dtype.Float32)
            ans = self.scene.cast_rays(rays)
            t_hit = ans['t_hit'][0].item()
            if np.isfinite(t_hit) and t_hit < length:
                return True
        
        # Tool Check
        if self.tool_mesh is not None:
             if not hasattr(self, '_tool_pcd'):
                 self._tool_pcd = self.tool_mesh.sample_points_poisson_disk(number_of_points=100)
                 self._tool_pcd_pts = np.asarray(self._tool_pcd.points)

             check_poses = [p2]
             if length > 0.5:
                 num_inter = int(length / 0.5)
                 for i in range(1, num_inter + 1):
                     ratio = i / (num_inter + 1)
                     check_poses.append(p1 + (p2 - p1) * ratio)
             
             for pose in check_poses:
                 R = o3d.geometry.get_rotation_matrix_from_xyz(pose[3:])
                 transformed_pts = (R @ self._tool_pcd_pts.T).T + pose[:3]
                 query = o3d.core.Tensor(transformed_pts, dtype=o3d.core.Dtype.Float32)
                 dist = self.scene.compute_distance(query)
                 if dist.min().item() < 1.0:
                     return True
                     
        return False

    def generate(self, current_pose: Union[List[float], np.ndarray], target_pose: Union[List[float], np.ndarray], step_callback: Optional[callable] = None) -> List[np.ndarray]:
        current_pose = np.array(current_pose)
        target_pose = np.array(target_pose)
        
        mask = np.isnan(target_pose)
        if np.any(mask):
            target_pose[mask] = current_pose[mask]
            
        nodes = [current_pose]
        parents = {0: None}
        costs = {0: 0.0}
        
        w_pos = self.weights['pos']
        w_ori = self.weights['orient']
        
        best_goal_idx = None
        min_goal_cost = float('inf')
        
        min_dist_to_goal = float('inf')
        
        for i in range(self.max_iter):
            # Logging
            # Logging
            if best_goal_idx is not None:
                logging.info(f"Iteration {i+1}/{self.max_iter} | Tree Size: {len(nodes)} | Best Cost: {min_goal_cost:.4f} | Min Dist: {min_dist_to_goal:.2f}")
            else:
                 logging.info(f"Iteration {i+1}/{self.max_iter} | Tree Size: {len(nodes)} | Min Dist: {min_dist_to_goal:.2f}")

            # 1. Sample
            if np.random.random() < self.goal_bias:
                rnd_point = target_pose
            else:
                rnd_point = np.zeros(6)
                rnd_point[0] = np.random.uniform(self.bounds['x_min'], self.bounds['x_max'])
                rnd_point[1] = np.random.uniform(self.bounds['y_min'], self.bounds['y_max'])
                rnd_point[2] = np.random.uniform(self.bounds['z_min'], self.bounds['z_max'])
                rnd_point[3] = np.random.uniform(self.bounds['roll_min'], self.bounds['roll_max'])
                rnd_point[4] = np.random.uniform(self.bounds['pitch_min'], self.bounds['pitch_max'])
                rnd_point[5] = np.random.uniform(self.bounds['yaw_min'], self.bounds['yaw_max'])
            
            # 2. Nearest
            diffs = np.array(nodes) - rnd_point
            pos_diff = diffs[:, :3]
            orient_diff = diffs[:, 3:]
            weighted_sq_dists = w_pos * np.sum(pos_diff**2, axis=1) + w_ori * np.sum(orient_diff**2, axis=1)
            nearest_idx = np.argmin(weighted_sq_dists)
            nearest_node = nodes[nearest_idx]
            
            # 3. Steer
            direction = rnd_point - nearest_node
            dist = np.sqrt(w_pos * np.sum(direction[:3]**2) + w_ori * np.sum(direction[3:]**2))
            if dist == 0: continue
            
            ratio = min(1.0, self.step_size / dist)
            new_point = nearest_node + direction * ratio
            
            # 4. Collision Check
            if self._check_collision(nearest_node, new_point):
                if np.array_equal(rnd_point, target_pose):
                     logging.warning(f"Goal approach blocked by collision! Dist to goal: {dist:.2f}")
                continue
                
            # 5. Near Nodes
            # Calculate dists to all nodes
            diffs_new = np.array(nodes) - new_point
            dists_new = np.sqrt(w_pos * np.sum(diffs_new[:, :3]**2, axis=1) + w_ori * np.sum(diffs_new[:, 3:]**2, axis=1))
            near_indices = np.where(dists_new <= self.search_radius)[0]
            
            # 6. Choose Parent
            min_cost = costs[nearest_idx] + dist * ratio # Cost from nearest
            parent_idx = nearest_idx
            
            for idx in near_indices:
                near_node = nodes[idx]
                # Check collision parent->new
                d_near = dists_new[idx]
                if not self._check_collision(near_node, new_point):
                    cost = costs[idx] + d_near
                    if cost < min_cost:
                        min_cost = cost
                        parent_idx = idx
            
            # Add Node
            nodes.append(new_point)
            new_idx = len(nodes) - 1
            parents[new_idx] = parent_idx
            costs[new_idx] = min_cost
            
            # Update Min Dist for Logging
            delta_g = target_pose - new_point
            d_goal_curr = np.sqrt(w_pos * np.sum(delta_g[:3]**2) + w_ori * np.sum(delta_g[3:]**2))
            if d_goal_curr < min_dist_to_goal:
                min_dist_to_goal = d_goal_curr
            
            # 7. Rewire
            for idx in near_indices:
                if idx == parent_idx: continue
                near_node = nodes[idx]
                d_near = dists_new[idx]
                
                # Check collision new->near
                if not self._check_collision(new_point, near_node):
                    new_cost = costs[new_idx] + d_near
                    if new_cost < costs[idx]:
                        # Rewire
                        parents[idx] = new_idx
                        costs[idx] = new_cost
                        
            # Callback
            if step_callback:
                step_callback(nodes, parents) 
                        
            # Check Goal
            delta = target_pose - new_point
            d_goal = np.sqrt(w_pos * np.sum(delta[:3]**2) + w_ori * np.sum(delta[3:]**2))
            
            if d_goal < self.step_size:
                 if not self._check_collision(new_point, target_pose):
                     cost_to_goal = costs[new_idx] + d_goal
                     if cost_to_goal < min_goal_cost:
                         min_goal_cost = cost_to_goal
                         best_goal_idx = new_idx
                         logging.info(f"Goal Reached! Stopping early with cost: {min_goal_cost:.2f}")
                         break
                         best_goal_idx = new_idx
                         
        if best_goal_idx is not None:
             # Connect to exact goal
             nodes.append(target_pose)
             goal_final_idx = len(nodes) - 1
             parents[goal_final_idx] = best_goal_idx
             
             path = []
             curr = goal_final_idx
             while curr is not None:
                 path.append(nodes[curr])
                 curr = parents[curr]
             return path[::-1]
             
        logging.error(f"Task Space RRT* failed to find path. Max iterations ({self.max_iter}) reached.")
        logging.error(f"Closest distance to goal achieved: {min_dist_to_goal:.4f}")
        return []
