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

class TaskSpaceRRTConnect(PlannerBase):
    def __init__(self, config_path: str = None):
        super().__init__()
        if config_path is None:
            config_path = os.path.splitext(__file__)[0] + '.json'
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
            
        self.step_size = self.config.get("step_size", 1.0)
        self.max_iter = self.config.get("max_iter", 1000)
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

    def generate(self, current_pose: Union[List[float], np.ndarray], target_pose: Union[List[float], np.ndarray], step_callback: Optional[callable] = None) -> List[np.ndarray]:
        current_pose = np.array(current_pose, dtype=float)
        target_pose = np.array(target_pose, dtype=float)
        
        # Determine strict goal from target_pose (handle NaNs for Don't Care)
        # For RRT-Connect, dual tree needs explicit start and goal.
        # If Goal has NaNs, we can't easily grow a tree from it unless we pick a concrete goal.
        # Strategy: Sample a concrete goal pose consistent with target_pose (fill NaNs with random or heuristics).
        # Since this is a planner, let's substitute NaNs with current_pose values or 0?
        # Better: Sample one concrete goal and try to connect. 
        # Or better: Just use current_pose values for NaNs (maintain orientation etc.)
        
        concrete_goal = np.copy(target_pose)
        mask_goal = np.isnan(concrete_goal)
        # Using current pose orientation for goal if unspecified seems safe for "Mainly position" task
        concrete_goal[mask_goal] = current_pose[mask_goal]
        
        # Tree A (Start), Tree B (Goal)
        tree_a = [current_pose]
        parents_a = {0: None}
        
        tree_b = [concrete_goal]
        parents_b = {0: None}
        
        path_found = False
        connect_node_a_idx = -1
        connect_node_b_idx = -1
        
        w_pos = self.weights['pos']
        w_ori = self.weights['orient']
        
        min_dist_between_trees = float('inf')
        
        for i in range(self.max_iter):
            logging.info(f"Iteration {i+1}/{self.max_iter} | Tree A: {len(tree_a)} | Tree B: {len(tree_b)} | Min Gap: {min_dist_between_trees:.2f}")
            # Sample
            if np.random.random() < 0.1: # Small bias just in case (though Connect is greedy)
                rnd_point = np.copy(concrete_goal) # Or swap
            else:
                rnd_point = np.zeros(6)
                rnd_point[0] = np.random.uniform(self.bounds['x_min'], self.bounds['x_max'])
                rnd_point[1] = np.random.uniform(self.bounds['y_min'], self.bounds['y_max'])
                rnd_point[2] = np.random.uniform(self.bounds['z_min'], self.bounds['z_max'])
                rnd_point[3] = np.random.uniform(self.bounds['roll_min'], self.bounds['roll_max'])
                rnd_point[4] = np.random.uniform(self.bounds['pitch_min'], self.bounds['pitch_max'])
                rnd_point[5] = np.random.uniform(self.bounds['yaw_min'], self.bounds['yaw_max'])
                
            # Extend A
            new_idx_a = self._extend(tree_a, parents_a, rnd_point, w_pos, w_ori)
            
            if new_idx_a is not None:
                new_node_a = tree_a[new_idx_a]
                
                # Check Gap
                diffs_b = np.array(tree_b) - new_node_a
                dists_b = w_pos * np.sum(diffs_b[:, :3]**2, axis=1) + w_ori * np.sum(diffs_b[:, 3:]**2, axis=1)
                min_gap = np.sqrt(np.min(dists_b))
                if min_gap < min_dist_between_trees:
                    min_dist_between_trees = min_gap
                
                # Connect B to new_node_a
                new_idx_b = self._connect(tree_b, parents_b, new_node_a, w_pos, w_ori)
                
                if new_idx_b is not None:
                    connect_node_a_idx = new_idx_a
                    connect_node_b_idx = new_idx_b
                    path_found = True
                    break
            
            # Callback
            if step_callback:
                step_callback(tree_a, parents_a, tree_b, parents_b)
                
            # Swap
            tree_a, tree_b = tree_b, tree_a
            parents_a, parents_b = parents_b, parents_a
            
        if path_found:
            # Check which tree is start
            root_a = tree_a[0]
            if np.allclose(root_a, current_pose):
                path_start = self._trace_path(tree_a, parents_a, connect_node_a_idx)[::-1]
                path_goal = self._trace_path(tree_b, parents_b, connect_node_b_idx)
                return path_start + path_goal
            else:
                path_start = self._trace_path(tree_b, parents_b, connect_node_b_idx)[::-1]
                path_goal = self._trace_path(tree_a, parents_a, connect_node_a_idx)
                return path_start + path_goal
                
        logging.error(f"Task Space RRT-Connect failed. Max iterations ({self.max_iter}) reached.")
        logging.error(f"Smallest gap between trees: {min_dist_between_trees:.4f}")
        return []

    def _extend(self, nodes, parents, target, w_pos, w_ori):
        diffs = np.array(nodes) - target
        pos_diff = diffs[:, :3]
        ori_diff = diffs[:, 3:]
        dists = w_pos * np.sum(pos_diff**2, axis=1) + w_ori * np.sum(ori_diff**2, axis=1)
        nearest_idx = np.argmin(dists)
        nearest_node = nodes[nearest_idx]
        
        direction = target - nearest_node
        length = np.sqrt(w_pos * np.sum(direction[:3]**2) + w_ori * np.sum(direction[3:]**2))
        
        if length == 0: return None
        
        ratio = min(1.0, self.step_size / length)
        new_point = nearest_node + direction * ratio
        
        if not self._check_collision(nearest_node, new_point):
            nodes.append(new_point)
            new_idx = len(nodes) - 1
            parents[new_idx] = nearest_idx
            return new_idx
        return None

    def _connect(self, nodes, parents, target, w_pos, w_ori):
        # Greedy connect: repeatedly extend
        curr_idx = -1
        # First, find nearest
        diffs = np.array(nodes) - target
        dists = w_pos * np.sum(diffs[:, :3]**2, axis=1) + w_ori * np.sum(diffs[:, 3:]**2, axis=1)
        nearest_idx = np.argmin(dists)
        
        curr_node = nodes[nearest_idx]
        curr_idx_in_tree = nearest_idx
        
        while True:
            direction = target - curr_node
            length = np.sqrt(w_pos * np.sum(direction[:3]**2) + w_ori * np.sum(direction[3:]**2))
            
            if length < self.step_size:
                # Reachable
                if not self._check_collision(curr_node, target):
                    nodes.append(target)
                    new_idx = len(nodes) - 1
                    parents[new_idx] = curr_idx_in_tree
                    return new_idx
                return None
                
            # Step
            ratio = self.step_size / length
            new_point = curr_node + direction * ratio
            
            if not self._check_collision(curr_node, new_point):
                nodes.append(new_point)
                new_idx = len(nodes) - 1
                parents[new_idx] = curr_idx_in_tree
                
                curr_node = new_point
                curr_idx_in_tree = new_idx
            else:
                return None # Blocked

    def _trace_path(self, nodes, parents, idx):
        path = []
        while idx is not None:
            path.append(nodes[idx])
            idx = parents[idx]
        return path

    def _check_collision(self, p1, p2):
        if self.scene is None:
            return False
            
        pos1 = p1[:3]
        pos2 = p2[:3]
        direction = pos2 - pos1
        length = np.linalg.norm(direction)
        
        # Ray Check
        if length > 1e-6:
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
                 # Check every 0.5 units
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
