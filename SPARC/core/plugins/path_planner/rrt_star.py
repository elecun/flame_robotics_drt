import numpy as np
import json
import os
import open3d as o3d
from typing import List, Union
import sys

# Adjust path to import PlannerBase
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from plugins.pluginbase.plannerbase import PlannerBase

class RRTStar(PlannerBase):
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
        self.bounds = self.config.get("workspace_bounds", {
            "x_min": -10.0, "x_max": 10.0,
            "y_min": -10.0, "y_max": 10.0,
            "z_min": -10.0, "z_max": 10.0
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
        
        start_pos = current_pose[:3]
        goal_pos = target_pose[:3]
        
        # Nodes list containing coordinates
        nodes = [start_pos]
        # Parents map: index -> parent_index
        parents = {0: None}
        # Costs map: index -> cost from start
        costs = {0: 0.0}
        
        for i in range(self.max_iter):
            # Sample
            if np.random.random() < self.goal_bias:
                rnd_point = goal_pos
            else:
                rnd_point = np.array([
                    np.random.uniform(self.bounds['x_min'], self.bounds['x_max']),
                    np.random.uniform(self.bounds['y_min'], self.bounds['y_max']),
                    np.random.uniform(self.bounds['z_min'], self.bounds['z_max'])
                ])
                
            # Nearest
            dists = np.linalg.norm(np.array(nodes) - rnd_point, axis=1)
            nearest_idx = np.argmin(dists)
            nearest_node = nodes[nearest_idx]
            
            # Steer
            direction = rnd_point - nearest_node
            length = np.linalg.norm(direction)
            if length == 0:
                continue
            
            direction /= length
            new_point = nearest_node + direction * min(self.step_size, length)
            
            if self._check_collision(nearest_node, new_point):
                continue
                
            # Optimization: Choose best parent
            new_idx = len(nodes)
            
            # Find neighbors within radius
            dists_all = np.linalg.norm(np.array(nodes) - new_point, axis=1)
            neighbor_indices = np.where(dists_all < self.search_radius)[0]
            
            min_cost = costs[nearest_idx] + np.linalg.norm(new_point - nearest_node)
            best_parent_idx = nearest_idx
            
            for nb_idx in neighbor_indices:
                if nb_idx == nearest_idx:
                    continue
                # Check collision from neighbor to new_point
                if not self._check_collision(nodes[nb_idx], new_point):
                    cost = costs[nb_idx] + np.linalg.norm(new_point - nodes[nb_idx])
                    if cost < min_cost:
                        min_cost = cost
                        best_parent_idx = nb_idx
                        
            # Add node
            nodes.append(new_point)
            parents[new_idx] = best_parent_idx
            costs[new_idx] = min_cost
            
            # Rewire
            for nb_idx in neighbor_indices:
                if nb_idx == best_parent_idx:
                    continue
                # Check if going through new node is shorter
                dist_to_nb = np.linalg.norm(nodes[nb_idx] - new_point)
                new_cost_to_nb = min_cost + dist_to_nb
                
                if new_cost_to_nb < costs[nb_idx]:
                    if not self._check_collision(new_point, nodes[nb_idx]):
                        parents[nb_idx] = new_idx
                        costs[nb_idx] = new_cost_to_nb
                        # Note: Technically need to propagate cost updates to children of nb_idx,
                        # but standard RRT* often omits full propagation or does it lazily.
                        # For simplicity, we update parent and immediate cost.
                        
        # Find best path to goal
        # Find nodes close to goal
        dists_to_goal = np.linalg.norm(np.array(nodes) - goal_pos, axis=1)
        # Check if any connect to goal
        close_indices = np.where(dists_to_goal < self.step_size)[0]
        
        goal_idx = -1
        min_total_cost = float('inf')
        
        for idx in close_indices:
            if not self._check_collision(nodes[idx], goal_pos):
                cost = costs[idx] + np.linalg.norm(goal_pos - nodes[idx])
                if cost < min_total_cost:
                    min_total_cost = cost
                    goal_idx = idx
                    
        if goal_idx != -1:
             # Reconstruct
            path = []
            
            # Add goal manually
            pose = np.copy(target_pose)
            # Handle NaN
            final_orient = target_pose[3:]
            current_orient = current_pose[3:]
            pose[3:] = np.where(np.isnan(final_orient), current_orient, final_orient)
            path.append(pose)
            
            curr_idx = goal_idx
            while curr_idx is not None:
                pose = np.copy(current_pose)
                pose[:3] = nodes[curr_idx]
                pose[3:] = current_pose[3:] # Default orientation
                path.append(pose)
                curr_idx = parents[curr_idx]
                
            return path[::-1]

        print("RRT* failed to find path")
        return []

    def _check_collision(self, p1, p2):
        if self.scene is None:
            return False
        direction = p2 - p1
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return False
        direction /= length
        rays = o3d.core.Tensor([[p1[0], p1[1], p1[2], direction[0], direction[1], direction[2]]], dtype=o3d.core.Dtype.Float32)
        ans = self.scene.cast_rays(rays)
        t_hit = ans['t_hit'][0].item()
        if np.isfinite(t_hit) and t_hit < length:
            return True
        return False
