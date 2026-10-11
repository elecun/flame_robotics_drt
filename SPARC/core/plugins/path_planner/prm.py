import numpy as np
import json
import os
import sys
import networkx as nx
from scipy.spatial import KDTree
import open3d as o3d
from typing import List, Union

# Adjust path to import PlannerBase
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from plugins.pluginbase.plannerbase import PlannerBase

class PRM(PlannerBase):
    def __init__(self, config_path: str = None):
        super().__init__()
        if config_path is None:
            config_path = os.path.splitext(__file__)[0] + '.json'
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
            
        self.num_samples = self.config.get("num_samples", 500)
        self.k_neighbors = self.config.get("k_neighbors", 10)
        self.step_size = self.config.get("step_size", 2.0)
        self.bounds = self.config.get("workspace_bounds", {
            "x_min": -50.0, "x_max": 50.0,
            "y_min": -50.0, "y_max": 50.0,
            "z_min": -50.0, "z_max": 50.0
        })
        
        self.scene = None
        self.graph = nx.Graph()
        self.samples = []

    def add_static_object(self, object_model):
        self.static_objects.append(object_model)
        if self.scene is None:
             self.scene = o3d.t.geometry.RaycastingScene()
        
        try:
            t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(object_model)
            self.scene.add_triangles(t_mesh)
        except Exception as e:
            print(f"Error adding object to scene: {e}")

    def _check_collision(self, p1, p2):
        if self.scene is None: return False
        
        direction = p2 - p1
        length = np.linalg.norm(direction)
        if length < 1e-6: return False
        
        direction /= length
        
        # Ray cast
        rays = o3d.core.Tensor([[p1[0], p1[1], p1[2], direction[0], direction[1], direction[2]]], dtype=o3d.core.Dtype.Float32)
        ans = self.scene.cast_rays(rays)
        t_hit = ans['t_hit'][0].item()
        
        if np.isfinite(t_hit) and t_hit < length:
            return True
        return False
        
    def _is_valid(self, p):
        # We can implement point collision check if needed, 
        # but _check_collision for edge handles point indirectly if we check from p to p?
        # Actually RRT usually checks edge. 
        # For PRM sampling, we should check if point is inside obstacle.
        # RaycastingScene closest point? Or verify bounds.
        # For now, simplistic check: assume valid if not colliding with 'known free space' center?
        # Actually, let's skip explicit point check in sampling for now and rely on edge checks 
        # or implement a "check_point" using simple raycast from a far point?
        
        # Hack for IsInside: raycast from point in random direction, if hit is backface? 
        # Open3D RaycastingScene doesn't easily convert to occupancy.
        # But we can assume samples are valid if they are within bounds. 
        # Edge collision is the main check.
        # Ideally we check collision of a very short ray or use distance transform.
        return True

    def generate(self, current_pose: Union[List[float], np.ndarray], target_pose: Union[List[float], np.ndarray]) -> List[np.ndarray]:
        current_pose = np.array(current_pose, dtype=float)
        target_pose = np.array(target_pose, dtype=float)
        
        start_pos = current_pose[:3]
        goal_pos = target_pose[:3]
        
        self.graph = nx.Graph()
        self.samples = [start_pos, goal_pos]
        
        # 1. Sampling Phase
        print(f"[PRM] Sampling {self.num_samples} points...")
        for _ in range(self.num_samples):
            # Sample Position
            rnd_point = np.array([
                np.random.uniform(self.bounds['x_min'], self.bounds['x_max']),
                np.random.uniform(self.bounds['y_min'], self.bounds['y_max']),
                np.random.uniform(self.bounds['z_min'], self.bounds['z_max'])
            ])
            
            if self._is_valid(rnd_point):
                self.samples.append(rnd_point)
                
        # 2. Connection Phase (Roadmap Construction)
        print(f"[PRM] Building roadmap with {len(self.samples)} nodes...")
        
        # Use KDTree for efficient Nearest Neighbor search
        tree = KDTree(self.samples)
        
        # Query k+1 neighbors (point itself is included)
        k = min(self.k_neighbors + 1, len(self.samples))
        dists, indices = tree.query(self.samples, k=k)
        
        for i, neighbors in enumerate(indices):
            p1 = self.samples[i]
            for j in neighbors[1:]: # Skip self (index 0)
                p2 = self.samples[j]
                
                # Distance check (already done by KDTree implicitly/explicitly, but if we want max connection dist?)
                # We trust KDTree k-neighbors.
                
                # Edge Collision Check
                if not self._check_collision(p1, p2):
                     distance = np.linalg.norm(p1 - p2)
                     self.graph.add_edge(i, j, weight=distance)
                     
        # 3. Query Phase
        print("[PRM] Searching for path...")
        start_idx = 0
        goal_idx = 1
        
        try:
            path_indices = nx.shortest_path(self.graph, source=start_idx, target=goal_idx, weight='weight')
            
            # Reconstruct Path
            path = []
            for idx in path_indices:
                pos = self.samples[idx]
                
                # Orientation Handling
                # Interpolate or stick to target/start?
                # Simple: Interpolate RPY from start to goal based on progress?
                # Or just keep start orientation?
                # Let's use target orientation for goal, start for others.
                full_pose = np.concatenate((pos, current_pose[3:])) # Default to start orient
                if idx == goal_idx:
                     full_pose[3:] = np.where(np.isnan(target_pose[3:]), current_pose[3:], target_pose[3:])
                
                path.append(full_pose)
                
            return path
        except nx.NetworkXNoPath:
            print("[PRM] No path found!")
            return []
