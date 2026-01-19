import numpy as np
import sys
import os
import json
from scipy.optimize import minimize

# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from plugins.pluginbase.optimizerbase import OptimizerBase

import open3d as o3d

class TrajOpt(OptimizerBase):
    def __init__(self, config_path: str = None):
        if config_path is None:
             config_path = os.path.splitext(__file__)[0] + '.json'
        super().__init__(config_path)
        
        try:
            with open(config_path, 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            print(f"[TrajOpt] Warning: Could not load config: {e}")
            self.config = {}
            
        self.safety_margin = self.config.get("safety_margin", 10.0)
        self.w_smooth = self.config.get("w_smooth", 1.0)
        self.w_collision = self.config.get("w_collision", 20.0)
        self.max_iter = self.config.get("max_iter", 20)

    def optimize(self, path: list, planner) -> list:
        if not path or len(path) < 3:
            return path
            
        path_arr = np.array(path)
        n_waypoints = path_arr.shape[0]
        dim = path_arr.shape[1]
        
        start_conf = path_arr[0]
        goal_conf = path_arr[-1]
        
        x0 = path_arr[1:-1].flatten()
        
        def reconstruct_path(x):
            inner = x.reshape((n_waypoints - 2, dim))
            return np.vstack([start_conf, inner, goal_conf])
            
        # Cost Function: Smoothness only
        # Collision handled as constraints or separate penalty
        def obj_fn(x):
            curr_path = reconstruct_path(x)
            # Smoothness: Sum of squared distances (shortest path) or accelerations
            # TrajOpt usually does min Sum ||v||^2 or ||a||^2
            # Let's do ||a||^2 (smoothness)
            acc = curr_path[2:] - 2*curr_path[1:-1] + curr_path[:-2]
            return 0.5 * np.sum(acc**2) * self.w_smooth

        # Constraints
        # dist(x_i) >= safety_margin
        # We need equality/inequality constraints for scipy
        
        constraints = []
        
        if hasattr(planner, 'scene') and planner.scene is not None:
             # Define constraint for each waypoint?
             # For intermediate points only (start/goal fixed)
             # x has (N-2) * D variables
             # We check distance for each of the (N-2) points.
             
             # Constraint function usually takes x and returns vector
             # SLSQP supports dictionary of constraints
             
             def collision_constraint(x):
                 # Returns vector of (dist - margin)
                 # Should be >= 0
                 curr_path = reconstruct_path(x)
                 inner_pos = curr_path[1:-1, :3] # Only optimize inner 3D collision
                 
                 query = o3d.core.Tensor(inner_pos, dtype=o3d.core.Dtype.Float32)
                 dist = planner.scene.compute_distance(query)
                 dist_np = dist.numpy()
                 
                 return dist_np - self.safety_margin
                 
             constraints.append({
                 'type': 'ineq',
                 'fun': collision_constraint
             })
             
             # We can also add penalty to objective if constraints are hard to satisfy
             # But 'Trailer' TrajOpt is constrained optimization.
        
        # Optimize using SLSQP
        try:
             res = minimize(obj_fn, x0, method='SLSQP', constraints=constraints, 
                            options={'maxiter': self.max_iter, 'disp': True})
             final_x = res.x
        except Exception as e:
             # SLSQP might fail if memory issue or singular
             print(f"[TrajOpt] Optimization failed: {e}")
             final_x = x0
             
        optimized_path_arr = reconstruct_path(final_x)
        return [p for p in optimized_path_arr]
