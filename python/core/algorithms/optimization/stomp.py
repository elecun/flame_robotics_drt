import numpy as np
import sys
import os

# Adjust path to import OptimizerBase
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from optimizerbase import OptimizerBase

class Stomp(OptimizerBase):
    def __init__(self, config_path: str = None):
        import json
        if config_path is None:
             config_path = os.path.splitext(__file__)[0] + '.json'
        super().__init__(config_path)
        
        try:
            with open(config_path, 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            print(f"[STOMP] Warning: Could not load config: {e}")
            self.config = {}

        self.num_iterations = self.config.get("num_iterations", 50)
        self.num_samples = self.config.get("num_samples", 20)
        self.std_dev_factor = self.config.get("std_dev_factor", 0.5)
        self.smoothing_factor = self.config.get("smoothing_factor", 0.1)

    def _is_collision_free(self, path, planner):
        if hasattr(planner, 'check_collision_on_path'): # If planner has efficient bulk check
             return not planner.check_collision_on_path(path)
        
        # Fallback to segment check
        for i in range(len(path) - 1):
            if planner._check_collision(path[i], path[i+1]):
                return False
        return True

    def optimize(self, path: list, planner) -> list:
        if not path or len(path) < 3:
            print("[STOMP] Path too short to optimize.")
            return path

        current_path = np.array(path)
        n_waypoints = len(current_path)
        dim = current_path.shape[1]
        
        start_pose = current_path[0].copy()
        goal_pose = current_path[-1].copy()
        
        best_valid_path = None
        if self._is_collision_free(current_path, planner):
            best_valid_path = current_path.copy()

        # Adaptive Noise Scale
        diffs = np.diff(current_path, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        avg_dist = np.mean(dists)
        current_std_dev = avg_dist * self.std_dev_factor
        print(f"[STOMP] Initializing with avg_step_dist={avg_dist:.2f}, std_dev={current_std_dev:.2f}")

        # Smoothing Kernel
        kernel_size = 5
        kernel = np.ones(kernel_size) / kernel_size

        for iter_num in range(self.num_iterations):
            # 1. Generate Correlated Noise
            raw_noise = np.random.normal(0, current_std_dev, (self.num_samples, n_waypoints, dim))
            noise = np.zeros_like(raw_noise)
            for i in range(self.num_samples):
                for d in range(dim):
                    noise[i, :, d] = np.convolve(raw_noise[i, :, d], kernel, mode='same')
            
            noise[:, 0, :] = 0
            noise[:, -1, :] = 0
            
            # 2. Candidates
            candidates = np.tile(current_path, (self.num_samples, 1, 1))
            candidates += noise
            
            # 3. Evaluate Costs
            costs = np.zeros(self.num_samples)
            for i in range(self.num_samples):
                cand = candidates[i]
                diffs = np.diff(cand, axis=0)
                length_cost = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
                
                obstacle_cost = 0.0
                if not self._is_collision_free(cand, planner):
                    obstacle_cost = 1e9
                
                costs[i] = length_cost + obstacle_cost
            
            # 4. Update Path
            best_idx = np.argmin(costs)
            min_cost = costs[best_idx]
            
            if min_cost < 1e9:
                 exp_cost = np.exp(-10.0 * (costs - min_cost) / (np.max(costs) - min_cost + 1e-6))
                 weights = exp_cost / np.sum(exp_cost)
                 
                 weighted_change = np.zeros_like(current_path)
                 for i in range(self.num_samples):
                     weighted_change += (candidates[i] - current_path) * weights[i]
                     
                 # Proposed update
                 updated_path = current_path + weighted_change
                 updated_path[0] = start_pose
                 updated_path[-1] = goal_pose
                 
                 # 5. Smoothing (with safety check)
                 smoothed_path = updated_path.copy()
                 pad_size = kernel_size // 2
                 for d in range(dim):
                     col = smoothed_path[:, d]
                     padded = np.pad(col, (pad_size, pad_size), mode='edge')
                     smoothed = np.convolve(padded, kernel, mode='valid')
                     smoothed_path[:, d] = smoothed
                 smoothed_path[0] = start_pose
                 smoothed_path[-1] = goal_pose

                 # Safety Checks
                 accepted = False
                 
                 # Try Smoothed
                 if self._is_collision_free(smoothed_path, planner):
                     current_path = smoothed_path
                     best_valid_path = current_path.copy()
                     accepted = True
                 # Try Unsmoothed
                 elif self._is_collision_free(updated_path, planner):
                     print(f"[STOMP] Smoothing caused collision. Using unsmoothed update.")
                     current_path = updated_path
                     best_valid_path = current_path.copy()
                     accepted = True
                 else:
                     # Both collided.
                     # We can either reject the update or accept strictly better cost even if invalid?
                     # For safety, rejection is better, but might get stuck.
                     # Let's revert to previous valid if available, OR keep updated if it's "less bad"?
                     # But cost function is binary 1e9.
                     print(f"[STOMP] Update caused collision. Skipping update for this iter.")
                     # current_path remains unchanged
                     pass
                 
                 if iter_num % 10 == 0:
                     print(f"[STOMP] Iter {iter_num}: Min Cost={min_cost:.2f}")
                     
                 current_std_dev *= 0.95
            else:
                 current_std_dev *= 0.5
        
        # Return best valid path if we have one, otherwise current (even if invalid)
        if best_valid_path is not None:
             return [p for p in best_valid_path]
        else:
             print("[STOMP] Warning: Could not find any collision-free path.")
             return [p for p in current_path]

