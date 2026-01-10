import numpy as np
import sys
import os

# Adjust path to import OptimizerBase
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../pluginbase')))
from optimizerbase import OptimizerBase

class Stomp(OptimizerBase):
    def __init__(self):
        super().__init__()
        self.num_iterations = 50
        self.num_samples = 20
        self.std_dev = 0.5  # Standard deviation for noise
        self.smoothing_factor = 0.1 # Very basic control cost

    def optimize(self, path: list, planner) -> list:
        if not path or len(path) < 3:
            print("[STOMP] Path too short to optimize.")
            return path

        current_path = np.array(path)
        n_waypoints = len(current_path)
        dim = current_path.shape[1]
        
        # Determine fixed start and goal
        start_pose = current_path[0].copy()
        goal_pose = current_path[-1].copy()

        # Adaptive Noise Scale
        diffs = np.diff(current_path, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        avg_dist = np.mean(dists)
        current_std_dev = avg_dist * 0.5
        print(f"[STOMP] Initializing with avg_step_dist={avg_dist:.2f}, std_dev={current_std_dev:.2f}")

        # Smoothing Kernel (Simple Box or Gaussian-like)
        kernel_size = 5
        kernel = np.ones(kernel_size) / kernel_size

        for iter_num in range(self.num_iterations):
            # 1. Generate Correlated Noise (Smooth Noise)
            # Independent noise
            raw_noise = np.random.normal(0, current_std_dev, (self.num_samples, n_waypoints, dim))
            
            # Smooth the noise along waypoint axis
            # Only apply to inner points, start/goal noise should be 0 (enforced later)
            noise = np.zeros_like(raw_noise)
            for i in range(self.num_samples):
                for d in range(dim):
                    # Pad to keep size
                    noise[i, :, d] = np.convolve(raw_noise[i, :, d], kernel, mode='same')
            
            # Enforce zero noise at start/goal
            noise[:, 0, :] = 0
            noise[:, -1, :] = 0
            
            # 2. Candidates
            candidates = np.tile(current_path, (self.num_samples, 1, 1))
            candidates += noise
            
            # 3. Evaluate Costs
            costs = np.zeros(self.num_samples)
            
            for i in range(self.num_samples):
                cand = candidates[i]
                
                # Smoothness Cost (Jerk / Acceleration) - using 2nd derivative approx
                # vel = np.diff(cand, axis=0)
                # acc = np.diff(vel, axis=0)
                # smoothness_cost = np.sum(np.sum(acc**2, axis=1)) 
                
                # Distance Cost (Minimizing length)
                diffs = np.diff(cand, axis=0)
                length_cost = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
                
                # Obstacle Cost
                obstacle_cost = 0.0
                in_collision = False
                
                # Fast check (only waypoints)
                # To be precise we should check segments
                for j in range(n_waypoints - 1):
                    if planner._check_collision(cand[j], cand[j+1]):
                         in_collision = True
                         break
                
                if in_collision:
                    obstacle_cost = 1e9
                
                costs[i] = length_cost + obstacle_cost
            
            # 4. Update Path
            best_idx = np.argmin(costs)
            min_cost = costs[best_idx]
            
            if min_cost < 1e9:
                 # Probability Weights
                 exp_cost = np.exp(-10.0 * (costs - min_cost) / (np.max(costs) - min_cost + 1e-6))
                 weights = exp_cost / np.sum(exp_cost)
                 
                 # Weighted Average Update
                 weighted_change = np.zeros_like(current_path)
                 for i in range(self.num_samples):
                     weighted_change += (candidates[i] - current_path) * weights[i]
                     
                 # Apply update
                 current_path += weighted_change
                 
                 # 5. Explicit Smoothing with Edge Padding
                 # Pad properly to avoid pulling boundaries to zero
                 pad_size = kernel_size // 2
                 for d in range(dim):
                     col = current_path[:, d]
                     # Pad with edge values
                     padded = np.pad(col, (pad_size, pad_size), mode='edge')
                     # Convolve valid part
                     smoothed = np.convolve(padded, kernel, mode='valid')
                     current_path[:, d] = smoothed
                 
                 # Re-enforce Start/Goal
                 current_path[0] = start_pose
                 current_path[-1] = goal_pose
                 
                 if iter_num % 10 == 0:
                     print(f"[STOMP] Iter {iter_num}: Min Cost={min_cost:.2f}")
                     
                 # Decay noise
                 current_std_dev *= 0.95
            else:
                 # All collided? Try reducing noise
                 current_std_dev *= 0.5
            
        return [p for p in current_path]

