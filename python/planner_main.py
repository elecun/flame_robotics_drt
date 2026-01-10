import open3d as o3d
import numpy as np
import importlib
import os
import sys
import argparse
import glob
import inspect

# Add Paths
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'core/pluginbase')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'core/algorithms')))

from plannerbase import PlannerBase

def load_plugins():
    plugins = {}
    algo_dir = os.path.join(os.path.dirname(__file__), 'core/algorithms')
    sys.path.append(algo_dir)
    
    for file in os.listdir(algo_dir):
        if file.endswith('.py') and not file == '__init__.py':
            module_name = file[:-3]
            try:
                module = importlib.import_module(module_name)
                for name, obj in inspect.getmembers(module):
                    if inspect.isclass(obj) and issubclass(obj, PlannerBase) and obj is not PlannerBase:
                        plugins[module_name] = obj
            except Exception as e:
                print(f"Error loading {module_name}: {e}")
                
    return plugins

from optimizerbase import OptimizerBase

def load_optimizers():
    optimizers = {}
    opt_dir = os.path.join(os.path.dirname(__file__), 'core/algorithms/optimization')
    sys.path.append(opt_dir)
    
    if os.path.exists(opt_dir):
        for file in os.listdir(opt_dir):
            if file.endswith('.py') and not file == '__init__.py':
                module_name = file[:-3]
                try:
                    module = importlib.import_module(module_name)
                    for name, obj in inspect.getmembers(module):
                        if inspect.isclass(obj) and issubclass(obj, OptimizerBase) and obj is not OptimizerBase:
                            # Use module name or class name? user asked for 'pruning', 'stomp'
                            # Let's map module name (scrip name) or class name to lower case
                            optimizers[module_name] = obj
                            optimizers[name.lower()] = obj
                except Exception as e:
                    print(f"Error loading optimizer {module_name}: {e}")
    return optimizers

# ... (create_coordinate_frame_mesh, generate_random_pose, create_sphere_marker restored)

def create_coordinate_frame_mesh(pose, size=1.0):
    # Pose: [x, y, z, r, p, y]
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    R = o3d.geometry.get_rotation_matrix_from_xyz(pose[3:])
    mesh.rotate(R, center=(0, 0, 0))
    mesh.translate(pose[:3])
    return mesh

def generate_random_pose(mesh, distance_offset):
    min_b = mesh.get_min_bound()
    max_b = mesh.get_max_bound()
    center = (min_b + max_b) / 2.0
    extent = np.linalg.norm(max_b - min_b) / 2.0
    
    # Random direction
    while True:
        direction = np.random.uniform(-1, 1, 3)
        if np.linalg.norm(direction) > 0.1:
            direction /= np.linalg.norm(direction)
            break
            
    # Position
    # Using 'extent' as a rough radius.
    # User asked for 'distance from model'.
    # Start = Center + Direction * (extent + distance_offset)
    pos = center + direction * (extent + distance_offset)
    
    # Random Orientation (RPY)
    orient = np.random.uniform(-np.pi, np.pi, 3)
    
    return np.concatenate((pos, orient))

def create_sphere_marker(pose, color, radius=0.5):
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh.paint_uniform_color(color)
    mesh.translate(pose[:3])
    return mesh

def main():
    parser = argparse.ArgumentParser(description="Path Planner Main")
    parser.add_argument('--algorithm', type=str, default='task_space_rrt', help='Algorithm to use')
    parser.add_argument('--stl', type=str, default='sample/PIPE NO.1_fill.stl', help='Path to STL file')
    
    # Start: Default None to trigger randomization
    parser.add_argument('--start', nargs=6, type=float, default=None, help='Start pose x y z r p y')
    # Goal: Default explicit value (reverted to previous default)
    parser.add_argument('--goal', nargs=6, type=float, default=[-150, -300, 0, 0, 0, 0], help='Goal pose x y z r p y')
    parser.add_argument('--show_coord', action='store_true', help='Show coordinate frames for waypoints')
    parser.add_argument('--optimize', type=str, default=None, help='Optimization method (e.g., path_pruning, stomp)')
    args = parser.parse_args()
    
    # ... (STL Loading, Start/Goal gen remain same)
    
    stl_path = os.path.abspath(args.stl)
    if not os.path.exists(stl_path):
        print(f"STL file not found: {stl_path}")
        return
        
    mesh = o3d.io.read_triangle_mesh(stl_path)
    mesh.compute_vertex_normals()
    
    # Generate Start if not provided
    if args.start is None:
        print("Start pose not provided. Generating randomly (dist ~ 300)...")
        args.start = generate_random_pose(mesh, 300.0)
        print(f"Generated Start: {args.start}")
    else:
        args.start = np.array(args.start)
        
    args.goal = np.array(args.goal)
    
    # Load Plugins
    plugins = load_plugins()
    # ... (Plugin loading logic remains same)
    if args.algorithm not in plugins and args.algorithm not in [p.split('.')[-1] for p in plugins]:
        print(f"Algorithm {args.algorithm} not found. Available: {list(plugins.keys())}")
        # ... try find ...
        found = False
        for k in plugins:
            if args.algorithm in k:
                args.algorithm = k
                found = True
                break
        if not found:
            return

    planner_class = plugins[args.algorithm]
    planner = planner_class()
    print(f"Loaded planner: {planner_class.__name__}")
    
    # Add to planner
    planner.add_static_object(mesh)
    
    # ... (Dynamic Config Adjustment remains same)
    min_b = mesh.get_min_bound()
    max_b = mesh.get_max_bound()
    
    # ... (Bounds and Step Size logic remains same)
    # Re-implementing logic compactly if simple patch, but replacing 'main' block
    # Need to be careful to preserve context or rewrite it all.
    # Given the replace block size, I should probably replace the whole main function or large chunk.
    # Let's write the whole main function logic replacement to be safe and clean.

    # Calculate appropriate bounds with padding
    center = (min_b + max_b) / 2.0
    extent = max_b - min_b
    max_dim = np.max(extent)
    padding = max_dim * 2.0 
    
    if hasattr(planner, 'bounds') and isinstance(planner.bounds, dict):
        all_points = np.array([args.start[:3], args.goal[:3], min_b, max_b])
        p_min = np.min(all_points, axis=0) - 50.0 
        p_max = np.max(all_points, axis=0) + 50.0
        
        print(f"Auto-adjusting workspace bounds to: {p_min} ~ {p_max}")
        planner.bounds.update({
            "x_min": float(p_min[0]), "x_max": float(p_max[0]),
            "y_min": float(p_min[1]), "y_max": float(p_max[1]),
            "z_min": float(p_min[2]), "z_max": float(p_max[2])
        })
        
    if hasattr(planner, 'step_size'):
        dist = np.linalg.norm(args.start[:3] - args.goal[:3])
        new_step = max(1.0, dist / 50.0)
        print(f"Auto-adjusting step_size to: {new_step}")
        planner.step_size = new_step
        
    if hasattr(planner, 'max_iter'):
        planner.max_iter = 5000
        print(f"Set max_iter to: {planner.max_iter}")

    # Generate Path
    print(f"Generating path from {args.start} to {args.goal}...")
    
    path = planner.generate(args.start, args.goal)
    
    optimized_path = None
    if path:
        print(f"Path found with {len(path)} waypoints")
        if args.optimize:
            # Load Optimizers
            optimizers = load_optimizers()
            if args.optimize in optimizers:
                print(f"Optimizing path using {args.optimize}...")
                optimizer_class = optimizers[args.optimize]
                optimizer = optimizer_class()
                optimized_path = optimizer.optimize(path, planner)
                print(f"Optimized path has {len(optimized_path)} waypoints")
            else:
                print(f"Optimizer {args.optimize} not found. Available: {list(optimizers.keys())}")
    else:
        print("No path found!")
        
    # Visualization
    vis_elements = []
    
    # STL
    mesh.paint_uniform_color([0.5, 0.5, 0.5])
    vis_elements.append(mesh)
    
    # Calculate bounds for scaling
    bbox_diag = np.linalg.norm(max_b - min_b)
    scale = bbox_diag * 0.2 if bbox_diag > 0 else 1.0
    print(f"Stl Bounds: Min={min_b}, Max={max_b}")
    print(f"Visualization Scale: {scale}")
    
    # Start/Goal
    vis_elements.append(create_coordinate_frame_mesh(args.start, size=scale*0.2))
    vis_elements.append(create_sphere_marker(args.start, [1, 0, 0], radius=scale*0.02)) # Red
    vis_elements.append(create_coordinate_frame_mesh(args.goal, size=scale*0.2))
    vis_elements.append(create_sphere_marker(args.goal, [0, 0, 1], radius=scale*0.02)) # Blue
    
    # Original Path (Red)
    if path:
        points = [p[:3] for p in path]
        lines = [[i, i+1] for i in range(len(points)-1)]
        colors = [[1, 0, 0] for _ in range(len(lines))] # Red
        
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        vis_elements.append(line_set)
        
        if args.show_coord and not optimized_path: # Show coords for original if no optimized
             for p in path:
                vis_elements.append(create_coordinate_frame_mesh(p, size=scale*0.3))

    # Optimized Path (Blue)
    if optimized_path:
        points_opt = [p[:3] for p in optimized_path]
        lines_opt = [[i, i+1] for i in range(len(points_opt)-1)]
        colors_opt = [[0, 0, 1] for _ in range(len(lines_opt))] # Blue
        
        line_set_opt = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points_opt),
            lines=o3d.utility.Vector2iVector(lines_opt),
        )
        line_set_opt.colors = o3d.utility.Vector3dVector(colors_opt)
        vis_elements.append(line_set_opt)
        
        if args.show_coord:
             for p in optimized_path:
                vis_elements.append(create_coordinate_frame_mesh(p, size=scale*0.3))
        
        # Visualize Start/Goal of Optimized Path explicitly as requested
        # Making them slightly larger or distinct to confirm they match (or mismatch if bug persisted)
        vis_elements.append(create_coordinate_frame_mesh(optimized_path[0], size=scale*0.25))
        vis_elements.append(create_coordinate_frame_mesh(optimized_path[-1], size=scale*0.25))

    print(f"Visualizing {len(vis_elements)} elements...")
    o3d.visualization.draw_geometries(vis_elements, window_name="Path Planner", width=1024, height=768, left=50, top=50, point_show_normal=False, mesh_show_wireframe=False, mesh_show_back_face=False)

if __name__ == "__main__":
    main()
