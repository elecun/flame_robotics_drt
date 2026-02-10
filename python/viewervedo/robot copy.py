"""
Robot URDF Loader for VedoVisualizer
@note
- Parses URDF XML to extract kinematic chain and mesh references
- Loads STL meshes and applies forward kinematics transforms
- Returns vedo Assembly objects for rendering
"""

import os
import xml.etree.ElementTree as ET
import numpy as np
import vedo
from typing import List, Dict, Optional, Tuple
from util.logger.console import ConsoleLogger


def _rpy_to_matrix(rpy: List[float]) -> np.ndarray:
    """Convert roll-pitch-yaw angles to a 3x3 rotation matrix."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    
    return Rz @ Ry @ Rx


def _make_transform(xyz: List[float], rpy: List[float]) -> np.ndarray:
    """Create a 4x4 homogeneous transformation matrix from xyz + rpy."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


class URDFLink:
    """Represents a single link from a URDF file."""
    def __init__(self, name: str):
        self.name = name
        self.visual_mesh_path: Optional[str] = None
        self.visual_origin_xyz = [0, 0, 0]
        self.visual_origin_rpy = [0, 0, 0]
        self.visual_scale = [1, 1, 1]
        self.collision_mesh_path: Optional[str] = None
        self.material_color: Optional[List[float]] = None


class URDFJoint:
    """Represents a single joint from a URDF file."""
    def __init__(self, name: str, joint_type: str):
        self.name = name
        self.joint_type = joint_type  # revolute, fixed, etc.
        self.parent_link: str = ""
        self.child_link: str = ""
        self.origin_xyz = [0, 0, 0]
        self.origin_rpy = [0, 0, 0]
        self.axis = [0, 0, 1]


class RobotModel:
    """Parses a URDF file and produces vedo actors for each link."""
    
    def __init__(self, name: str, urdf_path: str, base_pose: List[float] = None):
        """
        Args:
            name: Robot name identifier
            urdf_path: Path to the URDF file
            base_pose: [x, y, z, roll, pitch, yaw] base position
        """
        self.name = name
        self.urdf_path = urdf_path
        self.urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        self.base_pose = base_pose or [0, 0, 0, 0, 0, 0]
        
        self.links: Dict[str, URDFLink] = {}
        self.joints: List[URDFJoint] = []
        self.actors: List[vedo.Mesh] = []
        
        self._console = ConsoleLogger.get_logger()
    
    def load(self) -> List[vedo.Mesh]:
        """Parse URDF and create vedo mesh actors with FK transforms applied."""
        self._parse_urdf()
        self._build_actors()
        return self.actors
    
    def _parse_urdf(self):
        """Parse the URDF XML file."""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        
        # Parse links
        for link_elem in root.findall('link'):
            link_name = link_elem.get('name', '')
            link = URDFLink(link_name)
            
            # Visual mesh
            visual = link_elem.find('visual')
            if visual is not None:
                origin = visual.find('origin')
                if origin is not None:
                    xyz_str = origin.get('xyz', '0 0 0')
                    rpy_str = origin.get('rpy', '0 0 0')
                    link.visual_origin_xyz = [float(v) for v in xyz_str.split()]
                    link.visual_origin_rpy = [float(v) for v in rpy_str.split()]
                
                geom = visual.find('geometry')
                if geom is not None:
                    mesh_elem = geom.find('mesh')
                    if mesh_elem is not None:
                        link.visual_mesh_path = mesh_elem.get('filename', '')
                        scale_str = mesh_elem.get('scale', '')
                        if scale_str:
                            link.visual_scale = [float(v) for v in scale_str.split()]
                
                material = visual.find('material')
                if material is not None:
                    color_elem = material.find('color')
                    if color_elem is not None:
                        rgba_str = color_elem.get('rgba', '')
                        if rgba_str:
                            link.material_color = [float(v) for v in rgba_str.split()]
            
            # Collision mesh (fallback)
            collision = link_elem.find('collision')
            if collision is not None:
                geom = collision.find('geometry')
                if geom is not None:
                    mesh_elem = geom.find('mesh')
                    if mesh_elem is not None:
                        link.collision_mesh_path = mesh_elem.get('filename', '')
            
            self.links[link_name] = link
        
        # Parse joints
        for joint_elem in root.findall('joint'):
            joint_name = joint_elem.get('name', '')
            joint_type = joint_elem.get('type', 'fixed')
            joint = URDFJoint(joint_name, joint_type)
            
            origin = joint_elem.find('origin')
            if origin is not None:
                xyz_str = origin.get('xyz', '0 0 0')
                rpy_str = origin.get('rpy', '0 0 0')
                joint.origin_xyz = [float(v) for v in xyz_str.split()]
                joint.origin_rpy = [float(v) for v in rpy_str.split()]
            
            parent = joint_elem.find('parent')
            if parent is not None:
                joint.parent_link = parent.get('link', '')
            
            child = joint_elem.find('child')
            if child is not None:
                joint.child_link = child.get('link', '')
            
            axis_elem = joint_elem.find('axis')
            if axis_elem is not None:
                axis_str = axis_elem.get('xyz', '0 0 1')
                joint.axis = [float(v) for v in axis_str.split()]
            
            self.joints.append(joint)
        
        self._console.debug(f"[Robot:{self.name}] Parsed {len(self.links)} links, {len(self.joints)} joints")
    
    def _compute_link_transforms(self) -> Dict[str, np.ndarray]:
        """Compute world transform for each link at zero joint angles."""
        # Build parent->child mapping
        child_to_joint: Dict[str, URDFJoint] = {}
        parent_children: Dict[str, List[str]] = {}
        
        for joint in self.joints:
            child_to_joint[joint.child_link] = joint
            parent_children.setdefault(joint.parent_link, []).append(joint.child_link)
        
        # Find root link (not a child of any joint)
        all_children = set(child_to_joint.keys())
        root_links = [name for name in self.links if name not in all_children]
        
        # Base transform
        bx, by, bz = self.base_pose[0], self.base_pose[1], self.base_pose[2]
        br, bp, byaw = self.base_pose[3], self.base_pose[4], self.base_pose[5]
        base_T = _make_transform([bx, by, bz], [br, bp, byaw])
        
        # BFS to compute transforms
        transforms: Dict[str, np.ndarray] = {}
        queue = []
        for root in root_links:
            transforms[root] = base_T.copy()
            queue.append(root)
        
        while queue:
            parent = queue.pop(0)
            for child in parent_children.get(parent, []):
                joint = child_to_joint[child]
                joint_T = _make_transform(joint.origin_xyz, joint.origin_rpy)
                transforms[child] = transforms[parent] @ joint_T
                queue.append(child)
        
        return transforms
    
    def _resolve_mesh_path(self, link: URDFLink) -> Optional[str]:
        """Resolve the mesh file path, preferring STL collision meshes."""
        # Try collision STL first (better vedo support)
        if link.collision_mesh_path:
            path = os.path.join(self.urdf_dir, link.collision_mesh_path)
            if os.path.exists(path):
                return path
        
        # Try visual mesh (may be .dae which vedo can't load)
        if link.visual_mesh_path:
            # Try STL version of visual path
            visual_path = link.visual_mesh_path
            stl_path = visual_path.replace('/visual/', '/collision/').replace('.dae', '.stl')
            full_stl = os.path.join(self.urdf_dir, stl_path)
            if os.path.exists(full_stl):
                return full_stl
            
            # Try original visual path
            full_visual = os.path.join(self.urdf_dir, visual_path)
            if os.path.exists(full_visual):
                return full_visual
        
        return None
    
    def _build_actors(self):
        """Build vedo Mesh actors for each link with transforms applied."""
        transforms = self._compute_link_transforms()
        
        for link_name, link in self.links.items():
            mesh_path = self._resolve_mesh_path(link)
            if mesh_path is None:
                continue
            
            try:
                mesh = vedo.load(mesh_path)
                if mesh is None:
                    self._console.warning(f"[Robot:{self.name}] Failed to load mesh: {mesh_path}")
                    continue
                
                # Apply visual origin transform (mesh-local offset)
                if any(v != 0 for v in link.visual_origin_xyz + link.visual_origin_rpy):
                    vis_T = _make_transform(link.visual_origin_xyz, link.visual_origin_rpy)
                    mesh.apply_transform(vis_T)
                
                # Apply scale if specified
                if link.visual_scale != [1, 1, 1]:
                    mesh.scale(link.visual_scale)
                
                # Apply FK world transform
                if link_name in transforms:
                    mesh.apply_transform(transforms[link_name])
                
                # Apply color
                if link.material_color:
                    r, g, b = link.material_color[0], link.material_color[1], link.material_color[2]
                    a = link.material_color[3] if len(link.material_color) > 3 else 1.0
                    mesh.c([r, g, b]).alpha(a)
                else:
                    mesh.c('steelblue').alpha(0.9)
                
                mesh.name = f"{self.name}_{link_name}"
                self.actors.append(mesh)
                
            except Exception as e:
                self._console.error(f"[Robot:{self.name}] Error loading {link_name}: {e}")


def load_robots_from_config(config: dict) -> List[vedo.Mesh]:
    """Load all robot models defined in the config and return their vedo actors.
    
    Config format:
        "urdf": [
            {"name": "robot1", "path": "urdf/robot.urdf", "base": [x, y, z, r, p, yaw]},
            ...
        ]
    """
    console = ConsoleLogger.get_logger()
    all_actors = []
    
    urdf_entries = config.get("urdf", [])
    if not urdf_entries:
        return all_actors
    
    root_path = config.get("root_path", "")
    
    for entry in urdf_entries:
        name = entry.get("name", "unknown")
        path = entry.get("path", "")
        base = entry.get("base", [0, 0, 0, 0, 0, 0])
        
        # Resolve path relative to root
        if root_path:
            full_path = os.path.join(str(root_path), path)
        else:
            full_path = path
        
        if not os.path.exists(full_path):
            console.error(f"[Robot] URDF file not found: {full_path}")
            continue
        
        console.info(f"[Robot] Loading {name} from {full_path}")
        robot = RobotModel(name=name, urdf_path=full_path, base_pose=base)
        actors = robot.load()
        all_actors.extend(actors)
        console.info(f"[Robot] Loaded {len(actors)} meshes for {name}")
    
    return all_actors
