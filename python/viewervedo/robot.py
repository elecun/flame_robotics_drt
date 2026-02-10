"""
Robot URDF Loader for VedoVisualizer
@note
- Uses urdf_parser.urdf.URDF to load and parse URDF files
- Leverages built-in forward kinematics from the URDF parser
- Converts trimesh meshes to vedo actors for rendering
"""

import os
import numpy as np
import vedo
import trimesh
from typing import List
from urdf_parser.urdf import URDF
from util.logger.console import ConsoleLogger


def _make_base_transform(base_pose: List[float]) -> np.ndarray:
    """Create a 4x4 homogeneous transformation matrix from [x, y, z, roll, pitch, yaw]."""
    x, y, z = base_pose[0], base_pose[1], base_pose[2]
    r, p, yaw = base_pose[3], base_pose[4], base_pose[5]

    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return T


def _trimesh_to_vedo(tm: trimesh.Trimesh, name: str = "") -> vedo.Mesh:
    """Convert a trimesh.Trimesh object to a vedo.Mesh."""
    mesh = vedo.Mesh([tm.vertices, tm.faces])
    mesh.name = name
    return mesh


class RobotModel:
    """Loads a URDF via urdf_parser and produces vedo actors for each link."""

    def __init__(self, name: str, urdf_path: str, base_pose: List[float] = None):
        """
        Args:
            name: Robot name identifier
            urdf_path: Absolute path to the URDF file
            base_pose: [x, y, z, roll, pitch, yaw] base position in world frame
        """
        self.name = name
        self.urdf_path = urdf_path
        self.base_pose = base_pose or [0, 0, 0, 0, 0, 0]
        self.actors: List[vedo.Mesh] = []
        self._console = ConsoleLogger.get_logger()

    def load(self) -> List[vedo.Mesh]:
        """Load the URDF and create vedo mesh actors with FK transforms applied."""
        urdf = URDF.load(self.urdf_path)
        self._console.debug(f"[Robot:{self.name}] Loaded URDF '{urdf.name}' "
                            f"({len(urdf.links)} links, {len(urdf.joints)} joints)")

        # Compute FK at zero configuration (default joint angles)
        fk = urdf.link_fk(cfg=None)

        # Base transform for placing the robot in world coordinates
        base_T = _make_base_transform(self.base_pose)

        for link, link_transform in fk.items():
            # World transform = base_T * FK_transform
            world_T = base_T @ link_transform

            # Process visual meshes
            for visual in link.visuals:
                visual_origin = visual.origin  # 4x4 transform (mesh-local offset)
                geom = visual.geometry

                for tm in geom.meshes:
                    tm_copy = tm.copy()

                    # Apply mesh scale if present
                    if geom.mesh is not None and geom.mesh.scale is not None:
                        S = np.eye(4)
                        S[:3, :3] = np.diag(geom.mesh.scale)
                        tm_copy.apply_transform(S)

                    # Apply visual origin (local mesh offset)
                    if visual_origin is not None:
                        tm_copy.apply_transform(visual_origin)

                    # Apply world transform (FK + base)
                    tm_copy.apply_transform(world_T)

                    # Convert trimesh to vedo
                    actor = _trimesh_to_vedo(tm_copy, f"{self.name}_{link.name}")
                    actor.c('steelblue').alpha(0.9)
                    self.actors.append(actor)

        self._console.info(f"[Robot:{self.name}] Created {len(self.actors)} mesh actors")
        return self.actors


def load_robots_from_config(config: dict) -> List[vedo.Mesh]:
    """Load all robot models defined in config and return their vedo actors.

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

    return all_actors
