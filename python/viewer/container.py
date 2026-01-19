import open3d as o3d
from typing import Dict, Optional

class GeoContainer:
    def __init__(self):
        self._geometries: Dict[str, dict] = {}
        
        # Add default origin coordinate
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500.0, origin=[0, 0, 0])
        self.add_geometry("origin_coordinate", origin)

        # Add default ground plane
        # 10000 x 10000 size, thin box
        ground = o3d.geometry.TriangleMesh.create_box(width=10000.0, height=10000.0, depth=0.1)
        # Center the ground at origin (x, y) and put it slightly below z=0
        ground.translate([-5000.0, -5000.0, -0.1])
        ground.paint_uniform_color([0.5, 0.5, 0.5]) # Gray color
        
        # Calculate normals for proper shading
        ground.compute_vertex_normals()
        
        self.add_geometry("ground", ground)

    def add_geometry(self, name: str, geometry, visibility: bool = True):
        """
        Add a geometry to the container.
        
        Args:
            name (str): Unique name of the geometry
            geometry: The Open3D geometry object
            visibility (bool): Initial visibility state
        """
        self._geometries[name] = {
            "geometry": geometry,
            "visible": visibility
        }

    def get_geometry(self, name: str) -> Optional[dict]:
        """Get a specific geometry by name"""
        return self._geometries.get(name)

    def get_geometries(self) -> Dict[str, dict]:
        """Get all geometries"""
        return self._geometries

    def remove_geometry(self, name: str):
        """Remove a geometry by name"""
        if name in self._geometries:
            del self._geometries[name]
