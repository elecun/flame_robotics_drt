import open3d as o3d
from typing import Dict, Optional

class GeoContainer:
    def __init__(self):
        self._geometries: Dict[str, dict] = {}
        
        # device selection (Priority: CUDA > CPU)
        self.device = o3d.core.Device("cuda:0") if o3d.core.cuda.is_available() else o3d.core.Device("cpu:0")
        print(f"GeoContainer using device: {self.device}")

        # Add default origin coordinate
        # Create legacy and convert to tensor (as simplistic creation might not be available in tensor API for coords yet)
        origin_legacy = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500.0, origin=[0, 0, 0])
        origin = o3d.t.geometry.TriangleMesh.from_legacy(origin_legacy, device=self.device)
        self.add_geometry("origin_coordinate", origin)

        # Add default ground plane
        # 10000 x 10000 size, thin box
        ground_legacy = o3d.geometry.TriangleMesh.create_box(width=10000.0, height=10000.0, depth=0.1)
        ground_legacy.translate([-5000.0, -5000.0, -0.1])
        ground_legacy.paint_uniform_color([0.5, 0.5, 0.5]) # Gray color
        ground_legacy.compute_vertex_normals()
        
        ground = o3d.t.geometry.TriangleMesh.from_legacy(ground_legacy, device=self.device)
        self.add_geometry("ground", ground)

    def add_geometry(self, name: str, geometry, visibility: bool = True):
        """
        Add a geometry to the container.
        
        Args:
            name (str): Unique name of the geometry
            geometry: The Open3D geometry object (Tensor or Legacy)
            visibility (bool): Initial visibility state
        """
        # Ensure geometry is on the correct device if it's a tensor geometry
        if isinstance(geometry, o3d.t.geometry.Geometry):
             if geometry.device != self.device:
                 geometry = geometry.to(self.device)
        
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

