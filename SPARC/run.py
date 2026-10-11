import argparse
import configparser
import html
import os
import time
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import viser
import viser.theme
from viser.extras import ViserUrdf


PROJECT_DIR = Path(__file__).resolve().parent
URDF_DIRECTORY = PROJECT_DIR / "resource" / "urdf"
PCD_WORK_DIRECTORY = PROJECT_DIR / "work" / "pcd"
WORLD_AXIS_LENGTH = 0.5
NAVY_BRAND_COLOR = (10, 43, 84)


def create_title_svg_data_uri(title: str) -> str:
    """Create a simple white title wordmark for the navy Viser header."""
    safe_title = html.escape(title)
    width = max(100, len(title) * 14 + 20)
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="32" viewBox="0 0 {width} 32">'
        f'<text x="0" y="22" font-family="-apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica, Arial, sans-serif" '
        f'font-size="18" font-weight="700" fill="#ffffff" letter-spacing="0.5px">{safe_title}</text>'
        f'</svg>'
    )
    return f"data:image/svg+xml;utf8,{urllib.parse.quote(svg)}"


def load_config(config_path: str) -> configparser.ConfigParser:
    """Load configuration file using configparser."""
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    config = configparser.ConfigParser()
    config.read(config_path, encoding="utf-8")
    return config


def parse_hex_color(value: str) -> tuple[int, int, int]:
    """Parse a CSS-style #RGB or #RRGGBB color into an RGB tuple."""
    color = value.strip().lstrip("#")
    if len(color) == 3:
        color = "".join(component * 2 for component in color)
    if len(color) != 6:
        raise ValueError("must use #RGB or #RRGGBB format")

    try:
        return tuple(int(color[index : index + 2], 16) for index in range(0, 6, 2))
    except ValueError as error:
        raise ValueError("must use #RGB or #RRGGBB format") from error


def resolve_urdf_path(urdf_filename: str) -> Path:
    """Resolve a configured URDF filename within SPARC's URDF resources."""
    path = (URDF_DIRECTORY / urdf_filename).resolve()
    try:
        path.relative_to(URDF_DIRECTORY.resolve())
    except ValueError as error:
        raise ValueError("URDF file must be located under resource/urdf") from error

    if path.suffix.lower() != ".urdf":
        raise ValueError("URDF file must have a .urdf extension")
    if not path.is_file():
        raise FileNotFoundError(f"URDF model file not found: {path}")
    return path


def get_actuated_joint_settings(urdf_path: Path) -> dict[str, tuple[float, float, float, str]]:
    """Read slider ranges from URDF instead of hard-coding robot joint names."""
    settings: dict[str, tuple[float, float, float, str]] = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        joint_type = joint.get("type")
        if joint_type not in {"continuous", "prismatic", "revolute"}:
            continue

        limit = joint.find("limit")
        if joint_type == "prismatic":
            lower = float(limit.get("lower", "-1")) if limit is not None else -1.0
            upper = float(limit.get("upper", "1")) if limit is not None else 1.0
            settings[joint.get("name")] = (lower, upper, 0.001, "m")
        else:
            lower = float(limit.get("lower", str(-np.pi))) if limit is not None else -np.pi
            upper = float(limit.get("upper", str(np.pi))) if limit is not None else np.pi
            settings[joint.get("name")] = (lower, upper, 0.01, "rad")
    return settings


def load_point_cloud(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load vertices (and optional PLY vertex colors) from PCD or PLY."""
    if path.suffix.lower() == ".ply":
        asset = trimesh.load(path, process=False)
        geometries = asset.geometry.values() if isinstance(asset, trimesh.Scene) else (asset,)
        point_sets, color_sets = [], []
        for geometry in geometries:
            vertices = np.asarray(geometry.vertices, dtype=np.float32)
            if not len(vertices):
                continue
            point_sets.append(vertices)
            colors = getattr(getattr(geometry, "visual", None), "vertex_colors", None)
            color_sets.append(
                np.asarray(colors[:, :3], dtype=np.uint8)
                if colors is not None and len(colors) == len(vertices)
                else np.full((len(vertices), 3), (80, 180, 230), dtype=np.uint8)
            )
        if not point_sets:
            raise ValueError("The PLY file contains no vertices.")
        return np.vstack(point_sets), np.vstack(color_sets)

    data = path.read_bytes()
    header, payload = {}, b""
    offset = 0
    while offset < len(data):
        line_end = data.find(b"\n", offset)
        if line_end < 0:
            raise ValueError("Invalid PCD header.")
        line = data[offset:line_end].decode("ascii", errors="strict").strip()
        offset = line_end + 1
        if not line or line.startswith("#"):
            continue
        key, *values = line.split()
        header[key.upper()] = values
        if key.upper() == "DATA":
            payload = data[offset:]
            break
    if header.get("DATA", [""])[0].lower() == "binary_compressed":
        raise ValueError("Compressed PCD files are not supported; use ASCII or binary PCD.")

    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD must provide x, y, and z fields.")
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    point_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
    if point_count <= 0:
        raise ValueError("PCD contains no points.")
    scalar_offsets = np.cumsum([0, *counts[:-1]])
    field_indices = {field: int(offset) for field, offset in zip(fields, scalar_offsets)}
    data_type = header.get("DATA", [""])[0].lower()
    def decode_packed_rgb(values: np.ndarray | None) -> np.ndarray:
        if values is None:
            return np.full((point_count, 3), (80, 180, 230), dtype=np.uint8)
        values = np.asarray(values).reshape(-1)
        packed = (
            values.astype(np.float32).view(np.uint32)
            if values.dtype.kind == "f"
            else values.astype(np.uint32)
        )
        return np.column_stack(
            ((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)
        ).astype(np.uint8)

    color_field = "rgb" if "rgb" in field_indices else "rgba" if "rgba" in field_indices else None
    if data_type == "ascii":
        values = np.fromstring(payload.decode("ascii"), sep=" ", dtype=np.float32)
        columns = sum(counts)
        if len(values) < point_count * columns:
            raise ValueError("PCD data is shorter than its header declares.")
        rows = values[: point_count * columns].reshape(point_count, columns)
        points = rows[:, [field_indices["x"], field_indices["y"], field_indices["z"]]]
        colors = decode_packed_rgb(rows[:, field_indices[color_field]] if color_field else None)
    elif data_type == "binary":
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        if len(sizes) != len(fields) or len(types) != len(fields):
            raise ValueError("Invalid binary PCD field metadata.")
        type_codes = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4"}
        try:
            dtype = np.dtype([(field, type_codes[(kind, size)], count) for field, kind, size, count in zip(fields, types, sizes, counts)])
        except KeyError as error:
            raise ValueError("Unsupported binary PCD field type.") from error
        rows = np.frombuffer(payload, dtype=dtype, count=point_count)
        if len(rows) != point_count:
            raise ValueError("PCD data is shorter than its header declares.")
        points = np.column_stack((rows["x"], rows["y"], rows["z"])).astype(np.float32)
        colors = decode_packed_rgb(rows[color_field] if color_field else None)
    else:
        raise ValueError("PCD DATA must be ascii or binary.")
    return points, colors


def point_cloud_destination(filename: str) -> Path:
    """Resolve an upload destination under the project work directory."""
    safe_name = Path(filename).name
    if Path(safe_name).suffix.lower() not in {".pcd", ".ply"}:
        raise ValueError("Only .pcd and .ply files can be uploaded.")
    PCD_WORK_DIRECTORY.mkdir(parents=True, exist_ok=True)
    return PCD_WORK_DIRECTORY / safe_name


def store_uploaded_point_cloud(destination: Path, content: bytes) -> Path:
    """Store already-validated upload bytes at the selected destination."""
    destination.write_bytes(content)
    return destination


def add_global_axis_labels(scene: viser.SceneApi) -> None:
    """Add X, Y, and Z labels just beyond the global axes' arrow tips."""
    label_distance = WORLD_AXIS_LENGTH + 0.08
    for axis, position in (
        ("X", (label_distance, 0.0, 0.0)),
        ("Y", (0.0, label_distance, 0.0)),
        ("Z", (0.0, 0.0, label_distance)),
    ):
        scene.add_label(
            f"/WorldAxes/{axis}_label",
            axis,
            position=position,
            font_size_mode="scene",
            font_scene_height=0.12,
            depth_test=False,
            anchor="center-center",
        )


def main():
    parser = argparse.ArgumentParser(description="SPARC System")
    parser.add_argument(
        "--config",
        type=str,
        default="default.cfg",
        help="Path to configuration file (default: default.cfg)",
    )
    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config)

    # Read system and visualization settings.
    title = config.get("system", "title", fallback="SPARC")
    host = config.get("system", "host", fallback="0.0.0.0")
    port = config.getint("system", "port", fallback=8080)
    urdf_filename = config.get("visualization", "urdf_file", fallback="model.urdf")
    background_color = config.get("visualization", "background_color", fallback="#ffffff")

    try:
        urdf_path = resolve_urdf_path(urdf_filename)
        background_rgb = parse_hex_color(background_color)
        show_collision_obb = config.getboolean(
            "visualization", "show_collision_obb", fallback=False
        )
    except (FileNotFoundError, ValueError) as error:
        parser.error(str(error))

    print(f"Starting SPARC system with config: {args.config}")
    print(f"  Title: {title}")
    print(f"  Host: {host}")
    print(f"  Port: {port}")
    print(f"  URDF: {urdf_path}")
    print(f"  Background color: {background_color}")
    print(f"  Show collision OBB: {show_collision_obb}")

    # Initialize Viser server
    server = viser.ViserServer(host=host, port=port)

    # Configure a light UI and a fixed right control panel.
    title_svg_uri = create_title_svg_data_uri(title)
    server.gui.configure_theme(
        titlebar_content=viser.theme.TitlebarConfig(
            image=viser.theme.TitlebarImage(
                image_url_light=title_svg_uri,
                image_url_dark=title_svg_uri,
                image_alt=title,
                href=None,
            ),
            buttons=None,
        ),
        control_width="large",
        dark_mode=False,
        show_logo=False,
        show_share_button=False,
        brand_color=NAVY_BRAND_COLOR,
    )

    # A 1x1 image provides a solid, configurable scene background color.
    server.scene.set_background_image(
        np.array([[background_rgb]], dtype=np.uint8), format="png"
    )

    # Load visual meshes and collision geometry. model.urdf collision elements
    # are OBB boxes and are rendered as translucent red meshes.
    robot = ViserUrdf(
        server,
        urdf_path,
        root_node_name="/robot",
        load_collision_meshes=True,
        collision_mesh_color_override=(1.0, 0.15, 0.15, 0.35),
    )
    joint_names = robot.get_actuated_joint_names()
    joint_settings = get_actuated_joint_settings(urdf_path)
    joint_values = np.zeros(len(joint_names))
    for index, joint_name in enumerate(joint_names):
        lower, upper, _, _ = joint_settings.get(
            joint_name, (-np.pi, np.pi, 0.01, "rad")
        )
        joint_values[index] = np.clip(0.0, lower, upper)

    # Apply the initial configuration so fixed-joint transforms take effect.
    robot.update_cfg(joint_values)
    robot.show_collision = show_collision_obb

    # Use editable mesh nodes for the visual model so transparency can change live.
    urdf_model = robot._urdf
    visual_scene = urdf_model.scene
    visual_mesh_count = len(visual_scene.geometry)
    collision_mesh_handles = robot._meshes[visual_mesh_count:]
    for mesh_handle in robot._meshes[:visual_mesh_count]:
        mesh_handle.remove()

    def get_visual_node_name(frame_name: str) -> str:
        frames = []
        while frame_name != visual_scene.graph.base_frame:
            frames.append(frame_name)
            frame_name = visual_scene.graph.transforms.parents[frame_name]
        return "/".join(("/robot/visual", *frames[::-1]))

    visual_mesh_handles = []
    visual_mesh_base_opacities = []
    for frame_name, mesh in visual_scene.geometry.items():
        parent_frame = visual_scene.graph.transforms.parents[frame_name]
        mesh_transform = urdf_model.get_transform(frame_name, parent_frame)
        mesh = mesh.copy()
        mesh.apply_transform(mesh_transform)
        rgba = np.asarray(mesh.visual.main_color, dtype=np.float32)
        base_opacity = float(rgba[3] / 255.0) if len(rgba) > 3 else 1.0
        visual_mesh_handles.append(
            server.scene.add_mesh_simple(
                get_visual_node_name(frame_name),
                mesh.vertices,
                mesh.faces,
                color=tuple(int(channel) for channel in rgba[:3]),
                opacity=base_opacity,
            )
        )
        visual_mesh_base_opacities.append(base_opacity)

    collision_mesh_base_opacities = [
        1.0 if mesh_handle.opacity is None else mesh_handle.opacity
        for mesh_handle in collision_mesh_handles
    ]

    def set_model_transparency(transparency: float) -> None:
        remaining_opacity = 1.0 - transparency
        for mesh_handle, base_opacity in zip(
            visual_mesh_handles, visual_mesh_base_opacities
        ):
            mesh_handle.opacity = base_opacity * remaining_opacity
        for mesh_handle, base_opacity in zip(
            collision_mesh_handles, collision_mesh_base_opacities
        ):
            mesh_handle.opacity = base_opacity * remaining_opacity

    movable_joint_types = {"continuous", "prismatic", "revolute"}
    movable_joint_names = [
        joint.get("name")
        for joint in ET.parse(urdf_path).getroot().findall("joint")
        if joint.get("type") in movable_joint_types
    ]
    joint_coordinate_root = server.scene.add_frame(
        "/JointCoordinates", show_axes=False, visible=False
    )
    joint_coordinate_frames = {
        joint_name: server.scene.add_frame(
            f"/JointCoordinates/{joint_name}",
            axes_length=0.15,
            axes_radius=0.006,
            visible=True,
        )
        for joint_name in movable_joint_names
    }
    joint_coordinate_labels = {
        joint_name: server.scene.add_label(
            f"/JointCoordinates/{joint_name}/label",
            joint_name,
            position=(0.08, 0.08, 0.08),
            font_size_mode="screen",
            depth_test=False,
            anchor="center-left",
        )
        for joint_name in movable_joint_names
    }

    def update_joint_coordinate_frames() -> None:
        urdf_model = robot._urdf
        for joint_name, frame_handle in joint_coordinate_frames.items():
            joint = urdf_model.joint_map[joint_name]
            parent_transform = urdf_model.get_transform(
                joint.parent, frame_from=urdf_model.base_link
            )
            joint_origin = np.eye(4) if joint.origin is None else joint.origin
            joint_transform = parent_transform @ joint_origin
            frame_handle.position = joint_transform[:3, 3]
            frame_handle.wxyz = viser.transforms.SO3.from_matrix(
                joint_transform[:3, :3]
            ).wxyz

    update_joint_coordinate_frames()

    # Dock main panel to the right (fixed layout)
    server.gui.main_panel.dock_right()

    # Set GUI panel label
    server.gui.set_panel_label(title)

    active_mode = ["Simulation"]
    pending_mode_modal = [None]
    updating_mode_selection = [False]

    def request_mode_change(_event) -> None:
        if updating_mode_selection[0]:
            return
        requested_mode = mode_selector.value
        # Keep the visible selection on the confirmed mode until the user approves.
        updating_mode_selection[0] = True
        mode_selector.value = active_mode[0]
        updating_mode_selection[0] = False
        if requested_mode == active_mode[0] or pending_mode_modal[0] is not None:
            return

        modal = server.gui.add_modal("Important: mode change")
        pending_mode_modal[0] = modal
        with modal:
            server.gui.add_markdown(
                f"Change operating mode from **{active_mode[0]}** to **{requested_mode}**?"
            )
            confirm_button = server.gui.add_button("확인", color="blue")
            cancel_button = server.gui.add_button("취소")

        @confirm_button.on_click
        def _confirm_mode_change(_) -> None:
            active_mode[0] = requested_mode
            updating_mode_selection[0] = True
            mode_selector.value = requested_mode
            updating_mode_selection[0] = False
            modal.close()
            pending_mode_modal[0] = None

        @cancel_button.on_click
        def _cancel_mode_change(_) -> None:
            modal.close()
            pending_mode_modal[0] = None

    mode_selector = server.gui.add_button_group(
        "Mode", ("Simulation", "Auto", "Manual"), hint="Select the operating mode."
    )
    mode_selector.on_click(request_mode_change)

    loaded_point_cloud = [None]
    point_cloud_size = [0.015]
    point_cloud_offset = [np.zeros(3, dtype=np.float32)]
    control_tabs = server.gui.add_tab_group()
    with control_tabs.add_tab("Simulation"):
        with server.gui.add_folder("PCD"):
            point_cloud_upload = server.gui.add_upload_button(
                "PCD/PLY File Open",
                mime_type=".pcd,.ply",
                hint="Choose a local PCD or PLY point-cloud file.",
            )
        with server.gui.add_folder("Algorithm"):
            pose_finder_dropdown = server.gui.add_dropdown(
                "Pose Finder",
                ("Select...",),
                initial_value="Select...",
            )
            path_planner_dropdown = server.gui.add_dropdown(
                "Path Planner",
                ("Select...",),
                initial_value="Select...",
            )
            optimizer_dropdown = server.gui.add_dropdown(
                "Optimizer",
                ("Select...",),
                initial_value="Select...",
            )
        run_button = server.gui.add_button("Run", color="blue")
        stop_button = server.gui.add_button("Stop", color="red")
    control_tabs.add_tab("Auto")
    with control_tabs.add_tab("Manual"):
        with server.gui.add_folder("Control"):
            for index, joint_name in enumerate(joint_names):
                lower, upper, step, unit = joint_settings.get(
                    joint_name, (-np.pi, np.pi, 0.01, "rad")
                )
                slider = server.gui.add_slider(
                    joint_name,
                    lower,
                    upper,
                    step,
                    float(joint_values[index]),
                    hint=f"{joint_name} ({unit})",
                )

                def bind_joint_slider(slider_handle, joint_index: int) -> None:
                    @slider_handle.on_update
                    def _update_joint(_event) -> None:
                        joint_values[joint_index] = slider_handle.value
                        robot.update_cfg(joint_values)
                        update_joint_coordinate_frames()
                        joint_status_handles[joint_index].value = slider_handle.value

                bind_joint_slider(slider, index)

    with control_tabs.add_tab("Settings"):
        with server.gui.add_folder("Visualization"):
            collision_obb_checkbox = server.gui.add_checkbox(
                "Show collision model",
                initial_value=show_collision_obb,
                hint="Display collision geometry from the loaded URDF.",
            )
            joint_coordinates_checkbox = server.gui.add_checkbox(
                "Show joint coordinates",
                initial_value=False,
                hint="Display coordinate axes at every movable joint origin.",
            )
            model_transparency_slider = server.gui.add_slider(
                "Model transparency",
                min=0.0,
                max=1.0,
                step=0.01,
                initial_value=0.0,
                hint="Adjust the transparency of URDF visual and collision meshes.",
            )
        with server.gui.add_folder("PCD"):
            point_size_input = server.gui.add_number(
                "Point Size", 0.015, min=0.001, max=1.0, step=0.001
            )
            point_size_set_button = server.gui.add_button("Set")
            point_offset_input = server.gui.add_vector3(
                "XYZ Offset", (0.0, 0.0, 0.0), step=0.001
            )
            point_offset_set_button = server.gui.add_button("Set")

    # A movable status panel with separate robot and system views.
    status_panel = server.gui.add_panel()
    joint_status_handles = []
    with status_panel.add_tab("status"):
        status_tabs = server.gui.add_tab_group()
        with status_tabs.add_tab("Robot"):
            for index, joint_name in enumerate(joint_names):
                _, _, step, unit = joint_settings.get(
                    joint_name, (-np.pi, np.pi, 0.01, "rad")
                )
                joint_status_handles.append(
                    server.gui.add_number(
                        f"{joint_name} ({unit})",
                        float(joint_values[index]),
                        step=step,
                        disabled=True,
                    )
                )
        status_tabs.add_tab("System")
    status_panel.dock_left()
    status_panel.set_width(300)

    # Keep the collision option in Settings, rather than the Simulation tab.
    # The callback is intentionally independent of the current tab selection.
    @collision_obb_checkbox.on_update
    def _toggle_collision_obb(_) -> None:
        robot.show_collision = collision_obb_checkbox.value

    @joint_coordinates_checkbox.on_update
    def _toggle_joint_coordinates(_) -> None:
        joint_coordinate_root.visible = joint_coordinates_checkbox.value

    @model_transparency_slider.on_update
    def _set_model_transparency(_) -> None:
        set_model_transparency(model_transparency_slider.value)

    def show_point_cloud_error(error: Exception) -> None:
        modal = server.gui.add_modal("파일을 열 수 없습니다")
        with modal:
            server.gui.add_markdown(f"파일을 열 수 없습니다.\n\n`{error}`")
            close_button = server.gui.add_button("확인")

        @close_button.on_click
        def _close_error_modal(_) -> None:
            modal.close()

    def render_point_cloud(
        points: np.ndarray, colors: np.ndarray, source_path: Path
    ) -> None:
        # The robot cell uses metres. Scanner PCD files are exported in millimetres.
        scale = 0.001 if source_path.suffix.lower() == ".pcd" else 1.0
        points = np.asarray(points, dtype=np.float32) * scale
        valid = np.isfinite(points).all(axis=1)
        points, colors = points[valid], np.asarray(colors, dtype=np.uint8)[valid]
        if not len(points):
            raise ValueError("The point cloud has no finite points.")
        if loaded_point_cloud[0] is not None:
            loaded_point_cloud[0].remove()
        loaded_point_cloud[0] = server.scene.add_point_cloud(
            "/UploadedPointCloud",
            points=points,
            colors=colors,
            point_size=point_cloud_size[0],
            point_shape="circle",
            precision="float32",
            position=point_cloud_offset[0],
        )
        lower, upper = points.min(axis=0), points.max(axis=0)
        center = (lower + upper) / 2
        distance = max(float(np.linalg.norm(upper - lower)), 1.0)
        camera_position = center + np.array((distance, -distance, distance * 0.65))
        server.initial_camera.look_at = center
        server.initial_camera.position = camera_position
        for client in server.get_clients().values():
            client.camera.look_at = center
            client.camera.position = camera_position
        print(f"Loaded point cloud: {source_path} ({len(points):,} points)")

    def load_upload_to_destination(uploaded_file, destination: Path) -> None:
        # Validate the incoming bytes before replacing an existing saved file.
        temporary_path = destination.with_name(f".{destination.name}.upload")
        try:
            temporary_path.write_bytes(uploaded_file.content)
            points, colors = load_point_cloud(temporary_path)
        finally:
            if temporary_path.exists():
                temporary_path.unlink()
        store_uploaded_point_cloud(destination, uploaded_file.content)
        render_point_cloud(points, colors, destination)

    @point_cloud_upload.on_upload
    def _load_uploaded_point_cloud(_) -> None:
        try:
            uploaded_file = point_cloud_upload.value
            destination = point_cloud_destination(uploaded_file.name)
            if not destination.exists():
                load_upload_to_destination(uploaded_file, destination)
                return

            modal = server.gui.add_modal("동일한 파일이 이미 존재합니다")
            with modal:
                server.gui.add_markdown(
                    "동일한 파일이 이미 존재합니다. 덮어쓸까요?"
                )
                yes_button = server.gui.add_button("Yes", color="blue")
                no_button = server.gui.add_button("No")

            @yes_button.on_click
            def _overwrite_file(_) -> None:
                try:
                    load_upload_to_destination(uploaded_file, destination)
                    modal.close()
                except (OSError, ValueError, UnicodeDecodeError) as error:
                    modal.close()
                    show_point_cloud_error(error)

            @no_button.on_click
            def _open_existing_file(_) -> None:
                try:
                    points, colors = load_point_cloud(destination)
                    render_point_cloud(points, colors, destination)
                    modal.close()
                except (OSError, ValueError, UnicodeDecodeError) as error:
                    modal.close()
                    show_point_cloud_error(error)
        except (OSError, ValueError, UnicodeDecodeError) as error:
            show_point_cloud_error(error)

    @point_size_set_button.on_click
    def _set_point_size(_) -> None:
        point_cloud_size[0] = point_size_input.value
        if loaded_point_cloud[0] is not None:
            loaded_point_cloud[0].point_size = point_cloud_size[0]

    @point_offset_set_button.on_click
    def _set_point_offset(_) -> None:
        point_cloud_offset[0] = np.asarray(point_offset_input.value, dtype=np.float32)
        if loaded_point_cloud[0] is not None:
            loaded_point_cloud[0].position = point_cloud_offset[0]

    # Origin coordinate frame (x, y, z axes)
    server.scene.world_axes.visible = True
    add_global_axis_labels(server.scene)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping SPARC system...")
        server.stop()


if __name__ == "__main__":
    main()
