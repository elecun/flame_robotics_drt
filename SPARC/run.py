import argparse
import configparser
import html
import os
import time
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import viser
import viser.theme
from viser.extras import ViserUrdf


PROJECT_DIR = Path(__file__).resolve().parent
URDF_DIRECTORY = PROJECT_DIR / "resource" / "urdf"
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

    control_tabs = server.gui.add_tab_group()
    control_tabs.add_tab("Simulation")
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
                        joint_status_handles[joint_index].value = slider_handle.value

                bind_joint_slider(slider, index)

    with control_tabs.add_tab("Settings"):
        with server.gui.add_folder("Visualization"):
            collision_obb_checkbox = server.gui.add_checkbox(
                "Show collision model",
                initial_value=show_collision_obb,
                hint="Display collision geometry from the loaded URDF.",
            )

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
