import argparse
import configparser
import os
import sys
import time
import urllib.parse
import viser
import viser.theme


def create_title_svg_data_uri(title: str) -> str:
    """Create a data URI for an SVG displaying the title text in the titlebar."""
    # Approximate width based on character count
    width = max(100, len(title) * 14 + 20)
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="32" viewBox="0 0 {width} 32">'
        f'<text x="0" y="22" font-family="-apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica, Arial, sans-serif" '
        f'font-size="18" font-weight="700" fill="#ffffff" letter-spacing="0.5px">{title}</text>'
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

    # Read system section
    title = config.get("system", "title", fallback="SPARC")
    host = config.get("system", "host", fallback="0.0.0.0")
    port = config.getint("system", "port", fallback=8080)

    print(f"Starting SPARC system with config: {args.config}")
    print(f"  Title: {title}")
    print(f"  Host: {host}")
    print(f"  Port: {port}")

    # Initialize Viser server
    server = viser.ViserServer(host=host, port=port)

    # Configure theme: Dark mode (black background), Title in top-left titlebar
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
        dark_mode=True,
        show_logo=False,
    )

    # Set GUI panel label
    server.gui.set_panel_label(title)

    # Origin coordinate frame (x, y, z axes)
    server.scene.world_axes.visible = True

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping SPARC system...")
        server.stop()


if __name__ == "__main__":
    main()
