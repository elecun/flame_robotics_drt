# SPARC
Python 3.10+ / FastAPI / React / Ant Design / three.js.

## Run
Activate your virtual environment and run:
```
python -m pip install -r requirements.txt
python run.py
```
Open http://localhost:8080 or http://HOST_IP:8080. The existing venv has dependencies installed.
Frontend bundles, URDF and STL assets are local; normal startup requires no network or Node.js.
On another OS, recreate the Python virtual environment (virtual environments themselves are not portable).

## Architecture
- core/server.py: authoritative in-memory state, validated REST changes, WebSocket snapshots and async processing demo.
- core/frontend.jsx: Ant Design collapsible Sider/Header/Content and three.js viewport.
- core/interfaces.py: algorithm and device boundaries for future integration.
- resource/dist: bundled browser runtime, including React, Ant Design, three.js and URDF loader.
- resources/urdf: supplied model.urdf and its STL meshes (original path preserved).
- tools: independent sidebar utility modules and device adapters.
- work: user data, logs and test screenshots.

Exactly one mode is active: offline (default), simulation, online or custom. Mode selection does not connect physical hardware. State is shared across all clients and reconnects; restarting the server resets it. Run one Uvicorn worker: multiple workers would need an external shared-state store. The async task demonstrates shared backend progress; it does not perform physical simulation. Orbit cameras remain local to each viewer.

## Development and verification
```
npm ci
npm run build
python -m pytest -q
# With python run.py running:
npx playwright install chromium
node tests/browser.cjs
```
Browser verification covers provided URDF/STL loading, two-client mode/progress updates, desktop/compact sizes and absence of external requests. Python tests cover late joins, mode/parameter validation, async progress and private-path exclusion.

## Manual joints
Double-click a visible URDF link to select it. Only one link is highlighted red at a time; selecting another restores the previous material. Double-click empty space or click Close to clear selection.

With an adjustable link selected, left-drag in the viewport adjusts its nearest movable ancestor joint. Pointer rays are projected onto the world-space joint rotation plane (revolute/continuous) or an axis-containing drag plane (prismatic). Joint values follow the resulting angle or axis distance, preserving the initial grab offset. Near-parallel views require changing the camera angle. URDF limits apply and changes synchronize to all clients. Fixed links can be selected but cannot move without a movable ancestor. Selection and camera are local to each viewer.

Without a selection, left-drag orbits the camera. Right-drag pans and the wheel zooms. Numeric joint controls remain available. Browser tests: `node tests/joints.cjs` and `node tests/drag.cjs`, with the server running.

Geometry tests: `node tests/joint-geometry.mjs` (world-space positions, camera/zoom independence, angle continuity and degenerate views).

## View controls
Escape clears the selected link and restores its material. Header File → Reload reloads the page. View → Show Origin Coordinates toggles the global (0,0,0) axes, initially enabled and shared across clients. The corner orientation widget remains available independently; its labels render without depth occlusion and a fixed XYZ legend remains visible at every camera angle. Tests: `node tests/view-controls.cjs`.

Header File/View use Ant Design Dropdown with click triggers and downward indicators. Click a trigger to open its menu. Existing Reload and origin visibility actions are preserved.

File → Load PCD opens the local file picker and renders ASCII/binary/compressed PCD using the local three.js loader. The latest cloud replaces the previous cloud and Fit view frames it. Clouds remain local to the selecting browser.
File → Settings edits processing speed and origin visibility. Apply broadcasts without writing a file; Save & Close atomically writes work/settings.json, applies and closes. Saved settings reload on server startup. Process contains Pose Finder, Path Planner and Optimizer categories with disabled placeholders until algorithms are registered.

View → Show Robot Coord. toggles XYZ frames at every URDF joint, including fixed joints. X is red, Y green, Z blue. Frames follow joint transforms and do not intercept model picking. Visibility defaults off and is shared with current and newly connected viewers. Geometry test: `node tests/robot-coordinates.mjs`.
