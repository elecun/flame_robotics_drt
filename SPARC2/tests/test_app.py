from fastapi.testclient import TestClient
from core.server import create_app

def test_shared_state():
    with TestClient(create_app()) as client:
        with client.websocket_connect("/ws") as a, client.websocket_connect("/ws") as b:
            assert a.receive_json()["mode"] == "offline"
            b.receive_json()
            assert client.patch("/api/state", json={"mode":"simulation","speed":1.5}).status_code == 200
            first, second = a.receive_json(), b.receive_json()
            assert first == second and first["speed"] == 1.5
            with client.websocket_connect("/ws") as late:
                assert late.receive_json() == first
            assert client.patch("/api/state", json={"mode":"invalid"}).status_code == 422
            assert client.patch("/api/state", json={"speed":-1}).status_code == 422

def test_async_progress():
    with TestClient(create_app()) as client:
        with client.websocket_connect("/ws") as ws:
            ws.receive_json()
            assert client.post("/api/process").json()["accepted"]
            assert ws.receive_json()["processing"]
            assert not client.post("/api/process").json()["accepted"]
            states = [ws.receive_json() for _ in range(10)]
            assert states[-1]["progress"] == 100
            assert not states[-1]["processing"]

def test_assets_and_boundaries():
    with TestClient(create_app()) as client:
        for path in ["/", "/resource/dist/app.js", "/resources/urdf/model.urdf"]:
            assert client.get(path).status_code == 200
        for path in ["/run.py","/.task","/resource/%2e%2e/run.py"]:
            assert client.get(path).status_code == 404

def test_joint_update_limits_and_late_join():
    with TestClient(create_app()) as client:
        specs = client.get("/api/joints").json()
        name, spec = next(iter(specs.items()))
        value = (spec["lower"] + spec["upper"]) / 2 if spec["lower"] is not None else .5
        with client.websocket_connect("/ws") as a, client.websocket_connect("/ws") as b:
            a.receive_json()
            b.receive_json()
            assert client.patch("/api/joints/"+name, json={"value":value}).status_code == 200
            first = a.receive_json()
            assert first == b.receive_json()
            assert first["joints"][name] == value
            with client.websocket_connect("/ws") as late:
                assert late.receive_json()["joints"][name] == value
        assert client.patch("/api/joints/missing", json={"value":0}).status_code == 404
        if spec["upper"] is not None:
            assert client.patch("/api/joints/"+name, json={"value":spec["upper"]+1}).status_code == 422
        assert client.patch("/api/joints/"+name, json={"value":"NaN"}).status_code == 422

def test_origin_visibility_shared():
    with TestClient(create_app()) as client:
        with client.websocket_connect("/ws") as ws:
            assert ws.receive_json()["show_origin"] is True
            assert client.patch("/api/state",json={"show_origin":False}).status_code == 200
            assert ws.receive_json()["show_origin"] is False
            with client.websocket_connect("/ws") as late:
                assert late.receive_json()["show_origin"] is False

def test_settings_persistence(tmp_path):
    path = tmp_path / "settings.json"
    with TestClient(create_app(path)) as client:
        assert client.patch("/api/state",json={"speed":1.3}).status_code == 200
        assert not path.exists()
        assert client.put("/api/settings",json={"speed":1.7,"show_origin":False}).status_code == 200
        assert client.put("/api/settings",json={"speed":9,"show_origin":True}).status_code == 422
    with TestClient(create_app(path)) as client:
        state = client.get("/api/state").json()
        assert state["speed"] == 1.7 and state["show_origin"] is False

def test_robot_coordinates_shared():
    with TestClient(create_app()) as client:
        with client.websocket_connect("/ws") as ws:
            assert ws.receive_json()["show_robot_coordinates"] is False
            client.patch("/api/state",json={"show_robot_coordinates":True})
            assert ws.receive_json()["show_robot_coordinates"] is True
            with client.websocket_connect("/ws") as late:
                assert late.receive_json()["show_robot_coordinates"] is True
