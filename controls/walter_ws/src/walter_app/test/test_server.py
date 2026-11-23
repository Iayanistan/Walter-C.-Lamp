"""The procedure-builder API, against a real server on a free port (no ROS graph needed)."""

import json
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from walter_app import server

STATIONS = Path(__file__).resolve().parents[2] / "walter_skills" / "config" / "stations.yaml"


@pytest.fixture
def base(tmp_path, monkeypatch):
    monkeypatch.setenv("WALTER_STATIONS", str(STATIONS))
    monkeypatch.setenv("WALTER_PROCEDURES", str(tmp_path))
    srv = server.make_server("127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def call(method, url, body=None):
    req = urllib.request.Request(url, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_page_and_catalogue(base):
    with urllib.request.urlopen(base + "/") as r:
        assert b"Walter procedure builder" in r.read()
    _, skills = call("GET", base + "/api/skills")
    assert {"pick", "place", "verify"} <= {s["name"] for s in skills}
    _, stations = call("GET", base + "/api/stations")
    assert any(s["name"] == "hotplate" and s["tagged"] for s in stations)


def test_save_load_and_reject(base, tmp_path):
    good = {"steps": [{"pick": {"station": "rack_1"}}, {"place": {"station": "hotplate"}}]}
    code, res = call("PUT", base + "/api/procedures/move_flask", good)
    assert code == 200 and (tmp_path / "move_flask.yaml").exists()
    _, names = call("GET", base + "/api/procedures")
    assert names == ["move_flask"]
    _, loaded = call("GET", base + "/api/procedures/move_flask")
    assert loaded["steps"] == good["steps"] and loaded["name"] == "move_flask"

    code, res = call("PUT", base + "/api/procedures/bad", {"steps": [{"pick": {"station": "oven"}}]})
    assert code == 400 and "unknown station" in res["error"]
    assert not (tmp_path / "bad.yaml").exists()
    code, res = call("PUT", base + "/api/procedures/..%2Fescape", good)
    assert code == 400


def test_compile_reports_steps(base):
    code, res = call("POST", base + "/api/compile", {"steps": [{"home": {}}]})
    assert code == 200 and res["steps"][0]["type"] == "move"
    code, res = call("POST", base + "/api/run", {"name": "missing"})
    assert code == 404
