import re
from pathlib import Path

import numpy as np
import pytest

from walter_fr3 import DryRunArm, HELPERS, Halted, Operator, ProcedureError, Walter, calc
from walter_fr3.arm import ArmError, check_limits
from walter_fr3.d4607 import d4607
from walter_fr3.run import run_procedure

ROOT = Path(__file__).resolve().parents[1]


def walter(**kw):
    return Walter(DryRunArm(verbose=False), operator=Operator(auto=True), **kw)


def test_fig1_isotherm():
    iso = calc.fit_isotherm([(0.012, 932), (0.020, 964), (0.032, 1040)])
    assert iso.iodine_number == pytest.approx(964, rel=0.02)
    assert iso.slope == pytest.approx(0.111, abs=0.005)


def test_equations():
    assert calc.dilution_factor() == pytest.approx(2.2)                 # the standard's own example
    assert calc.residual_c(0.1, 10.0) == pytest.approx(0.02)
    assert calc.x_over_m(0.1, 0.1, 10.0, 1.0) == pytest.approx(1269.3 - 2.2 * 12.693 * 10)
    m = calc.carbon_dosage(0.1, 950, 0.02)
    assert calc.x_over_m(0.1, 0.1, 10.0, m) == pytest.approx(950)       # eq. 8 inverts eq. 6


def test_full_d4607_dry_run():
    w = walter()
    iso = d4607(w)
    assert iso.iodine_number == pytest.approx(964, rel=0.02) and not iso.warnings
    assert not w.run_log.deviations and w.holding is None
    timed = [e for e in w.run_log.events if e["event"] == "timed_step"]
    assert [e["label"] for e in timed] == ["boil"] * 3 + ["shake"] * 3 and all(e["ok"] for e in timed)
    kinds = [e["event"] for e in w.run_log.events]
    assert kinds.count("swirl") == 6 and kinds.count("pour") == 3 and kinds.count("dispense") == 3


def test_state_checks():
    w = walter()
    with pytest.raises(ProcedureError):
        w.place("hotplate")                    # nothing in the gripper
    with pytest.raises(ProcedureError):
        w.swirl()
    with pytest.raises(ProcedureError):
        w.boil()                               # nothing on the hot plate
    with pytest.raises(ProcedureError):
        w.move_to("rack_1", "pour")            # no such taught pose


def test_swirl_and_shake_return_to_start():
    w = walter()
    w.items["rack_1"] = "flask_1"
    w.pick("rack_1")
    q0 = w.arm.joints()
    w.swirl(seconds=4)
    assert np.allclose(w.arm.joints(), q0, atol=1e-6)
    with pytest.raises(ArmError):
        w.shake(amplitude_deg=30, frequency_hz=4)   # faster than the FR3's joints are rated for


def test_limits_and_halt():
    with pytest.raises(ArmError):
        check_limits([0, 2.0, 0, 0, 0, 0])
    w = walter()
    w.arm.halt("spill detected")
    with pytest.raises(Halted):
        w.home()


def test_builder_yaml_runs():
    w = walter()
    run_procedure(w, {"start": [{"item": "flask_1", "at": "rack_1"}], "steps": [
        {"pick": {"station": "rack_1", "spec": "10.4", "recover": "slide the rack back"}},
        {"swirl": {"seconds": 5}},
        {"place": {"station": "hotplate"}},
        {"boil": {}},
        {"place": {"station": "cool_pad"}},
        {"home": None},
    ]})
    assert w.items == {"cool_pad": "flask_1"}
    with pytest.raises(ProcedureError):
        run_procedure(walter(), {"steps": [{"juggle": {}}]})


def test_builder_blocks_match_helpers():
    """The procedure builder must offer exactly the Python helpers, with the same parameter names."""
    html = (ROOT.parent / "procedure_builder_skeleton" / "index.html")
    if not html.exists():
        pytest.skip("builder not next to this package")
    text = html.read_text()
    body = text[text.index("const SKILLS = {"):text.index("const COMMON")]
    blocks = dict(re.findall(r"^  (\w+):\s+\{ label.*?params: \{(.*?)\} \},?$", body, re.M | re.S))
    assert set(blocks) == set(HELPERS)
    import inspect
    for name, params in blocks.items():
        js = set(re.findall(r"(\w+): (?:P\.\w+\(|\{ t:)", params)) - {"recover", "retries"}
        py = set(inspect.signature(getattr(Walter, name)).parameters) - {"self", "quiet", "default"}
        assert js == py, f"{name}: builder {sorted(js)} vs python {sorted(py)}"


class _FakeRPC:
    """Stand-in with the call shapes of FAIRINO's Python SDK, to exercise FairinoArm without a controller."""

    def __init__(self, ip):
        self.q = [-19.9, -98.8, 77.2, -68.4, -90.0, 0.0]
        self.calls = []

    def _rec(self, name, *a, **k):
        self.calls.append((name, a, k))
        return 0

    def ResetAllError(self): return self._rec("ResetAllError")
    def Mode(self, m): return self._rec("Mode", m)
    def RobotEnable(self, s): return self._rec("RobotEnable", s)
    def GetActualJointPosDegree(self, flag=1): return 0, list(self.q)
    def MoveJ(self, joint_pos, tool, user, desc_pos=[0.0] * 6, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0):
        self.q = list(joint_pos)
        return self._rec("MoveJ", vel=vel)
    def ServoMoveStart(self): return self._rec("ServoMoveStart")
    def ServoMoveEnd(self): return self._rec("ServoMoveEnd")
    def ServoJ(self, joint_pos, axisPos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0):
        self.q = list(joint_pos)
        return self._rec("ServoJ")
    def SetToolDO(self, i, s, smooth, block): return self._rec("SetToolDO", i, s)
    def SetDO(self, i, s, smooth, block): return self._rec("SetDO", i, s)
    def StopMotion(self): return self._rec("StopMotion")
    def CloseRPC(self): return self._rec("CloseRPC")


def test_fairino_backend_against_fake_sdk(monkeypatch):
    import sys
    import types
    fake = types.ModuleType("fairino")
    fake.Robot = types.SimpleNamespace(RPC=_FakeRPC)
    monkeypatch.setitem(sys.modules, "fairino", fake)
    from walter_fr3 import FairinoArm
    arm = FairinoArm("192.168.58.2")
    w = Walter(arm, operator=Operator(auto=True))
    w.grip_settle = 0.0
    w.items["rack_1"] = "flask_1"
    w.pick("rack_1")
    w.swirl(seconds=0.2, amplitude_deg=2)
    names = [c[0] for c in arm.r.calls]
    assert names.count("MoveJ") == 3 and "ServoJ" in names and names[-1] == "ServoMoveEnd"
    assert all(1.0 <= c[2]["vel"] <= 30.0 for c in arm.r.calls if c[0] == "MoveJ")
    assert ("SetToolDO", (0, 1), {}) in arm.r.calls
    arm.halt("test")
    assert arm.r.calls[-1][0] == "StopMotion"
