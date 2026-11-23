import numpy as np

from walter_control.chunk_guard import GuardConfig, check_chunk

HOME = np.array([0, 0, 0, 0, np.pi / 2, 0])
CFG = GuardConfig(
    lower=np.array([-2.87979, -1.91986, -1.91986, -2.79253, -2.094395, -6.98132]),
    upper=np.array([2.87979, 1.91986, 1.22173, 2.79253, 2.094395, 6.98132]),
    max_velocity=np.array([4.36332, 4.36332, 4.36332, 5.58505, 5.58505, 7.33038]) * 0.25,
    margin=0.05, max_start_jump=0.15,
    workspace_min=np.array([0.10, -0.45, 0.02]), workspace_max=np.array([0.60, 0.45, 0.55]),
)


def ramp(delta, n=8):
    return HOME + np.outer(np.linspace(1 / n, 1, n), delta)


def test_small_chunk_passes_unchanged():
    chunk = ramp([0.02, 0.02, 0, 0, 0, 0])
    r = check_chunk(HOME, chunk, 1 / 15, CFG)
    assert r.ok and not r.retimed and not r.clamped
    np.testing.assert_allclose(r.times, np.arange(1, 9) / 15)


def test_rejects_start_jump():
    r = check_chunk(HOME, ramp([0.5, 0, 0, 0, 0, 0]) + [0.3, 0, 0, 0, 0, 0], 1 / 15, CFG)
    assert not r.ok and "first point" in r.reason


def test_rejects_nan_and_bad_shape():
    bad = ramp([0.01] * 6)
    bad[3, 2] = np.nan
    assert not check_chunk(HOME, bad, 1 / 15, CFG).ok
    assert not check_chunk(HOME, np.zeros((4, 5)), 1 / 15, CFG).ok


def test_rejects_leaving_workspace():
    # Swinging J1 to 90 deg puts the TCP at x = 0, behind the workspace box's x_min.
    chunk = HOME + np.outer(np.linspace(0.1, np.pi / 2, 16), [1, 0, 0, 0, 0, 0])
    loose = GuardConfig(**{**CFG.__dict__, "max_start_jump": 1.0})
    r = check_chunk(HOME, chunk, 1 / 15, loose)
    assert not r.ok and "workspace" in r.reason


def test_fast_chunk_is_slowed_not_rejected():
    chunk = ramp([0.0, 0.0, 0.0, 0.0, 0.0, 1.2])  # 0.15 rad per step on J6 at 15 Hz
    r = check_chunk(HOME, chunk, 1 / 15, CFG)
    assert r.ok and r.retimed
    speeds = np.abs(np.diff(np.vstack([HOME, r.positions]), axis=0)) / np.diff(np.r_[0, r.times])[:, None]
    assert np.all(speeds <= CFG.max_velocity + 1e-9)


def test_clamps_to_joint_limits():
    q0 = HOME.copy()
    q0[4] = 2.0
    chunk = np.tile(q0, (4, 1))
    chunk[:, 4] = [2.02, 2.05, 2.08, 2.09]
    r = check_chunk(q0, chunk, 1 / 15, CFG)
    assert r.ok and r.clamped
    assert r.positions[:, 4].max() <= 2.094395 - 0.05 + 1e-12
