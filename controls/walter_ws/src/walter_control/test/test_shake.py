import numpy as np

from walter_control.shake import peak_velocity, shake_trajectory


def test_shake_starts_and_ends_at_rest():
    q0 = [0, 0.1, 0.1, 0, 1.37, 0]
    t, q = shake_trajectory(q0, 4, np.radians(10), 2.5, 30.0)
    assert np.isclose(t[-1], 30.0)
    np.testing.assert_allclose(q[-1], q0, atol=1e-9)
    assert np.all(q[:, [0, 1, 2, 3, 5]] == np.array(q0)[[0, 1, 2, 3, 5]])
    assert np.abs(q[:, 4] - q0[4]).max() <= np.radians(10) + 1e-9


def test_peak_velocity_within_irb120_j5_rating():
    assert peak_velocity(np.radians(10), 2.5) < 5.58505
