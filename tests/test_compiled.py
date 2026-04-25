from __future__ import annotations

import numpy as np

from geofikpy import geofik


def test_fk() -> None:
    """Test foward kinematics
    """
    test = np.zeros(7, dtype=float)
    out = geofik.franka_fk(test)
    assert out.shape[0] == 4
    assert out.shape[1] == 4

def test_ik() -> None:
    """Test inverse kinematics
    """
    q = np.array([0.0, -np.pi / 4.0, 0.0, -3 * np.pi / 4.0, 0.0, np.pi / 2.0,  np.pi / 4.0])
    desired = geofik.franka_fk(q)
    out = geofik.franka_ik_q7(desired[:3, -1], desired[:3, :3], 0.0)
    np.testing.assert_allclose(desired, out)