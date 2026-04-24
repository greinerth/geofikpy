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