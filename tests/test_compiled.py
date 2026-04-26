from __future__ import annotations

import numpy as np

from geofikpy import geofik


def test_fk() -> None:
    """Test forward kinematics"""
    test = np.zeros(7, dtype=float)
    out = geofik.franka_fk(test)
    assert out.shape[0] == 4
    assert out.shape[1] == 4


def test_ik() -> None:
    """Test inverse kinematics"""
    q = np.array(
        [0.0, -np.pi / 4.0, 0.0, -3 * np.pi / 4.0, 0.0, np.pi / 2.0, np.pi / 4.0]
    )
    desired = geofik.franka_fk(q)

    _sols, idx = geofik.franka_ik_q7(
        desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 4.0
    )
    _sols = _sols[:idx]

    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = geofik.franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    _sols, idx = geofik.franka_ik_q6(
        desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 2.0
    )
    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = geofik.franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    _sols, idx = geofik.franka_ik_q4(
        desired[:3, -1], np.ravel(desired[:3, :3]), -3 * np.pi / 4.0
    )
    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = geofik.franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    theta = geofik.franka_swivel(q)

    _sols, idx = geofik.franka_ik_swivel(
        desired[:3, -1], np.ravel(desired[:3, :3]), theta
    )
    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = geofik.franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )
