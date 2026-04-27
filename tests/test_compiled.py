from __future__ import annotations

import numpy as np
import pytest

from geofikpy import (
    J_from_q,
    franka_fk,
    franka_ik_q4,
    franka_ik_q6,
    franka_ik_q7,
    franka_ik_swivel,
    franka_J_ik_q4,
    franka_J_ik_q6,
    franka_J_ik_q7,
    franka_J_ik_swivel,
    franka_swivel,
)


@pytest.fixture
def get_q() -> np.ndarray[float]:
    return np.array(
        [0.0, -np.pi / 4.0, 0.0, -3 * np.pi / 4.0, 0.0, np.pi / 2.0, np.pi / 4.0]
    )


def test_fk(get_q) -> None:
    """Test forward kinematics"""
    test = get_q
    out = franka_fk(test)
    assert out.shape[0] == 4
    assert out.shape[1] == 4


def test_ik(get_q) -> None:
    """Test inverse kinematics"""
    q = get_q
    desired = franka_fk(q)

    _sols, idx = franka_ik_q7(desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 4.0)
    _sols = _sols[:idx]

    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    _sols, idx = franka_ik_q6(desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 2.0)
    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    _sols, idx = franka_ik_q4(
        desired[:3, -1], np.ravel(desired[:3, :3]), -3 * np.pi / 4.0
    )
    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    theta = franka_swivel(q)

    _sols, idx = franka_ik_swivel(desired[:3, -1], np.ravel(desired[:3, :3]), theta)
    sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])

    for sol in sols:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )


def test_j_ik(get_q) -> None:
    """Test jacobian inverse kinematics"""
    q = get_q
    desired = franka_fk(q)
    _jacs, _configs, idx = franka_J_ik_q4(
        desired[:3, -1], np.ravel(desired[:3, :3]), -3 * np.pi / 4.0, joint_angles=True
    )
    jacs = np.array([jac for jac in _jacs[:idx] if not np.any(np.isnan(jac))])
    configs = np.array(
        [config for config in _configs[:idx] if not np.any(np.isnan(config))]
    )

    jacs = np.array([jac for jac in _jacs[:idx] if not np.any(np.isnan(jac))])
    configs = np.array(
        [config for config in _configs[:idx] if not np.any(np.isnan(config))]
    )

    assert jacs.shape[2] == 6
    assert jacs.shape[1] == 7
    assert configs.shape[1] == 7

    for sol in configs:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    _jacs, _configs, idx = franka_J_ik_q6(
        desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 2.0, joint_angles=True
    )

    jacs = np.array([jac for jac in _jacs[:idx] if not np.any(np.isnan(jac))])
    configs = np.array(
        [config for config in _configs[:idx] if not np.any(np.isnan(config))]
    )

    assert jacs.shape[2] == 6
    assert jacs.shape[1] == 7
    assert configs.shape[1] == 7

    for sol in configs:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    _jacs, _configs, idx = franka_J_ik_q4(
        desired[:3, -1], np.ravel(desired[:3, :3]), -3 * np.pi / 4.0, joint_angles=True
    )

    jacs = np.array([jac for jac in _jacs[:idx] if not np.any(np.isnan(jac))])
    configs = np.array(
        [config for config in _configs[:idx] if not np.any(np.isnan(config))]
    )

    assert jacs.shape[2] == 6
    assert jacs.shape[1] == 7
    assert configs.shape[1] == 7

    for sol in configs:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    theta = franka_swivel(q)

    _jacs, _configs, idx = franka_J_ik_swivel(
        desired[:3, -1], np.ravel(desired[:3, :3]), theta, joint_angles=True
    )

    jacs = np.array([jac for jac in _jacs[:idx] if not np.any(np.isnan(jac))])
    configs = np.array(
        [config for config in _configs[:idx] if not np.any(np.isnan(config))]
    )

    assert jacs.shape[2] == 6
    assert jacs.shape[1] == 7
    assert configs.shape[1] == 7

    for sol in configs:
        out = franka_fk(sol)
        (
            np.testing.assert_allclose(out[:3, :3], desired[:3, :3], atol=1e-6),
            f"Invalid rotation. Expected {desired[:3, :3]}, got {out[:3, :3]}!",
        )
        (
            np.testing.assert_allclose(out[:3, -1], desired[:3, -1], atol=1e-6),
            f"Invalid translation. Expected {desired[:3, -1]}, got {out[:3, -1]}!",
        )

    out = franka_J_ik_q4(
        desired[:3, -1], np.ravel(desired[:3, :3]), -3 * np.pi / 4, joint_angles=False
    )
    assert len(out) == 2

    out = franka_J_ik_q6(
        desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 2, joint_angles=False
    )
    assert len(out) == 2

    out = franka_J_ik_q7(
        desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 4, joint_angles=False
    )
    assert len(out) == 2

    out = franka_J_ik_swivel(
        desired[:3, -1], np.ravel(desired[:3, :3]), theta, joint_angles=False
    )
    assert len(out) == 2


def test_j_to_q(get_q) -> None:
    """Test Jacobian to joint angles"""
    q = get_q
    desired = franka_fk(q)
    jac, qsol, _ = franka_J_ik_q6(
        desired[:3, -1], np.ravel(desired[:3, :3]), np.pi / 2.0, joint_angles=True
    )

    ok_idx = np.array([i for i in range(len(qsol)) if not np.any(np.isnan(qsol[i]))])
    qsols = np.array(qsol)[ok_idx]
    jacs = np.array(jac)[ok_idx]

    new_jacs = np.array(J_from_q(qsols[0]))
    np.testing.assert_allclose(new_jacs, jacs[0], atol=1e-6)
