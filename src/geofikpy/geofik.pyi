from typing import Literal

from jaxtyping import Array, Float

def J_to_q(
    J: Float[Array, "7 6"],  # noqa: F722
    R: Float[Array, "3 3"],  # noqa: F722
    ee: Literal["E", "F", "8"],
) -> list[float]: ...
def J_from_q(
    q: Float[Array, 7], ee: Literal["E", "F", "8"] = "E"
) -> Float[Array, "7 6"]: ...  # noqa: F722
def franka_fk(
    q: Float[Array, 7], ee: Literal["E", "F", "8"] = "E"
) -> Float[Array, "4 4"]: ...  # noqa: F722
def franka_ik_q4(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    q4: float,
    q1_sing: float = ...,
    q7_sing: float = 0.0,
) -> tuple[list[list[float]], int]: ...
def franka_ik_q6(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    q6: float,
    q1_sing: float = ...,
    q7_sing: float = 0.0,
) -> tuple[list[list[float]], int]: ...
def franka_ik_q7(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    q7: float,
    q1_sing: float = ...,
) -> tuple[list[list[float]], int]: ...
def franka_ik_swivel(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    theta: float,
    q1_sing: float = ...,
    n_points: int = 500,
    n_fine_search: int = 3,
) -> tuple[list[list[float]], int]: ...
def franka_swivel(q: Float[Array, 7]) -> float: ...
def franka_J_ik_q4(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    q4: float,
    joint_angles: bool = False,
    Jacobian_ee: Literal["E", "F", "8"] = "E",
    q1_sing: float = ...,
) -> (
    tuple[list[list[list[float]]], list[list[float]], int]
    | tuple[list[list[list[float]]], int]
): ...
def franka_J_ik_q6(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    q6: float,
    joint_angles: bool = False,
    Jacobian_ee: Literal["E", "F", "8"] = "E",
    q1_sing: float = ...,
) -> (
    tuple[list[list[list[float]]], list[list[float]], int]
    | tuple[list[list[list[float]]], int]
): ...
def franka_J_ik_q7(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    q7: float,
    joint_angles: bool = False,
    Jacobian_ee: Literal["E", "F", "8"] = "E",
    q1_sing: float = ...,
) -> (
    tuple[list[list[list[float]]], list[list[float]], int]
    | tuple[list[list[list[float]]], int]
): ...
def franka_J_ik_swivel(
    r: Float[Array, 3],
    ROE: Float[Array, "3 3"],  # noqa: F722
    theta: float,
    joint_angles: bool = False,
    Jacobian_ee: Literal["E", "F", "8"] = "E",
    q1_sing: float = ...,
    n_points: int = 600,
    n_fine_search: int = 3,
) -> (
    tuple[list[list[list[float]]], list[list[float]], int]
    | tuple[list[list[list[float]]], int]
): ...
