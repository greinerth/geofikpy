#include "geofik.h"
#include <Eigen/Dense>
#include <array>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace std;
namespace py = pybind11;

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int>
_franka_ik_q4(const std::array<double, 3> &r, const std::array<double, 9> &ROE,
              const double q4, const double q1_sing = PI / 2,
              const double q7_sing = 0) {

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol = franka_ik_q4(r, ROE, q4, qsols, q1_sing, q7_sing);
  return std::make_tuple(qsols, sol);
}

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int>
_franka_ik_q6(const std::array<double, 3> &r, const std::array<double, 9> &ROE,
              const double q6, const double q1_sing = PI / 2,
              const double q7_sing = 0) {

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol = franka_ik_q6(r, ROE, q6, qsols, q1_sing, q7_sing);
  return std::make_tuple(qsols, sol);
}

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int>
_franka_ik_q7(const std::array<double, 3> &r, const std::array<double, 9> &ROE,
              const double q7, const double q1_sing = PI / 2) {

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol = franka_ik_q7(r, ROE, q7, qsols, q1_sing);
  return std::make_tuple(qsols, sol);
}

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int>
_franka_ik_swivel(const std::array<double, 3> &r,
                  const std::array<double, 9> &ROE, const double theta,
                  const double q1_sing = PI / 2,
                  const unsigned int n_points = 500,
                  const unsigned int n_fine_search = 3) {

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =
      franka_ik_swivel(r, ROE, theta, qsols, q1_sing, n_points, n_fine_search);
  return std::make_tuple(qsols, sol);
}
pybind11::object _franka_J_ik_q4(const std::array<double, 3> &r,
                                 const std::array<double, 9> &ROE,
                                 const double q4,
                                 const bool joint_angles = false,
                                 const char Jacobian_ee = 'E',
                                 const double q1_sing = PI / 2) {

  std::array<std::array<std::array<double, 6>, 7>, 8> Jsols{};
  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol = franka_J_ik_q4(r, ROE, q4, Jsols, qsols, joint_angles,
                                    Jacobian_ee, q1_sing);

  if (joint_angles) {
    return py::cast(std::make_tuple(Jsols, qsols, sol));
  }
  return py::cast(std::make_tuple(Jsols, sol));
}

pybind11::object _franka_J_ik_q6(const std::array<double, 3> &r,
                                 const std::array<double, 9> &ROE,
                                 const double q6,
                                 const bool joint_angles = false,
                                 const char Jacobian_ee = 'E',
                                 const double q1_sing = PI / 2) {

  std::array<std::array<std::array<double, 6>, 7>, 8> Jsols{};
  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol = franka_J_ik_q6(r, ROE, q6, Jsols, qsols, joint_angles,
                                    Jacobian_ee, q1_sing);

  if (joint_angles) {
    return pybind11::cast(std::make_tuple(Jsols, qsols, sol));
  }
  return pybind11::cast(std::make_tuple(Jsols, sol));
}

py::object _franka_J_ik_q7(const std::array<double, 3> &r,
                           const std::array<double, 9> &ROE, const double q7,
                           const bool joint_angles = false,
                           const char Jacobian_ee = 'E',
                           const double q1_sing = PI / 2) {

  std::array<std::array<std::array<double, 6>, 7>, 8> Jsols{};
  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol = franka_J_ik_q7(r, ROE, q7, Jsols, qsols, joint_angles,
                                    Jacobian_ee, q1_sing);

  if (joint_angles) {
    return py::cast(std::make_tuple(Jsols, qsols, sol));
  }
  return py::cast(std::make_tuple(Jsols, sol));
}

py::object _franka_J_ik_swivel(
    const std::array<double, 3> &r, const std::array<double, 9> &ROE,
    const double theta, const bool joint_angles = false,
    const char Jacobian_ee = 'E', const double q1_sing = PI / 2,
    const unsigned int n_points = 600, const unsigned int n_fine_search = 3) {

  std::array<std::array<std::array<double, 6>, 7>, 8> Jsols{};
  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =
      franka_J_ik_swivel(r, ROE, theta, Jsols, qsols, joint_angles, Jacobian_ee,
                         q1_sing, n_points, n_fine_search);

  if (joint_angles) {
    return py::cast(std::make_tuple(Jsols, qsols, sol));
  }
  return py::cast(std::make_tuple(Jsols, sol));
}

PYBIND11_MODULE(_core, m) {
  m.doc() = "Python wrapper for GeoFIK C++ library";
  m.def("J_to_q", &J_to_q, py::arg("J"), py::arg("R"), py::arg("ee"),
        R"delim(
      Compute joint configurations from Jacobian

      Args:
        J (np.ndarray[float, float]): Jacobian of shape [6, 7]
        R (np.ndarray[float, float]): Rotation matrix of shape [3, 3]
        ee (char): End-effector frame ["E", "F", "8"]

      Returns:
        q (np.ndarray[float]): Joint configurations of shape [7]

    )delim");
  m.def("J_from_q", &J_from_q, py::arg("q"), py::arg("ee") = 'E',
        R"delim(
      Compute joint configurations from Jacobian

      Args:
        q (np.ndarray[float]): Joint configurations of shape [7]
        ee (char, optional): End-effector frame ["E", "F", "8"], defaults to 'E'

      Returns:
        J (np.ndarray[float, float]): Jacobian of shape [6, 7]
    )delim");
  m.def("franka_fk", &franka_fk, py::arg("q"), py::arg("ee") = 'E',
        R"delim(
      Perform forward kinematics

      Args:
        q (np.ndarray[float]): Joint configurations of shape [7]
        ee (char, optional)): End-effector frame ["E", "F", "8"]

      Returns:
        T (np.ndarray[float, float]): Transformation matrix of shape [4, 4]
    )delim");
  m.def("franka_ik_q4", &_franka_ik_q4, py::arg("r"), py::arg("ROE"),
        py::arg("q4"), py::arg("q1_sing") = PI / 2, py::arg("q7_sing") = 0,
        R"delim(
        Perform IK with q4 locked

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          q4 (np.ndarray[float]): joint angle of joint 4 (radians)
          qsols: array to store 8 solutions
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
          q7_sing (float, optional): emergency value of q7 in case of singularity of S7 intersecting S (type-2 singularity), defaults to 0

        Returns:
          result (tuple[list[list[float]], int): Solutions and number of solutions of IK
    )delim");
  m.def("franka_ik_q6", &_franka_ik_q6, py::arg("r"), py::arg("ROE"),
        py::arg("q6"), py::arg("q1_sing") = PI / 2, py::arg("q7_sing") = 0,
        R"delim(
        Perform IK with q6 locked

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          q6 (np.ndarray[float]): joint angle of joint 4 (radians)
          qsols: array to store 8 solutions
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
          q7_sing (float, optional): emergency value of q7 in case of singularity of S7 intersecting S (type-2 singularity), defaults to 0

        Returns:
          result (tuple[list[list[float]], int): Solutions and number of solutions of IK
    )delim");
  m.def("franka_ik_q7", &_franka_ik_q7, py::arg("r"), py::arg("ROE"),
        py::arg("q7"), py::arg("q1_sing") = PI / 2,
        R"delim(
        Perform IK with q7 locked

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          q7 (np.ndarray[float]): joint angle of joint 7 (radians)
          qsols: array to store 8 solutions
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2

        Returns:
          result (tuple[list[list[float]], int): Solutions and number of solutions of IK
    )delim");
  m.def("franka_ik_swivel", &_franka_ik_swivel, py::arg("r"), py::arg("ROE"),
        py::arg("theta"), py::arg("q1_sing") = PI / 2,
        py::arg("n_points") = 500, py::arg("n_fine_search") = 3,
        R"delim(
        Perform IK with predefined swivel angle

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          theta (np.ndarray[float]): swivel angle (radians)
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
          n_points (int, optional): number of points to discretise the range of q7, defaults to 500
          n_fine_search (int, optional): Number of fine search iterations, defaults to 3

        Returns:
          result (tuple[list[list[float]], int): Solutions and number of solutions of IK
    )delim");
  m.def("franka_swivel", &franka_swivel, py::arg("q"),
        R"delim(
        Compute swivel angle given joint angles

        Args:
          q (np.ndarray[float]): joint angles

        Returns:
          theta (float): swivel angle (radians)
        )delim");
  m.def("franka_J_ik_q7", &_franka_J_ik_q7, py::arg("r"), py::arg("ROE"),
        py::arg("q7"), py::arg("joint_angles") = false,
        py::arg("Jacobian_ee") = 'E', py::arg("q1_sing") = PI / 2,
        R"delim(
        Compute IK with q7 locked and compute Jacobian

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          q7 (float): joint angle of joint 7 (radians)
          joint_angles (bool, optional): if false only Jacobians are returned, else the joint angles are also returned, defaults to False
          Jacobian_ee (char, optional): ee frame of the Jacobian, not the IK ('E', 'F', '8' or '6'), defaults to 'E'
          q1_sing (float, optional):  emergency value of q1 in case of singularity at shoulder joints (type-1 singularity)

        Returns:
          result (tuple[list[list[list[float]]], list[list[float], int]: Returns Jacobians, joint angles and number of solutions if joint_angles is True
                else returns Jacobians and number of solutions if joint_angles is False
        )delim");
  m.def("franka_J_ik_q4", &_franka_J_ik_q4, py::arg("r"), py::arg("ROE"),
        py::arg("q4"), py::arg("joint_angles") = false,
        py::arg("Jacobian_ee") = 'E', py::arg("q1_sing") = PI / 2,
        R"delim(
        Compute IK with q4 locked and compute Jacobian

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          q4 (float): joint angle of joint 4 (radians)
          joint_angles (bool, optional): if false only Jacobians are returned, else the joint angles are also returned, defaults to False
          Jacobian_ee (char, optional): ee frame of the Jacobian, not the IK ('E', 'F', '8' or '6'), defaults to 'E'
          q1_sing (float, optional):  emergency value of q1 in case of singularity at shoulder joints (type-1 singularity)

        Returns:
          result (tuple[list[list[list[float]]], list[list[float], int]: Returns Jacobians, joint angles and number of solutions if joint_angles is True
                else returns Jacobians and number of solutions if joint_angles is False
        )delim");
  m.def("franka_J_ik_q6", &_franka_J_ik_q6, py::arg("r"), py::arg("ROE"),
        py::arg("q6"), py::arg("joint_angles") = false,
        py::arg("Jacobian_ee") = 'E', py::arg("q1_sing") = PI / 2,
        R"delim(
        Compute IK with q4 locked and compute Jacobian

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          q6 (float): joint angle of joint 6 (radians)
          joint_angles (bool, optional): if false only Jacobians are returned, else the joint angles are also returned, defaults to False
          Jacobian_ee (char, optional): ee frame of the Jacobian, not the IK ('E', 'F', '8' or '6'), defaults to 'E'
          q1_sing (float, optional):  emergency value of q1 in case of singularity at shoulder joints (type-1 singularity)

        Returns:
          result (tuple[list[list[list[float]]], list[list[float], int]: Returns Jacobians, joint angles and number of solutions if joint_angles is True
                else returns Jacobians and number of solutions if joint_angles is False
        )delim");
  m.def("franka_J_ik_swivel", &_franka_J_ik_swivel, py::arg("r"),
        py::arg("ROE"), py::arg("theta"), py::arg("joint_angles") = false,
        py::arg("Jacobian_ee") = 'E', py::arg("q1_sing") = PI / 2,
        py::arg("n_points") = 600, py::arg("n_fine_search") = 3,
        R"delim(
        Compute IK with q4 locked and compute Jacobian

        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float]): Flattened rotation matrix of frame E with respect to frame O (row-first format).
          theta (float): swivel angle (radians)
          joint_angles (bool, optional): if false only Jacobians are returned, else the joint angles are also returned, defaults to False
          Jacobian_ee (char, optional): ee frame of the Jacobian, not the IK ('E', 'F', '8' or '6'), defaults to 'E'
          q1_sing (float, optional):  emergency value of q1 in case of singularity at shoulder joints (type-1 singularity)

        Returns:
          result (tuple[np.ndarray[float, float, float], np.ndarray[float, float], int): Returns Jacobians, joint angles and number of solutions if joint_angles is True
                else returns Jacobians and number of solutions if joint_angles is False
        )delim");
}
