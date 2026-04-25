#include <pybind11/pybind11.h>
#include "geofik.h"
#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <array>

using namespace std;

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int> _franka_ik_q4(const std::array<double, 3>& r,
                          const std::array<double, 9>& ROE,
                          const double q4,
                          const double q1_sing = PI / 2,
                          const double q7_sing = 0
                        ){

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =  franka_ik_q4(r, ROE, q4, qsols, q1_sing, q7_sing);
  return std::make_tuple(qsols, sol);
}

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int> _franka_ik_q6(const std::array<double, 3>& r,
                          const std::array<double, 9>& ROE,
                          const double q6,
                          const double q1_sing = PI / 2,
                          const double q7_sing = 0
                        ){

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =  franka_ik_q6(r, ROE, q6, qsols, q1_sing, q7_sing);
  return std::make_tuple(qsols, sol);
}

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int> _franka_ik_q7(const std::array<double, 3>& r,
                          const std::array<double, 9>& ROE,
                          const double q7,
                          const double q1_sing = PI / 2){

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =  franka_ik_q7(r, ROE, q7, qsols, q1_sing);
  return std::make_tuple(qsols, sol);
}

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int> _franka_ik_swivel(const std::array<double, 3>& r,
                              const std::array<double, 9>& ROE,
                              const double theta,
                              const double q1_sing = PI / 2,
                              const unsigned int n_points = 500,
                              const unsigned int n_fine_search = 3){

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =  franka_ik_swivel(r, ROE, theta, qsols, q1_sing, n_points, n_fine_search);
  return std::make_tuple(qsols, sol);
}


namespace py = pybind11;

PYBIND11_MODULE(geofik, m) {
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
  m.def("franka_ik_q4", &_franka_ik_q4, py::arg("r"), py::arg("ROE"), py::arg("q4"), py::arg("q1_sing") = PI/2, py::arg("q7_sing") = 0,
        R"delim(
        Perform IK with q4 locked
        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float, float]): rotation matrix of frame E with respect to frame O (row-first format).
          q4 (np.ndarray[float]): joint angle of joint 4 (radians)
          qsols: array to store 8 solutions
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
          q7_sing (float, optional): emergency value of q7 in case of singularity of S7 intersecting S (type-2 singularity), defaults to 0
        Returns:
          result (tuple[np.ndarray[float, float], int)
    )delim");
  m.def("franka_ik_q6", &_franka_ik_q6, py::arg("r"), py::arg("ROE"), py::arg("q6"), py::arg("q1_sing") = PI/2, py::arg("q7_sing") = 0,
        R"delim(
        Perform IK with q6 locked
        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float, float]): rotation matrix of frame E with respect to frame O (row-first format).
          q6 (np.ndarray[float]): joint angle of joint 4 (radians)
          qsols: array to store 8 solutions
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
          q7_sing (float, optional): emergency value of q7 in case of singularity of S7 intersecting S (type-2 singularity), defaults to 0
        Returns:
          result (tuple[np.ndarray[float, float], int)
    )delim");
  m.def("franka_ik_q7", &_franka_ik_q7, py::arg("r"), py::arg("ROE"), py::arg("q7"), py::arg("q1_sing") = PI/2,
        R"delim(
        Perform IK with q7 locked
        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float, float]): rotation matrix of frame E with respect to frame O (row-first format).
          q7 (np.ndarray[float]): joint angle of joint 7 (radians)
          qsols: array to store 8 solutions
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
        Returns:
          result (tuple[np.ndarray[float, float], int)
    )delim");
  m.def("franka_ik_swivel", &_franka_ik_swivel, py::arg("r"), py::arg("ROE"), py::arg("theta"), py::arg("q1_sing") = PI/2, py::arg("n_points") = 500, py::arg("n_fine_search") = 3,
        R"delim(
        Perform IK with predefined swivel angle
        Args:
          r (np.ndarray[float]): position of frame E with respect to frame O.
          ROE (np.ndarray[float, float]): rotation matrix of frame E with respect to frame O (row-first format).
          theta (np.ndarray[float]): swivel angle (radians)
          q1_sing (float, optional): emergency value of q1 in case of singularity at shoulder joints (type-1 singularity), defaults to PI/2
          n_points (int, optional): number of points to discretise the range of q7, defaults to 500
          n_fine_search (int, optional): Number of fine search iterations, defaults to 3
        Returns:
          result (tuple[np.ndarray[float, float], int)
    )delim");
}
