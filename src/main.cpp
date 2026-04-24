#include <pybind11/pybind11.h>
#include "geofik.h"
#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <array>

using namespace std;

std::tuple<std::array<std::array<double, 7>, 8>, unsigned int> _franka_ik_q7(const std::array<double, 3>& r,
                          const std::array<double, 9>& ROE,
                          const double q7,
                          const double q1_sing = PI / 2){

  std::array<std::array<double, 7>, 8> qsols{};
  unsigned int sol =  franka_ik_q7(r, ROE, q7, qsols, q1_sing);
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
  m.def("franka_ik_q7", &_franka_ik_q7, py::arg("r"), py::arg("ROE"), py::arg("q7"), py::arg("q1_sing"),
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
}
