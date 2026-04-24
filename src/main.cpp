#include <pybind11/pybind11.h>
#include "geofik.h"
#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <array>

using Eigen::Matrix;
using Eigen::Matrix4d;

using namespace std;


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
    )delim");

}
