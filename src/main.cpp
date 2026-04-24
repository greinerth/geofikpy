#include <pybind11/pybind11.h>
#include <geofik.h>
#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <array>

using Eigen::Matrix;
using Eigen::VectorXcd;
using Eigen::MatrixXcd;
using namespace std;



int add(int i, int j) { return i + j; }

namespace py = pybind11;

std::vector<double> _J_to_q(const Eigen::Ref<const Eigen::Matrix<double, 6, 7>>& J, const Eigen::Ref<const Eigen::Matrix<double, 3, 3>>& R, const char ee) {

    std::array<std::array<double, 6>, 7> J_arr{};
    for (int i = 0; i < 7; i++){
        for (int j = 0; j < 6; j++){
            J_arr[i][j] = J(j, i);
        }
    }
    std::array<std::array<double, 3>, 3> R_arr{};
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            R_arr[i][j] = R(i, j);
        }
    }

    return J_to_q(J_arr, R_arr, ee);
}

PYBIND11_MODULE(_core, m) {
  m.doc() = R"pbdoc(
      Pybind11 example plugin
      -----------------------
      .. currentmodule:: python_example
      .. autosummary::
         :toctree: _generate
         add
         subtract
  )pbdoc";
  m.def("J_to_q", &_J_to_q, py::arg("J"), py::arg("R"), py::arg("ee"),
    R"delim(
      Compute joint configurations from Jacobian
      
      Args:
        J: Jacobian of shape [6, 7]
        R: Rotation matrix of shape [3, 3]
        ee: End-effector frame ["E", "F", "8"]
    )delim");
}
