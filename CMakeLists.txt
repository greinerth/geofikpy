cmake_minimum_required(VERSION 3.15...3.26)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(_core MODULE src/main.cpp src/geofik.cpp)
target_include_directories(_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(_core PRIVATE Eigen3::Eigen)
install(TARGETS _core DESTINATION ${SKBUILD_PROJECT_NAME})
