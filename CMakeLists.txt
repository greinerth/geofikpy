cmake_minimum_required(VERSION 3.15...3.26)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(geofik MODULE src/main.cpp src/geofik.cpp)
target_include_directories(geofik PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(geofik PRIVATE Eigen3::Eigen)
install(TARGETS geofik DESTINATION ${SKBUILD_PROJECT_NAME})
