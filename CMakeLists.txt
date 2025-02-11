cmake_minimum_required(VERSION 3.17)


project(CalibrationSolver)

set(CMAKE_CXX_STANDARD 17)

# Get the directory of the CMakeLists.txt file
get_filename_component(PROJECT_DIR ${CMAKE_SOURCE_DIR} ABSOLUTE)

# Pass the project directory path as a preprocessor macro to the C++ code
add_definitions(-DPROJECT_DIR="${PROJECT_DIR}")


find_package(Ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)


add_executable(CalibrationSolver
  src/cpp_calibration_solver.cpp
)

target_link_libraries(CalibrationSolver PUBLIC
    ${CERES_LIBRARIES}
    Eigen3::Eigen
)

