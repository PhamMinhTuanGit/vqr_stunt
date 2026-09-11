cmake_minimum_required(VERSION 3.14)
project(stand_4_wheel CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# -O3 BAT BUOC: Eigen o -O0 cham hon ~20x -> truot deadline 500 Hz
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
# Keep Eigen's alignment ABI compatible with prebuilt Pinocchio by default.
# Enable only when Pinocchio and this target are built with the same ISA flags.
option(WQ_NATIVE_ARCH "Bat -march=native (requires ABI-compatible dependencies)" OFF)
if(WQ_NATIVE_ARCH)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-march=native" HAS_NATIVE)
  if(HAS_NATIVE)
    string(APPEND CMAKE_CXX_FLAGS_RELEASE " -march=native")
  endif()
endif()

find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(wq_robot_model STATIC
  src/robot_model.cpp
)
target_include_directories(wq_robot_model PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/config
)
target_link_libraries(wq_robot_model PUBLIC
  pinocchio::pinocchio
  Eigen3::Eigen
)

enable_testing()
add_executable(test_jacobian_fd test/test_jacobian_fd.cpp)
target_link_libraries(test_jacobian_fd PRIVATE wq_robot_model)
set(WQ_VQR_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../../vqr_description/vqr_urdf/urdf/VQRWheel.urdf")
target_compile_definitions(test_jacobian_fd PRIVATE
    WQ_DEFAULT_URDF_PATH="${WQ_VQR_URDF}")
add_test(NAME jacobian_fd
         COMMAND test_jacobian_fd ${WQ_VQR_URDF})

# The model and its finite-difference test remain buildable without the optional
# QP backend. This is useful on deployment images that only carry Pinocchio.
find_package(eiquadprog QUIET)
if(eiquadprog_FOUND)
  add_library(wq_stand4w STATIC
    src/balance_qp.cpp
    src/fsm/state_qp_balance.cpp
  )
  target_link_libraries(wq_stand4w PUBLIC
    wq_robot_model
    eiquadprog::eiquadprog
  )

  add_executable(demo_balance src/main_demo.cpp)
  target_link_libraries(demo_balance PRIVATE wq_stand4w)
  target_compile_definitions(demo_balance PRIVATE
      WQ_DEFAULT_URDF_PATH="${WQ_VQR_URDF}")
else()
  message(WARNING
    "eiquadprog not found: building RobotModel and Jacobian test only; "
    "install eiquadprog to build BalanceQP and demo_balance")
endif()
