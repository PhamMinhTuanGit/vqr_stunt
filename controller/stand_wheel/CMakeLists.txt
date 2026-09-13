cmake_minimum_required(VERSION 3.14)
project(stand_wheel_model_audit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)

set(STAND_WHEEL_CASADI_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/casadi/install"
    CACHE PATH "CasADi installation used by stand_wheel symbolic model")
find_package(casadi 3.7 REQUIRED CONFIG
  HINTS "${STAND_WHEEL_CASADI_ROOT}/lib/cmake/casadi")

set(STAND_WHEEL_IPOPT_PLUGIN_DIR "" CACHE PATH
    "Directory containing CasADi's libcasadi_nlpsol_ipopt plugin")
if(NOT STAND_WHEEL_IPOPT_PLUGIN_DIR)
  if(EXISTS "${STAND_WHEEL_CASADI_ROOT}/lib/libcasadi_nlpsol_ipopt.so")
    set(STAND_WHEEL_IPOPT_PLUGIN_DIR "${STAND_WHEEL_CASADI_ROOT}/lib")
  else()
    file(GLOB STAND_WHEEL_IPOPT_PLUGIN_CANDIDATES
      "${CMAKE_CURRENT_SOURCE_DIR}/../../.venv/lib/python*/site-packages/casadi/libcasadi_nolpsol_ipopt.so"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../.venv/lib/python*/site-packages/casadi/libcasadi_nlpsol_ipopt.so"
    )
    if(STAND_WHEEL_IPOPT_PLUGIN_CANDIDATES)
      list(GET STAND_WHEEL_IPOPT_PLUGIN_CANDIDATES 0
        STAND_WHEEL_IPOPT_PLUGIN_LIBRARY)
      get_filename_component(STAND_WHEEL_IPOPT_PLUGIN_DIR
        "${STAND_WHEEL_IPOPT_PLUGIN_LIBRARY}" DIRECTORY)
    endif()
  endif()
endif()
if(NOT EXISTS "${STAND_WHEEL_IPOPT_PLUGIN_DIR}/libcasadi_nlpsol_ipopt.so")
  message(FATAL_ERROR
    "CasADi IPOPT plugin not found; set STAND_WHEEL_IPOPT_PLUGIN_DIR")
endif()
message(STATUS "CasADi IPOPT plugin: ${STAND_WHEEL_IPOPT_PLUGIN_DIR}")

add_library(stand_wheel_pinocchio_model STATIC
  src/pinocchio_model.cpp
  src/full_dynamics.cpp
  src/wheel_contact.cpp
)
target_include_directories(stand_wheel_pinocchio_model PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(stand_wheel_pinocchio_model PUBLIC
  pinocchio::pinocchio
  Eigen3::Eigen
)

add_library(stand_wheel_casadi_model STATIC
  src/casadi_model.cpp
)
target_include_directories(stand_wheel_casadi_model PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(stand_wheel_casadi_model PUBLIC
  stand_wheel_pinocchio_model
  casadi::casadi
)
# Template-instantiating the complete Pinocchio model with SX is expensive;
# -O1 keeps release-build memory/time bounded without changing expressions.
target_compile_options(stand_wheel_casadi_model PRIVATE
  $<$<CONFIG:Release>:-O1>
)

add_library(stand_wheel_trajectory_optimizer STATIC
  src/trajectory_optimizer.cpp
)
target_include_directories(stand_wheel_trajectory_optimizer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(stand_wheel_trajectory_optimizer PUBLIC
  stand_wheel_casadi_model
)
target_compile_definitions(stand_wheel_trajectory_optimizer PUBLIC
  STAND_WHEEL_IPOPT_PLUGIN_DIR="${STAND_WHEEL_IPOPT_PLUGIN_DIR}"
)

enable_testing()

set(STAND_WHEEL_VQR_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../../vqr_description/vqr_urdf/urdf/VQRWheel.urdf")

add_executable(pinocchio_model_smoke_test
  test/pinocchio_model_smoke_test.cpp
)
target_link_libraries(pinocchio_model_smoke_test PRIVATE
  stand_wheel_pinocchio_model
)
target_compile_definitions(pinocchio_model_smoke_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
)

add_test(
  NAME pinocchio_model_smoke
  COMMAND pinocchio_model_smoke_test ${STAND_WHEEL_VQR_URDF}
)

add_executable(full_dynamics_test
  test/full_dynamics_test.cpp
)
target_link_libraries(full_dynamics_test PRIVATE
  stand_wheel_pinocchio_model
)
target_compile_definitions(full_dynamics_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
)

add_test(
  NAME full_dynamics
  COMMAND full_dynamics_test ${STAND_WHEEL_VQR_URDF}
)

add_executable(wheel_contact_test
  test/wheel_contact_test.cpp
)
target_link_libraries(wheel_contact_test PRIVATE
  stand_wheel_pinocchio_model
)
target_compile_definitions(wheel_contact_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
)

add_test(
  NAME wheel_contact
  COMMAND wheel_contact_test ${STAND_WHEEL_VQR_URDF}
)

add_executable(casadi_model_test
  test/casadi_model_test.cpp
)
target_link_libraries(casadi_model_test PRIVATE
  stand_wheel_casadi_model
)
target_compile_definitions(casadi_model_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
)

add_test(
  NAME casadi_model
  COMMAND casadi_model_test ${STAND_WHEEL_VQR_URDF}
)

add_executable(yaw_feasibility_test
  test/yaw_feasibility_test.cpp
)
target_link_libraries(yaw_feasibility_test PRIVATE
  stand_wheel_pinocchio_model
)
target_compile_definitions(yaw_feasibility_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
)

add_test(
  NAME yaw_feasibility
  COMMAND yaw_feasibility_test ${STAND_WHEEL_VQR_URDF}
)

add_executable(skid_steer_contact_test
  test/skid_steer_contact_test.cpp
)
target_link_libraries(skid_steer_contact_test PRIVATE
  stand_wheel_casadi_model
)
target_compile_definitions(skid_steer_contact_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
)

add_test(
  NAME skid_steer_contact
  COMMAND skid_steer_contact_test ${STAND_WHEEL_VQR_URDF}
)

add_executable(stand_wheel_static_opt
  src/stand_wheel_static_opt.cpp
)
target_link_libraries(stand_wheel_static_opt PRIVATE
  stand_wheel_trajectory_optimizer
)

add_executable(trajectory_optimizer_test
  test/trajectory_optimizer_test.cpp
)
target_link_libraries(trajectory_optimizer_test PRIVATE
  stand_wheel_trajectory_optimizer
)
target_compile_definitions(trajectory_optimizer_test PRIVATE
  STAND_WHEEL_DEFAULT_URDF_PATH="${STAND_WHEEL_VQR_URDF}"
  STAND_WHEEL_TEST_CSV_PATH="/tmp/stand_wheel_static_nlp_test.csv"
)

add_test(
  NAME trajectory_optimizer
  COMMAND trajectory_optimizer_test ${STAND_WHEEL_VQR_URDF}
)
