cmake_minimum_required(VERSION 3.16)
project(wbdopt_ik_lib VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED CONFIG)
find_package(pinocchio REQUIRED HINTS /opt/openrobots/lib/cmake/pinocchio)
find_package(qpOASES REQUIRED HINTS /opt/openrobots/lib/cmake/qpOASES)

if(TARGET NLopt::nlopt_cxx)
  set(WBDOPT_IK_NLOPT_TARGET NLopt::nlopt_cxx)
elseif(TARGET NLopt::nlopt)
  set(WBDOPT_IK_NLOPT_TARGET NLopt::nlopt)
else()
  message(FATAL_ERROR "NLopt was found but did not export a supported CMake target")
endif()

# Declare the custom option so CMake recognizes it
option(WBDOPT_IK_DEBUG "Enable verbose debug logs for optimal IK solver" OFF)

add_library(wbdopt_ik SHARED
  src/nlopt_solver.cpp
  src/qp_solver.cpp
  src/solver_base.cpp
  src/solver_factory.cpp
  src/wbdopt_ik.cpp)
add_library(${PROJECT_NAME}::wbdopt_ik ALIAS wbdopt_ik)
target_compile_features(wbdopt_ik PUBLIC cxx_std_17)
target_compile_definitions(wbdopt_ik PRIVATE COAL_DISABLE_HPP_FCL_WARNINGS)
if(WBDOPT_IK_DEBUG)
  target_compile_definitions(wbdopt_ik PRIVATE WBDOPT_IK_DEBUG)
endif()
target_include_directories(wbdopt_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# qpOASES is installed into /opt/openrobots on robotpkg systems. Use
# -idirafter so that a second Pinocchio installation in that same prefix cannot
# shadow the Pinocchio headers selected by find_package above.
target_compile_options(wbdopt_ik PRIVATE "SHELL:-idirafter ${qpOASES_INCLUDE_DIR}")
target_link_libraries(wbdopt_ik
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    ${WBDOPT_IK_NLOPT_TARGET}
    pinocchio::pinocchio
    ${qpOASES_LIBRARY})
set_target_properties(wbdopt_ik PROPERTIES
  BUILD_RPATH "${qpOASES_LIBRARY_DIR}"
  INSTALL_RPATH "${qpOASES_LIBRARY_DIR}")

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS wbdopt_ik
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 NLopt)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wbdopt_ik test/test_wbdopt_ik.cpp)
  target_link_libraries(test_wbdopt_ik wbdopt_ik)
endif()

ament_package()
