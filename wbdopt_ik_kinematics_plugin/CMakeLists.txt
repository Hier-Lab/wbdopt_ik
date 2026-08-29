cmake_minimum_required(VERSION 3.16)
project(wbdopt_ik_kinematics_plugin VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(wbdopt_ik_lib REQUIRED)

generate_parameter_library(
  ${PROJECT_NAME}_parameters
  src/wbdopt_ik_kinematics_plugin_parameters.yaml)

add_library(${PROJECT_NAME} SHARED src/wbdopt_ik_kinematics_plugin.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_parameters
    ament_index_cpp::ament_index_cpp
    class_loader::class_loader
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    rclcpp::rclcpp
    srdfdom::srdfdom
    urdf::urdf
    wbdopt_ik_lib::wbdopt_ik)

pluginlib_export_plugin_description_file(
  moveit_core wbdopt_ik_kinematics_description.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES wbdopt_ik_kinematics_description.xml DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp class_loader generate_parameter_library moveit_core pluginlib rclcpp
  srdfdom urdf wbdopt_ik_lib)
ament_package()
