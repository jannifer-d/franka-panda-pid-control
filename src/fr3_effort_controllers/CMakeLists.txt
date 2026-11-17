cmake_minimum_required(VERSION 3.20)
project(fr3_effort_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# FR3 robot model library
add_library(fr3_model SHARED
  src/fr3_model.cpp
)

target_compile_features(fr3_model PUBLIC cxx_std_17)

target_include_directories(fr3_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fr3_model PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
)

# Robot model smoke test executable
add_executable(model_smoke_test
  src/model_smoke_test.cpp
)

target_compile_features(model_smoke_test PUBLIC cxx_std_17)

target_link_libraries(model_smoke_test
  fr3_model
)

# Quintic trajectory test executable
add_executable(trajectory_test
  src/trajectory_test.cpp
)

target_compile_features(trajectory_test PUBLIC cxx_std_17)

target_include_directories(trajectory_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(trajectory_test
  Eigen3::Eigen
)

# Install the model library
install(
  TARGETS fr3_model
  EXPORT export_fr3_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the smoke test executable
install(
  TARGETS model_smoke_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the trajectory test executable
install(
  TARGETS trajectory_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install public headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the generated FR3 URDF
install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/fr3.urdf"
  DESTINATION
    share/${PROJECT_NAME}/urdf
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(fr3_model)
ament_export_dependencies(Eigen3 pinocchio)

ament_package()
