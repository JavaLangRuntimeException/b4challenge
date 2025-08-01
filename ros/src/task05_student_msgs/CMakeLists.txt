cmake_minimum_required(VERSION 3.8)
project(task05_student_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies - 参考資料の順序に従って整理
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces - 参考資料の形式に準拠
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StudentInfo.msg"
  "msg/StudentArray.msg"
  DEPENDENCIES std_msgs sensor_msgs # Add packages that above messages depend on
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
