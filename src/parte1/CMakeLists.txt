cmake_minimum_required(VERSION 3.8)
project(parte1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- find dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)   # necessário para GetMap

# ---------------- executável ----------------
add_executable(astar_node src/astar.cpp)

# Inclui headers próprias do pacote (NavigationCore, RobotInterface, etc.)
target_include_directories(astar_node
  PRIVATE
    include
)

ament_target_dependencies(astar_node
  rclcpp
  cg_interfaces
)

install(TARGETS
  astar_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- testes ----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
