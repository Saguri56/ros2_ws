cmake_minimum_required(VERSION 3.8)
project(husky_ur5)

# Activar opciones de compilación
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Incluir dependencias de ROS 2 necesarias
find_package(husky_description REQUIRED)
find_package(ur_description REQUIRED)

# Configuración para las pruebas
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Instalar directorios de lanzamiento y URDF/XACRO
install(DIRECTORY 
  launch
  urdf
  config
  worlds
  maps
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
