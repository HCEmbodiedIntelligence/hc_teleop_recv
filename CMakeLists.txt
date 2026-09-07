cmake_minimum_required(VERSION 3.8)
project(hc_teleop_recv)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/hc_teleop_recv_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_frontend test/test_frontend.py)
  ament_add_pytest_test(test_peripherals test/test_peripherals.py)
  ament_add_pytest_test(test_ros_frontend test/test_ros_frontend.py TIMEOUT 60)
endif()
ament_package()
