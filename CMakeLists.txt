cmake_minimum_required(VERSION 3.8)
project(ai4r_policy)
find_package(ament_cmake REQUIRED)

# The policy remains one directly readable Python script.
install(PROGRAMS scripts/policy_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(policy_node tests/test_policy_node.py TIMEOUT 60)
  ament_add_pytest_test(estimation tests/test_estimation.py TIMEOUT 30)
  ament_add_pytest_test(planning tests/test_planning.py TIMEOUT 30)
  ament_add_pytest_test(control tests/test_control.py TIMEOUT 30)
endif()
ament_package()
