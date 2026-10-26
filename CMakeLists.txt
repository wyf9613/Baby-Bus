cmake_minimum_required(VERSION 3.8)
project(ai4r_policy)
find_package(ament_cmake REQUIRED)

# Both executable names use the same modular MVP2.1 implementation.
install(PROGRAMS scripts/policy_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/mvp21_policy_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY mvp21_policy DESTINATION lib/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.py" PATTERN "*.json")
# Shared inference math only; no optimizer, data collector or simulator.
install(FILES offline/lattice_training/__init__.py offline/lattice_training/features.py
        offline/lattice_training/spatial.py offline/lattice_training/source.py
        DESTINATION lib/${PROJECT_NAME}/offline/lattice_training)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(policy_node tests/test_policy_node.py TIMEOUT 60)
  ament_add_pytest_test(estimation tests/test_estimation.py TIMEOUT 30)
  ament_add_pytest_test(colour_assignment tests/test_colour_assignment.py TIMEOUT 30)
  ament_add_pytest_test(ordered_pairing tests/test_ordered_pairing.py TIMEOUT 30)
  ament_add_pytest_test(planning tests/test_planning.py TIMEOUT 30)
  ament_add_pytest_test(centerline_follow tests/test_centerline_follow.py TIMEOUT 30)
  ament_add_pytest_test(lattice tests/test_lattice.py TIMEOUT 60)
  ament_add_pytest_test(planner_execution tests/test_planner_execution.py TIMEOUT 60)
  ament_add_pytest_test(mvp21_compatibility tests/test_mvp21_compatibility.py TIMEOUT 60)
  ament_add_pytest_test(control tests/test_control.py TIMEOUT 30)
  ament_add_pytest_test(learned_policy tests/test_learned_policy.py TIMEOUT 60)
  ament_add_pytest_test(lidar_obstacles tests/test_lidar_obstacles.py TIMEOUT 60)
endif()
ament_package()
