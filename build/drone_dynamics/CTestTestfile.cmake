# CMake generated Testfile for 
# Source directory: /home/ake/sim_drone/ake_drone_sim/src/drone_dynamics
# Build directory: /home/ake/sim_drone/ake_drone_sim/build/drone_dynamics
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_dynamics "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ake/sim_drone/ake_drone_sim/build/drone_dynamics/test_results/drone_dynamics/test_dynamics.gtest.xml" "--package-name" "drone_dynamics" "--output-file" "/home/ake/sim_drone/ake_drone_sim/build/drone_dynamics/ament_cmake_gtest/test_dynamics.txt" "--command" "/home/ake/sim_drone/ake_drone_sim/build/drone_dynamics/test_dynamics" "--gtest_output=xml:/home/ake/sim_drone/ake_drone_sim/build/drone_dynamics/test_results/drone_dynamics/test_dynamics.gtest.xml")
set_tests_properties(test_dynamics PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ake/sim_drone/ake_drone_sim/build/drone_dynamics/test_dynamics" TIMEOUT "60" WORKING_DIRECTORY "/home/ake/sim_drone/ake_drone_sim/build/drone_dynamics" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ake/sim_drone/ake_drone_sim/src/drone_dynamics/CMakeLists.txt;32;ament_add_gtest;/home/ake/sim_drone/ake_drone_sim/src/drone_dynamics/CMakeLists.txt;0;")
subdirs("gtest")
