# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jane/lidardrone/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jane/lidardrone/build

# Utility rule file for clean_test_results_uav_utils.

# Include the progress variables for this target.
include planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/progress.make

planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils:
	cd /home/jane/lidardrone/build/planner/utils/uav_utils && /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/jane/lidardrone/build/test_results/uav_utils

clean_test_results_uav_utils: planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils
clean_test_results_uav_utils: planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/build.make

.PHONY : clean_test_results_uav_utils

# Rule to build all files generated by this target.
planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/build: clean_test_results_uav_utils

.PHONY : planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/build

planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/clean:
	cd /home/jane/lidardrone/build/planner/utils/uav_utils && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_uav_utils.dir/cmake_clean.cmake
.PHONY : planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/clean

planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/depend:
	cd /home/jane/lidardrone/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jane/lidardrone/src /home/jane/lidardrone/src/planner/utils/uav_utils /home/jane/lidardrone/build /home/jane/lidardrone/build/planner/utils/uav_utils /home/jane/lidardrone/build/planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : planner/utils/uav_utils/CMakeFiles/clean_test_results_uav_utils.dir/depend

