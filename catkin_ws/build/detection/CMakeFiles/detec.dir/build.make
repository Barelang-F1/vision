# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


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
CMAKE_SOURCE_DIR = /home/bf1/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bf1/catkin_ws/build

# Include any dependencies generated for this target.
include detection/CMakeFiles/detec.dir/depend.make

# Include the progress variables for this target.
include detection/CMakeFiles/detec.dir/progress.make

# Include the compile flags for this target's objects.
include detection/CMakeFiles/detec.dir/flags.make

detection/CMakeFiles/detec.dir/src/korban.cpp.o: detection/CMakeFiles/detec.dir/flags.make
detection/CMakeFiles/detec.dir/src/korban.cpp.o: /home/bf1/catkin_ws/src/detection/src/korban.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bf1/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object detection/CMakeFiles/detec.dir/src/korban.cpp.o"
	cd /home/bf1/catkin_ws/build/detection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detec.dir/src/korban.cpp.o -c /home/bf1/catkin_ws/src/detection/src/korban.cpp

detection/CMakeFiles/detec.dir/src/korban.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detec.dir/src/korban.cpp.i"
	cd /home/bf1/catkin_ws/build/detection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bf1/catkin_ws/src/detection/src/korban.cpp > CMakeFiles/detec.dir/src/korban.cpp.i

detection/CMakeFiles/detec.dir/src/korban.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detec.dir/src/korban.cpp.s"
	cd /home/bf1/catkin_ws/build/detection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bf1/catkin_ws/src/detection/src/korban.cpp -o CMakeFiles/detec.dir/src/korban.cpp.s

detection/CMakeFiles/detec.dir/src/korban.cpp.o.requires:

.PHONY : detection/CMakeFiles/detec.dir/src/korban.cpp.o.requires

detection/CMakeFiles/detec.dir/src/korban.cpp.o.provides: detection/CMakeFiles/detec.dir/src/korban.cpp.o.requires
	$(MAKE) -f detection/CMakeFiles/detec.dir/build.make detection/CMakeFiles/detec.dir/src/korban.cpp.o.provides.build
.PHONY : detection/CMakeFiles/detec.dir/src/korban.cpp.o.provides

detection/CMakeFiles/detec.dir/src/korban.cpp.o.provides.build: detection/CMakeFiles/detec.dir/src/korban.cpp.o


# Object files for target detec
detec_OBJECTS = \
"CMakeFiles/detec.dir/src/korban.cpp.o"

# External object files for target detec
detec_EXTERNAL_OBJECTS =

detection/detec: detection/CMakeFiles/detec.dir/src/korban.cpp.o
detection/detec: detection/CMakeFiles/detec.dir/build.make
detection/detec: /opt/ros/melodic/lib/libroscpp.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
detection/detec: /opt/ros/melodic/lib/librosconsole.so
detection/detec: /opt/ros/melodic/lib/librosconsole_log4cxx.so
detection/detec: /opt/ros/melodic/lib/librosconsole_backend_interface.so
detection/detec: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_regex.so
detection/detec: /opt/ros/melodic/lib/libxmlrpcpp.so
detection/detec: /opt/ros/melodic/lib/libroscpp_serialization.so
detection/detec: /opt/ros/melodic/lib/librostime.so
detection/detec: /opt/ros/melodic/lib/libcpp_common.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_system.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_thread.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
detection/detec: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
detection/detec: /usr/lib/aarch64-linux-gnu/libpthread.so
detection/detec: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
detection/detec: /usr/local/cuda/lib64/libcudart_static.a
detection/detec: /usr/lib/aarch64-linux-gnu/librt.so
detection/detec: detection/CMakeFiles/detec.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bf1/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable detec"
	cd /home/bf1/catkin_ws/build/detection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detec.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
detection/CMakeFiles/detec.dir/build: detection/detec

.PHONY : detection/CMakeFiles/detec.dir/build

detection/CMakeFiles/detec.dir/requires: detection/CMakeFiles/detec.dir/src/korban.cpp.o.requires

.PHONY : detection/CMakeFiles/detec.dir/requires

detection/CMakeFiles/detec.dir/clean:
	cd /home/bf1/catkin_ws/build/detection && $(CMAKE_COMMAND) -P CMakeFiles/detec.dir/cmake_clean.cmake
.PHONY : detection/CMakeFiles/detec.dir/clean

detection/CMakeFiles/detec.dir/depend:
	cd /home/bf1/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bf1/catkin_ws/src /home/bf1/catkin_ws/src/detection /home/bf1/catkin_ws/build /home/bf1/catkin_ws/build/detection /home/bf1/catkin_ws/build/detection/CMakeFiles/detec.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : detection/CMakeFiles/detec.dir/depend

