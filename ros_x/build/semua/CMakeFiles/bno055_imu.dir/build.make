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
CMAKE_SOURCE_DIR = /home/bf1/ros_x/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bf1/ros_x/build

# Include any dependencies generated for this target.
include semua/CMakeFiles/bno055_imu.dir/depend.make

# Include the progress variables for this target.
include semua/CMakeFiles/bno055_imu.dir/progress.make

# Include the compile flags for this target's objects.
include semua/CMakeFiles/bno055_imu.dir/flags.make

semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o: semua/CMakeFiles/bno055_imu.dir/flags.make
semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o: /home/bf1/ros_x/src/semua/src/bno055_imu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bf1/ros_x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o"
	cd /home/bf1/ros_x/build/semua && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o -c /home/bf1/ros_x/src/semua/src/bno055_imu.cpp

semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.i"
	cd /home/bf1/ros_x/build/semua && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bf1/ros_x/src/semua/src/bno055_imu.cpp > CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.i

semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.s"
	cd /home/bf1/ros_x/build/semua && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bf1/ros_x/src/semua/src/bno055_imu.cpp -o CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.s

semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.requires:

.PHONY : semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.requires

semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.provides: semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.requires
	$(MAKE) -f semua/CMakeFiles/bno055_imu.dir/build.make semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.provides.build
.PHONY : semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.provides

semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.provides.build: semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o


# Object files for target bno055_imu
bno055_imu_OBJECTS = \
"CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o"

# External object files for target bno055_imu
bno055_imu_EXTERNAL_OBJECTS =

/home/bf1/ros_x/devel/lib/semua/bno055_imu: semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o
/home/bf1/ros_x/devel/lib/semua/bno055_imu: semua/CMakeFiles/bno055_imu.dir/build.make
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libdynamixel_sdk.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libroscpp.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librosconsole.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librostime.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libcpp_common.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libdynamixel_sdk.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libroscpp.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librosconsole.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/librostime.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /opt/ros/melodic/lib/libcpp_common.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/bf1/ros_x/devel/lib/semua/bno055_imu: /usr/local/lib/libJetsonGPIO.so
/home/bf1/ros_x/devel/lib/semua/bno055_imu: semua/CMakeFiles/bno055_imu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bf1/ros_x/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bf1/ros_x/devel/lib/semua/bno055_imu"
	cd /home/bf1/ros_x/build/semua && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bno055_imu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
semua/CMakeFiles/bno055_imu.dir/build: /home/bf1/ros_x/devel/lib/semua/bno055_imu

.PHONY : semua/CMakeFiles/bno055_imu.dir/build

semua/CMakeFiles/bno055_imu.dir/requires: semua/CMakeFiles/bno055_imu.dir/src/bno055_imu.cpp.o.requires

.PHONY : semua/CMakeFiles/bno055_imu.dir/requires

semua/CMakeFiles/bno055_imu.dir/clean:
	cd /home/bf1/ros_x/build/semua && $(CMAKE_COMMAND) -P CMakeFiles/bno055_imu.dir/cmake_clean.cmake
.PHONY : semua/CMakeFiles/bno055_imu.dir/clean

semua/CMakeFiles/bno055_imu.dir/depend:
	cd /home/bf1/ros_x/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bf1/ros_x/src /home/bf1/ros_x/src/semua /home/bf1/ros_x/build /home/bf1/ros_x/build/semua /home/bf1/ros_x/build/semua/CMakeFiles/bno055_imu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : semua/CMakeFiles/bno055_imu.dir/depend
