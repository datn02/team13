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
CMAKE_SOURCE_DIR = /home/ducanh/team13/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ducanh/team13/build

# Include any dependencies generated for this target.
include ee4308_turtle/CMakeFiles/turtle_motion.dir/depend.make

# Include the progress variables for this target.
include ee4308_turtle/CMakeFiles/turtle_motion.dir/progress.make

# Include the compile flags for this target's objects.
include ee4308_turtle/CMakeFiles/turtle_motion.dir/flags.make

ee4308_turtle/CMakeFiles/turtle_motion.dir/src/common.cpp.o: ee4308_turtle/CMakeFiles/turtle_motion.dir/flags.make
ee4308_turtle/CMakeFiles/turtle_motion.dir/src/common.cpp.o: /home/ducanh/team13/src/ee4308_turtle/src/common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ducanh/team13/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ee4308_turtle/CMakeFiles/turtle_motion.dir/src/common.cpp.o"
	cd /home/ducanh/team13/build/ee4308_turtle && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/turtle_motion.dir/src/common.cpp.o -c /home/ducanh/team13/src/ee4308_turtle/src/common.cpp

ee4308_turtle/CMakeFiles/turtle_motion.dir/src/common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/turtle_motion.dir/src/common.cpp.i"
	cd /home/ducanh/team13/build/ee4308_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ducanh/team13/src/ee4308_turtle/src/common.cpp > CMakeFiles/turtle_motion.dir/src/common.cpp.i

ee4308_turtle/CMakeFiles/turtle_motion.dir/src/common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/turtle_motion.dir/src/common.cpp.s"
	cd /home/ducanh/team13/build/ee4308_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ducanh/team13/src/ee4308_turtle/src/common.cpp -o CMakeFiles/turtle_motion.dir/src/common.cpp.s

ee4308_turtle/CMakeFiles/turtle_motion.dir/src/motion.cpp.o: ee4308_turtle/CMakeFiles/turtle_motion.dir/flags.make
ee4308_turtle/CMakeFiles/turtle_motion.dir/src/motion.cpp.o: /home/ducanh/team13/src/ee4308_turtle/src/motion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ducanh/team13/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ee4308_turtle/CMakeFiles/turtle_motion.dir/src/motion.cpp.o"
	cd /home/ducanh/team13/build/ee4308_turtle && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/turtle_motion.dir/src/motion.cpp.o -c /home/ducanh/team13/src/ee4308_turtle/src/motion.cpp

ee4308_turtle/CMakeFiles/turtle_motion.dir/src/motion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/turtle_motion.dir/src/motion.cpp.i"
	cd /home/ducanh/team13/build/ee4308_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ducanh/team13/src/ee4308_turtle/src/motion.cpp > CMakeFiles/turtle_motion.dir/src/motion.cpp.i

ee4308_turtle/CMakeFiles/turtle_motion.dir/src/motion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/turtle_motion.dir/src/motion.cpp.s"
	cd /home/ducanh/team13/build/ee4308_turtle && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ducanh/team13/src/ee4308_turtle/src/motion.cpp -o CMakeFiles/turtle_motion.dir/src/motion.cpp.s

# Object files for target turtle_motion
turtle_motion_OBJECTS = \
"CMakeFiles/turtle_motion.dir/src/common.cpp.o" \
"CMakeFiles/turtle_motion.dir/src/motion.cpp.o"

# External object files for target turtle_motion
turtle_motion_EXTERNAL_OBJECTS =

/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: ee4308_turtle/CMakeFiles/turtle_motion.dir/src/common.cpp.o
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: ee4308_turtle/CMakeFiles/turtle_motion.dir/src/motion.cpp.o
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: ee4308_turtle/CMakeFiles/turtle_motion.dir/build.make
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/libroscpp.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/librosconsole.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/librostime.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /opt/ros/noetic/lib/libcpp_common.so
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion: ee4308_turtle/CMakeFiles/turtle_motion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ducanh/team13/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion"
	cd /home/ducanh/team13/build/ee4308_turtle && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/turtle_motion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ee4308_turtle/CMakeFiles/turtle_motion.dir/build: /home/ducanh/team13/devel/lib/ee4308_turtle/turtle_motion

.PHONY : ee4308_turtle/CMakeFiles/turtle_motion.dir/build

ee4308_turtle/CMakeFiles/turtle_motion.dir/clean:
	cd /home/ducanh/team13/build/ee4308_turtle && $(CMAKE_COMMAND) -P CMakeFiles/turtle_motion.dir/cmake_clean.cmake
.PHONY : ee4308_turtle/CMakeFiles/turtle_motion.dir/clean

ee4308_turtle/CMakeFiles/turtle_motion.dir/depend:
	cd /home/ducanh/team13/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ducanh/team13/src /home/ducanh/team13/src/ee4308_turtle /home/ducanh/team13/build /home/ducanh/team13/build/ee4308_turtle /home/ducanh/team13/build/ee4308_turtle/CMakeFiles/turtle_motion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ee4308_turtle/CMakeFiles/turtle_motion.dir/depend

