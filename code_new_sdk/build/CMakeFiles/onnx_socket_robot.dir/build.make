# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/ti5/cmake-3.28.5-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /home/ti5/cmake-3.28.5-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/onnx_socket_robot.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/onnx_socket_robot.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/onnx_socket_robot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/onnx_socket_robot.dir/flags.make

CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o: CMakeFiles/onnx_socket_robot.dir/flags.make
CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o: /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/onnx_socket_robot.cpp
CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o: CMakeFiles/onnx_socket_robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o -MF CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o.d -o CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o -c /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/onnx_socket_robot.cpp

CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/onnx_socket_robot.cpp > CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.i

CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/onnx_socket_robot.cpp -o CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.s

CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o: CMakeFiles/onnx_socket_robot.dir/flags.make
CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o: /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/socket_server.cpp
CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o: CMakeFiles/onnx_socket_robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o -MF CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o.d -o CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o -c /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/socket_server.cpp

CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/socket_server.cpp > CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.i

CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/src/socket_server.cpp -o CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.s

# Object files for target onnx_socket_robot
onnx_socket_robot_OBJECTS = \
"CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o" \
"CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o"

# External object files for target onnx_socket_robot
onnx_socket_robot_EXTERNAL_OBJECTS =

onnx_socket_robot: CMakeFiles/onnx_socket_robot.dir/src/onnx_socket_robot.cpp.o
onnx_socket_robot: CMakeFiles/onnx_socket_robot.dir/src/socket_server.cpp.o
onnx_socket_robot: CMakeFiles/onnx_socket_robot.dir/build.make
onnx_socket_robot: /usr/local/lib/libonnxruntime.so
onnx_socket_robot: CMakeFiles/onnx_socket_robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable onnx_socket_robot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/onnx_socket_robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/onnx_socket_robot.dir/build: onnx_socket_robot
.PHONY : CMakeFiles/onnx_socket_robot.dir/build

CMakeFiles/onnx_socket_robot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/onnx_socket_robot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/onnx_socket_robot.dir/clean

CMakeFiles/onnx_socket_robot.dir/depend:
	cd /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build /home/ti5/mechanical_arm_5_0_SDK/code_new_sdk/build/CMakeFiles/onnx_socket_robot.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/onnx_socket_robot.dir/depend

