# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /home/jarl/Programming/IDEs/apps/CLion/ch-1/171.2822.8/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/jarl/Programming/IDEs/apps/CLion/ch-1/171.2822.8/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jarl/Programming/RobotArm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jarl/Programming/RobotArm/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/uno_AFMotor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/uno_AFMotor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/uno_AFMotor.dir/flags.make

CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj: CMakeFiles/uno_AFMotor.dir/flags.make
CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj: ../lib/AFMotor/AFMotor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jarl/Programming/RobotArm/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj"
	/usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj -c /home/jarl/Programming/RobotArm/lib/AFMotor/AFMotor.cpp

CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.i"
	/usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jarl/Programming/RobotArm/lib/AFMotor/AFMotor.cpp > CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.i

CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.s"
	/usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jarl/Programming/RobotArm/lib/AFMotor/AFMotor.cpp -o CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.s

CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.requires:

.PHONY : CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.requires

CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.provides: CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.requires
	$(MAKE) -f CMakeFiles/uno_AFMotor.dir/build.make CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.provides.build
.PHONY : CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.provides

CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.provides.build: CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj


# Object files for target uno_AFMotor
uno_AFMotor_OBJECTS = \
"CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj"

# External object files for target uno_AFMotor
uno_AFMotor_EXTERNAL_OBJECTS =

libuno_AFMotor.a: CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj
libuno_AFMotor.a: CMakeFiles/uno_AFMotor.dir/build.make
libuno_AFMotor.a: CMakeFiles/uno_AFMotor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jarl/Programming/RobotArm/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libuno_AFMotor.a"
	$(CMAKE_COMMAND) -P CMakeFiles/uno_AFMotor.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uno_AFMotor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/uno_AFMotor.dir/build: libuno_AFMotor.a

.PHONY : CMakeFiles/uno_AFMotor.dir/build

CMakeFiles/uno_AFMotor.dir/requires: CMakeFiles/uno_AFMotor.dir/lib/AFMotor/AFMotor.cpp.obj.requires

.PHONY : CMakeFiles/uno_AFMotor.dir/requires

CMakeFiles/uno_AFMotor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/uno_AFMotor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/uno_AFMotor.dir/clean

CMakeFiles/uno_AFMotor.dir/depend:
	cd /home/jarl/Programming/RobotArm/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jarl/Programming/RobotArm /home/jarl/Programming/RobotArm /home/jarl/Programming/RobotArm/cmake-build-debug /home/jarl/Programming/RobotArm/cmake-build-debug /home/jarl/Programming/RobotArm/cmake-build-debug/CMakeFiles/uno_AFMotor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/uno_AFMotor.dir/depend

