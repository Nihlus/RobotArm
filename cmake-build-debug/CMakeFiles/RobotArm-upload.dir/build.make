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

# Utility rule file for RobotArm-upload.

# Include the progress variables for this target.
include CMakeFiles/RobotArm-upload.dir/progress.make

CMakeFiles/RobotArm-upload: RobotArm.elf
	/opt/arduino-1.8.0/hardware/tools/avr/bin/avrdude -C/opt/arduino-1.8.0/hardware/tools/avr/etc/avrdude.conf -patmega328p -carduino -b115200 -P/dev/ttyACM0 -D -V -Uflash:w:/home/jarl/Programming/RobotArm/cmake-build-debug/RobotArm.hex:i -Ueeprom:w:/home/jarl/Programming/RobotArm/cmake-build-debug/RobotArm.eep:i

RobotArm-upload: CMakeFiles/RobotArm-upload
RobotArm-upload: CMakeFiles/RobotArm-upload.dir/build.make

.PHONY : RobotArm-upload

# Rule to build all files generated by this target.
CMakeFiles/RobotArm-upload.dir/build: RobotArm-upload

.PHONY : CMakeFiles/RobotArm-upload.dir/build

CMakeFiles/RobotArm-upload.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RobotArm-upload.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RobotArm-upload.dir/clean

CMakeFiles/RobotArm-upload.dir/depend:
	cd /home/jarl/Programming/RobotArm/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jarl/Programming/RobotArm /home/jarl/Programming/RobotArm /home/jarl/Programming/RobotArm/cmake-build-debug /home/jarl/Programming/RobotArm/cmake-build-debug /home/jarl/Programming/RobotArm/cmake-build-debug/CMakeFiles/RobotArm-upload.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RobotArm-upload.dir/depend

