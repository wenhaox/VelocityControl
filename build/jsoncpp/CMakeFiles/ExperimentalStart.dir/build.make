# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

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
CMAKE_COMMAND = /opt/homebrew/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/peter/Desktop/Research/zynq

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/peter/Desktop/Research/zynq/build

# Utility rule file for ExperimentalStart.

# Include any custom commands dependencies for this target.
include jsoncpp/CMakeFiles/ExperimentalStart.dir/compiler_depend.make

# Include the progress variables for this target.
include jsoncpp/CMakeFiles/ExperimentalStart.dir/progress.make

jsoncpp/CMakeFiles/ExperimentalStart:
	cd /Users/peter/Desktop/Research/zynq/build/jsoncpp && /opt/homebrew/bin/ctest -D ExperimentalStart

jsoncpp/CMakeFiles/ExperimentalStart.dir/codegen:
.PHONY : jsoncpp/CMakeFiles/ExperimentalStart.dir/codegen

ExperimentalStart: jsoncpp/CMakeFiles/ExperimentalStart
ExperimentalStart: jsoncpp/CMakeFiles/ExperimentalStart.dir/build.make
.PHONY : ExperimentalStart

# Rule to build all files generated by this target.
jsoncpp/CMakeFiles/ExperimentalStart.dir/build: ExperimentalStart
.PHONY : jsoncpp/CMakeFiles/ExperimentalStart.dir/build

jsoncpp/CMakeFiles/ExperimentalStart.dir/clean:
	cd /Users/peter/Desktop/Research/zynq/build/jsoncpp && $(CMAKE_COMMAND) -P CMakeFiles/ExperimentalStart.dir/cmake_clean.cmake
.PHONY : jsoncpp/CMakeFiles/ExperimentalStart.dir/clean

jsoncpp/CMakeFiles/ExperimentalStart.dir/depend:
	cd /Users/peter/Desktop/Research/zynq/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/peter/Desktop/Research/zynq /Users/peter/Desktop/Research/zynq/jsoncpp /Users/peter/Desktop/Research/zynq/build /Users/peter/Desktop/Research/zynq/build/jsoncpp /Users/peter/Desktop/Research/zynq/build/jsoncpp/CMakeFiles/ExperimentalStart.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : jsoncpp/CMakeFiles/ExperimentalStart.dir/depend

