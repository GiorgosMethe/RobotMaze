# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/build

# Include any dependencies generated for this target.
include CMakeFiles/VariantExample.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/VariantExample.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VariantExample.dir/flags.make

CMakeFiles/VariantExample.dir/examples/Variant.cpp.o: CMakeFiles/VariantExample.dir/flags.make
CMakeFiles/VariantExample.dir/examples/Variant.cpp.o: ../examples/Variant.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/VariantExample.dir/examples/Variant.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/VariantExample.dir/examples/Variant.cpp.o -c /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/examples/Variant.cpp

CMakeFiles/VariantExample.dir/examples/Variant.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VariantExample.dir/examples/Variant.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/examples/Variant.cpp > CMakeFiles/VariantExample.dir/examples/Variant.cpp.i

CMakeFiles/VariantExample.dir/examples/Variant.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VariantExample.dir/examples/Variant.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/examples/Variant.cpp -o CMakeFiles/VariantExample.dir/examples/Variant.cpp.s

CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.requires:
.PHONY : CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.requires

CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.provides: CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.requires
	$(MAKE) -f CMakeFiles/VariantExample.dir/build.make CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.provides.build
.PHONY : CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.provides

CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.provides.build: CMakeFiles/VariantExample.dir/examples/Variant.cpp.o

# Object files for target VariantExample
VariantExample_OBJECTS = \
"CMakeFiles/VariantExample.dir/examples/Variant.cpp.o"

# External object files for target VariantExample
VariantExample_EXTERNAL_OBJECTS =

../out/VariantExample: CMakeFiles/VariantExample.dir/examples/Variant.cpp.o
../out/VariantExample: CMakeFiles/VariantExample.dir/build.make
../out/VariantExample: CMakeFiles/VariantExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../out/VariantExample"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VariantExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VariantExample.dir/build: ../out/VariantExample
.PHONY : CMakeFiles/VariantExample.dir/build

CMakeFiles/VariantExample.dir/requires: CMakeFiles/VariantExample.dir/examples/Variant.cpp.o.requires
.PHONY : CMakeFiles/VariantExample.dir/requires

CMakeFiles/VariantExample.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/VariantExample.dir/cmake_clean.cmake
.PHONY : CMakeFiles/VariantExample.dir/clean

CMakeFiles/VariantExample.dir/depend:
	cd /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/build /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/build /home/methe/Workspace/RobotMaze/HyperNEAT/JGTL/build/CMakeFiles/VariantExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VariantExample.dir/depend
