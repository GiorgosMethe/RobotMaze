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
CMAKE_SOURCE_DIR = /home/methe/Workspace/RobotMaze/HyperNEAT/Board

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build

# Include any dependencies generated for this target.
include CMakeFiles/clipping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/clipping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/clipping.dir/flags.make

CMakeFiles/clipping.dir/examples/clipping.cpp.o: CMakeFiles/clipping.dir/flags.make
CMakeFiles/clipping.dir/examples/clipping.cpp.o: ../examples/clipping.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/clipping.dir/examples/clipping.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/clipping.dir/examples/clipping.cpp.o -c /home/methe/Workspace/RobotMaze/HyperNEAT/Board/examples/clipping.cpp

CMakeFiles/clipping.dir/examples/clipping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/clipping.dir/examples/clipping.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/methe/Workspace/RobotMaze/HyperNEAT/Board/examples/clipping.cpp > CMakeFiles/clipping.dir/examples/clipping.cpp.i

CMakeFiles/clipping.dir/examples/clipping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/clipping.dir/examples/clipping.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/methe/Workspace/RobotMaze/HyperNEAT/Board/examples/clipping.cpp -o CMakeFiles/clipping.dir/examples/clipping.cpp.s

CMakeFiles/clipping.dir/examples/clipping.cpp.o.requires:
.PHONY : CMakeFiles/clipping.dir/examples/clipping.cpp.o.requires

CMakeFiles/clipping.dir/examples/clipping.cpp.o.provides: CMakeFiles/clipping.dir/examples/clipping.cpp.o.requires
	$(MAKE) -f CMakeFiles/clipping.dir/build.make CMakeFiles/clipping.dir/examples/clipping.cpp.o.provides.build
.PHONY : CMakeFiles/clipping.dir/examples/clipping.cpp.o.provides

CMakeFiles/clipping.dir/examples/clipping.cpp.o.provides.build: CMakeFiles/clipping.dir/examples/clipping.cpp.o

# Object files for target clipping
clipping_OBJECTS = \
"CMakeFiles/clipping.dir/examples/clipping.cpp.o"

# External object files for target clipping
clipping_EXTERNAL_OBJECTS =

../out/clipping: CMakeFiles/clipping.dir/examples/clipping.cpp.o
../out/clipping: CMakeFiles/clipping.dir/build.make
../out/clipping: ../out/libboard.a
../out/clipping: CMakeFiles/clipping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../out/clipping"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/clipping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/clipping.dir/build: ../out/clipping
.PHONY : CMakeFiles/clipping.dir/build

CMakeFiles/clipping.dir/requires: CMakeFiles/clipping.dir/examples/clipping.cpp.o.requires
.PHONY : CMakeFiles/clipping.dir/requires

CMakeFiles/clipping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clipping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clipping.dir/clean

CMakeFiles/clipping.dir/depend:
	cd /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/methe/Workspace/RobotMaze/HyperNEAT/Board /home/methe/Workspace/RobotMaze/HyperNEAT/Board /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build/CMakeFiles/clipping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clipping.dir/depend

