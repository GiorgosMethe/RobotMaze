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
include CMakeFiles/ruler.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ruler.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ruler.dir/flags.make

CMakeFiles/ruler.dir/examples/ruler.cpp.o: CMakeFiles/ruler.dir/flags.make
CMakeFiles/ruler.dir/examples/ruler.cpp.o: ../examples/ruler.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ruler.dir/examples/ruler.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ruler.dir/examples/ruler.cpp.o -c /home/methe/Workspace/RobotMaze/HyperNEAT/Board/examples/ruler.cpp

CMakeFiles/ruler.dir/examples/ruler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ruler.dir/examples/ruler.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/methe/Workspace/RobotMaze/HyperNEAT/Board/examples/ruler.cpp > CMakeFiles/ruler.dir/examples/ruler.cpp.i

CMakeFiles/ruler.dir/examples/ruler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ruler.dir/examples/ruler.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/methe/Workspace/RobotMaze/HyperNEAT/Board/examples/ruler.cpp -o CMakeFiles/ruler.dir/examples/ruler.cpp.s

CMakeFiles/ruler.dir/examples/ruler.cpp.o.requires:
.PHONY : CMakeFiles/ruler.dir/examples/ruler.cpp.o.requires

CMakeFiles/ruler.dir/examples/ruler.cpp.o.provides: CMakeFiles/ruler.dir/examples/ruler.cpp.o.requires
	$(MAKE) -f CMakeFiles/ruler.dir/build.make CMakeFiles/ruler.dir/examples/ruler.cpp.o.provides.build
.PHONY : CMakeFiles/ruler.dir/examples/ruler.cpp.o.provides

CMakeFiles/ruler.dir/examples/ruler.cpp.o.provides.build: CMakeFiles/ruler.dir/examples/ruler.cpp.o

# Object files for target ruler
ruler_OBJECTS = \
"CMakeFiles/ruler.dir/examples/ruler.cpp.o"

# External object files for target ruler
ruler_EXTERNAL_OBJECTS =

../out/ruler: CMakeFiles/ruler.dir/examples/ruler.cpp.o
../out/ruler: CMakeFiles/ruler.dir/build.make
../out/ruler: ../out/libboard.a
../out/ruler: CMakeFiles/ruler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../out/ruler"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ruler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ruler.dir/build: ../out/ruler
.PHONY : CMakeFiles/ruler.dir/build

CMakeFiles/ruler.dir/requires: CMakeFiles/ruler.dir/examples/ruler.cpp.o.requires
.PHONY : CMakeFiles/ruler.dir/requires

CMakeFiles/ruler.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ruler.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ruler.dir/clean

CMakeFiles/ruler.dir/depend:
	cd /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/methe/Workspace/RobotMaze/HyperNEAT/Board /home/methe/Workspace/RobotMaze/HyperNEAT/Board /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build /home/methe/Workspace/RobotMaze/HyperNEAT/Board/build/CMakeFiles/ruler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ruler.dir/depend
