# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/yuchen/SLAMbook/ch3/UseGeo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuchen/SLAMbook/ch3/UseGeo/build

# Include any dependencies generated for this target.
include CMakeFiles/UseGeometry.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/UseGeometry.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/UseGeometry.dir/flags.make

CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o: CMakeFiles/UseGeometry.dir/flags.make
CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o: ../UseGeometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuchen/SLAMbook/ch3/UseGeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o -c /home/yuchen/SLAMbook/ch3/UseGeo/UseGeometry.cpp

CMakeFiles/UseGeometry.dir/UseGeometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UseGeometry.dir/UseGeometry.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuchen/SLAMbook/ch3/UseGeo/UseGeometry.cpp > CMakeFiles/UseGeometry.dir/UseGeometry.cpp.i

CMakeFiles/UseGeometry.dir/UseGeometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UseGeometry.dir/UseGeometry.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuchen/SLAMbook/ch3/UseGeo/UseGeometry.cpp -o CMakeFiles/UseGeometry.dir/UseGeometry.cpp.s

CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.requires:

.PHONY : CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.requires

CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.provides: CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.requires
	$(MAKE) -f CMakeFiles/UseGeometry.dir/build.make CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.provides.build
.PHONY : CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.provides

CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.provides.build: CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o


# Object files for target UseGeometry
UseGeometry_OBJECTS = \
"CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o"

# External object files for target UseGeometry
UseGeometry_EXTERNAL_OBJECTS =

UseGeometry: CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o
UseGeometry: CMakeFiles/UseGeometry.dir/build.make
UseGeometry: CMakeFiles/UseGeometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuchen/SLAMbook/ch3/UseGeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable UseGeometry"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UseGeometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/UseGeometry.dir/build: UseGeometry

.PHONY : CMakeFiles/UseGeometry.dir/build

CMakeFiles/UseGeometry.dir/requires: CMakeFiles/UseGeometry.dir/UseGeometry.cpp.o.requires

.PHONY : CMakeFiles/UseGeometry.dir/requires

CMakeFiles/UseGeometry.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/UseGeometry.dir/cmake_clean.cmake
.PHONY : CMakeFiles/UseGeometry.dir/clean

CMakeFiles/UseGeometry.dir/depend:
	cd /home/yuchen/SLAMbook/ch3/UseGeo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuchen/SLAMbook/ch3/UseGeo /home/yuchen/SLAMbook/ch3/UseGeo /home/yuchen/SLAMbook/ch3/UseGeo/build /home/yuchen/SLAMbook/ch3/UseGeo/build /home/yuchen/SLAMbook/ch3/UseGeo/build/CMakeFiles/UseGeometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/UseGeometry.dir/depend

