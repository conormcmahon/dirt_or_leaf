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
CMAKE_SOURCE_DIR = /home/conor/dirt_or_leaf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/conor/dirt_or_leaf/build

# Include any dependencies generated for this target.
include CMakeFiles/las_filtering.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/las_filtering.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/las_filtering.dir/flags.make

CMakeFiles/las_filtering.dir/src/las_filtering.cpp.o: CMakeFiles/las_filtering.dir/flags.make
CMakeFiles/las_filtering.dir/src/las_filtering.cpp.o: ../src/las_filtering.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/conor/dirt_or_leaf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/las_filtering.dir/src/las_filtering.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/las_filtering.dir/src/las_filtering.cpp.o -c /home/conor/dirt_or_leaf/src/las_filtering.cpp

CMakeFiles/las_filtering.dir/src/las_filtering.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/las_filtering.dir/src/las_filtering.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/conor/dirt_or_leaf/src/las_filtering.cpp > CMakeFiles/las_filtering.dir/src/las_filtering.cpp.i

CMakeFiles/las_filtering.dir/src/las_filtering.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/las_filtering.dir/src/las_filtering.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/conor/dirt_or_leaf/src/las_filtering.cpp -o CMakeFiles/las_filtering.dir/src/las_filtering.cpp.s

# Object files for target las_filtering
las_filtering_OBJECTS = \
"CMakeFiles/las_filtering.dir/src/las_filtering.cpp.o"

# External object files for target las_filtering
las_filtering_EXTERNAL_OBJECTS =

liblas_filtering.a: CMakeFiles/las_filtering.dir/src/las_filtering.cpp.o
liblas_filtering.a: CMakeFiles/las_filtering.dir/build.make
liblas_filtering.a: CMakeFiles/las_filtering.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/conor/dirt_or_leaf/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblas_filtering.a"
	$(CMAKE_COMMAND) -P CMakeFiles/las_filtering.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/las_filtering.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/las_filtering.dir/build: liblas_filtering.a

.PHONY : CMakeFiles/las_filtering.dir/build

CMakeFiles/las_filtering.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/las_filtering.dir/cmake_clean.cmake
.PHONY : CMakeFiles/las_filtering.dir/clean

CMakeFiles/las_filtering.dir/depend:
	cd /home/conor/dirt_or_leaf/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/conor/dirt_or_leaf /home/conor/dirt_or_leaf /home/conor/dirt_or_leaf/build /home/conor/dirt_or_leaf/build /home/conor/dirt_or_leaf/build/CMakeFiles/las_filtering.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/las_filtering.dir/depend

