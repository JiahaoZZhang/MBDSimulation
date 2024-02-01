# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jiahao/vanetza

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jiahao/vanetza

# Include any dependencies generated for this target.
include vanetza/asn1/CMakeFiles/asn1_its.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include vanetza/asn1/CMakeFiles/asn1_its.dir/compiler_depend.make

# Include the progress variables for this target.
include vanetza/asn1/CMakeFiles/asn1_its.dir/progress.make

# Include the compile flags for this target's objects.
include vanetza/asn1/CMakeFiles/asn1_its.dir/flags.make

vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o: vanetza/asn1/CMakeFiles/asn1_its.dir/flags.make
vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o: vanetza/asn1/asn1_its_no_sources.c
vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o: vanetza/asn1/CMakeFiles/asn1_its.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o"
	cd /home/jiahao/vanetza/vanetza/asn1 && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o -MF CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o.d -o CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o -c /home/jiahao/vanetza/vanetza/asn1/asn1_its_no_sources.c

vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.i"
	cd /home/jiahao/vanetza/vanetza/asn1 && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jiahao/vanetza/vanetza/asn1/asn1_its_no_sources.c > CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.i

vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.s"
	cd /home/jiahao/vanetza/vanetza/asn1 && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jiahao/vanetza/vanetza/asn1/asn1_its_no_sources.c -o CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.s

# Object files for target asn1_its
asn1_its_OBJECTS = \
"CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o"

# External object files for target asn1_its
asn1_its_EXTERNAL_OBJECTS =

lib/static/libvanetza_asn1_its.a: vanetza/asn1/CMakeFiles/asn1_its.dir/asn1_its_no_sources.c.o
lib/static/libvanetza_asn1_its.a: vanetza/asn1/CMakeFiles/asn1_its.dir/build.make
lib/static/libvanetza_asn1_its.a: vanetza/asn1/CMakeFiles/asn1_its.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library ../../lib/static/libvanetza_asn1_its.a"
	cd /home/jiahao/vanetza/vanetza/asn1 && $(CMAKE_COMMAND) -P CMakeFiles/asn1_its.dir/cmake_clean_target.cmake
	cd /home/jiahao/vanetza/vanetza/asn1 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/asn1_its.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vanetza/asn1/CMakeFiles/asn1_its.dir/build: lib/static/libvanetza_asn1_its.a
.PHONY : vanetza/asn1/CMakeFiles/asn1_its.dir/build

vanetza/asn1/CMakeFiles/asn1_its.dir/clean:
	cd /home/jiahao/vanetza/vanetza/asn1 && $(CMAKE_COMMAND) -P CMakeFiles/asn1_its.dir/cmake_clean.cmake
.PHONY : vanetza/asn1/CMakeFiles/asn1_its.dir/clean

vanetza/asn1/CMakeFiles/asn1_its.dir/depend:
	cd /home/jiahao/vanetza && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jiahao/vanetza /home/jiahao/vanetza/vanetza/asn1 /home/jiahao/vanetza /home/jiahao/vanetza/vanetza/asn1 /home/jiahao/vanetza/vanetza/asn1/CMakeFiles/asn1_its.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vanetza/asn1/CMakeFiles/asn1_its.dir/depend

