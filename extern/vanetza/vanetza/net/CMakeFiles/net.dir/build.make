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
include vanetza/net/CMakeFiles/net.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include vanetza/net/CMakeFiles/net.dir/compiler_depend.make

# Include the progress variables for this target.
include vanetza/net/CMakeFiles/net.dir/progress.make

# Include the compile flags for this target's objects.
include vanetza/net/CMakeFiles/net.dir/flags.make

vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.o: vanetza/net/buffer_packet.cpp
vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.o -MF CMakeFiles/net.dir/buffer_packet.cpp.o.d -o CMakeFiles/net.dir/buffer_packet.cpp.o -c /home/jiahao/vanetza/vanetza/net/buffer_packet.cpp

vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/buffer_packet.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/buffer_packet.cpp > CMakeFiles/net.dir/buffer_packet.cpp.i

vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/buffer_packet.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/buffer_packet.cpp -o CMakeFiles/net.dir/buffer_packet.cpp.s

vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.o: vanetza/net/chunk_packet.cpp
vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.o -MF CMakeFiles/net.dir/chunk_packet.cpp.o.d -o CMakeFiles/net.dir/chunk_packet.cpp.o -c /home/jiahao/vanetza/vanetza/net/chunk_packet.cpp

vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/chunk_packet.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/chunk_packet.cpp > CMakeFiles/net.dir/chunk_packet.cpp.i

vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/chunk_packet.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/chunk_packet.cpp -o CMakeFiles/net.dir/chunk_packet.cpp.s

vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.o: vanetza/net/cohesive_packet.cpp
vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.o -MF CMakeFiles/net.dir/cohesive_packet.cpp.o.d -o CMakeFiles/net.dir/cohesive_packet.cpp.o -c /home/jiahao/vanetza/vanetza/net/cohesive_packet.cpp

vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/cohesive_packet.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/cohesive_packet.cpp > CMakeFiles/net.dir/cohesive_packet.cpp.i

vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/cohesive_packet.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/cohesive_packet.cpp -o CMakeFiles/net.dir/cohesive_packet.cpp.s

vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.o: vanetza/net/ethernet_header.cpp
vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.o -MF CMakeFiles/net.dir/ethernet_header.cpp.o.d -o CMakeFiles/net.dir/ethernet_header.cpp.o -c /home/jiahao/vanetza/vanetza/net/ethernet_header.cpp

vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/ethernet_header.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/ethernet_header.cpp > CMakeFiles/net.dir/ethernet_header.cpp.i

vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/ethernet_header.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/ethernet_header.cpp -o CMakeFiles/net.dir/ethernet_header.cpp.s

vanetza/net/CMakeFiles/net.dir/mac_address.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/mac_address.cpp.o: vanetza/net/mac_address.cpp
vanetza/net/CMakeFiles/net.dir/mac_address.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object vanetza/net/CMakeFiles/net.dir/mac_address.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/mac_address.cpp.o -MF CMakeFiles/net.dir/mac_address.cpp.o.d -o CMakeFiles/net.dir/mac_address.cpp.o -c /home/jiahao/vanetza/vanetza/net/mac_address.cpp

vanetza/net/CMakeFiles/net.dir/mac_address.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/mac_address.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/mac_address.cpp > CMakeFiles/net.dir/mac_address.cpp.i

vanetza/net/CMakeFiles/net.dir/mac_address.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/mac_address.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/mac_address.cpp -o CMakeFiles/net.dir/mac_address.cpp.s

vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.o: vanetza/net/packet_variant.cpp
vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.o -MF CMakeFiles/net.dir/packet_variant.cpp.o.d -o CMakeFiles/net.dir/packet_variant.cpp.o -c /home/jiahao/vanetza/vanetza/net/packet_variant.cpp

vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/packet_variant.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/packet_variant.cpp > CMakeFiles/net.dir/packet_variant.cpp.i

vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/packet_variant.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/packet_variant.cpp -o CMakeFiles/net.dir/packet_variant.cpp.s

vanetza/net/CMakeFiles/net.dir/packet.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/packet.cpp.o: vanetza/net/packet.cpp
vanetza/net/CMakeFiles/net.dir/packet.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object vanetza/net/CMakeFiles/net.dir/packet.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/packet.cpp.o -MF CMakeFiles/net.dir/packet.cpp.o.d -o CMakeFiles/net.dir/packet.cpp.o -c /home/jiahao/vanetza/vanetza/net/packet.cpp

vanetza/net/CMakeFiles/net.dir/packet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/packet.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/packet.cpp > CMakeFiles/net.dir/packet.cpp.i

vanetza/net/CMakeFiles/net.dir/packet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/packet.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/packet.cpp -o CMakeFiles/net.dir/packet.cpp.s

vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.o: vanetza/net/proxy_header.cpp
vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.o -MF CMakeFiles/net.dir/proxy_header.cpp.o.d -o CMakeFiles/net.dir/proxy_header.cpp.o -c /home/jiahao/vanetza/vanetza/net/proxy_header.cpp

vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/proxy_header.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/proxy_header.cpp > CMakeFiles/net.dir/proxy_header.cpp.i

vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/proxy_header.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/proxy_header.cpp -o CMakeFiles/net.dir/proxy_header.cpp.s

vanetza/net/CMakeFiles/net.dir/io_vector.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/io_vector.cpp.o: vanetza/net/io_vector.cpp
vanetza/net/CMakeFiles/net.dir/io_vector.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object vanetza/net/CMakeFiles/net.dir/io_vector.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/io_vector.cpp.o -MF CMakeFiles/net.dir/io_vector.cpp.o.d -o CMakeFiles/net.dir/io_vector.cpp.o -c /home/jiahao/vanetza/vanetza/net/io_vector.cpp

vanetza/net/CMakeFiles/net.dir/io_vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/io_vector.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/io_vector.cpp > CMakeFiles/net.dir/io_vector.cpp.i

vanetza/net/CMakeFiles/net.dir/io_vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/io_vector.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/io_vector.cpp -o CMakeFiles/net.dir/io_vector.cpp.s

vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.o: vanetza/net/CMakeFiles/net.dir/flags.make
vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.o: vanetza/net/sockaddr.cpp
vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.o: vanetza/net/CMakeFiles/net.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.o"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.o -MF CMakeFiles/net.dir/sockaddr.cpp.o.d -o CMakeFiles/net.dir/sockaddr.cpp.o -c /home/jiahao/vanetza/vanetza/net/sockaddr.cpp

vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/net.dir/sockaddr.cpp.i"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiahao/vanetza/vanetza/net/sockaddr.cpp > CMakeFiles/net.dir/sockaddr.cpp.i

vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/net.dir/sockaddr.cpp.s"
	cd /home/jiahao/vanetza/vanetza/net && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiahao/vanetza/vanetza/net/sockaddr.cpp -o CMakeFiles/net.dir/sockaddr.cpp.s

# Object files for target net
net_OBJECTS = \
"CMakeFiles/net.dir/buffer_packet.cpp.o" \
"CMakeFiles/net.dir/chunk_packet.cpp.o" \
"CMakeFiles/net.dir/cohesive_packet.cpp.o" \
"CMakeFiles/net.dir/ethernet_header.cpp.o" \
"CMakeFiles/net.dir/mac_address.cpp.o" \
"CMakeFiles/net.dir/packet_variant.cpp.o" \
"CMakeFiles/net.dir/packet.cpp.o" \
"CMakeFiles/net.dir/proxy_header.cpp.o" \
"CMakeFiles/net.dir/io_vector.cpp.o" \
"CMakeFiles/net.dir/sockaddr.cpp.o"

# External object files for target net
net_EXTERNAL_OBJECTS =

lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/buffer_packet.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/chunk_packet.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/cohesive_packet.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/ethernet_header.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/mac_address.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/packet_variant.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/packet.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/proxy_header.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/io_vector.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/sockaddr.cpp.o
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/build.make
lib/static/libvanetza_net.a: vanetza/net/CMakeFiles/net.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jiahao/vanetza/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX static library ../../lib/static/libvanetza_net.a"
	cd /home/jiahao/vanetza/vanetza/net && $(CMAKE_COMMAND) -P CMakeFiles/net.dir/cmake_clean_target.cmake
	cd /home/jiahao/vanetza/vanetza/net && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/net.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vanetza/net/CMakeFiles/net.dir/build: lib/static/libvanetza_net.a
.PHONY : vanetza/net/CMakeFiles/net.dir/build

vanetza/net/CMakeFiles/net.dir/clean:
	cd /home/jiahao/vanetza/vanetza/net && $(CMAKE_COMMAND) -P CMakeFiles/net.dir/cmake_clean.cmake
.PHONY : vanetza/net/CMakeFiles/net.dir/clean

vanetza/net/CMakeFiles/net.dir/depend:
	cd /home/jiahao/vanetza && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jiahao/vanetza /home/jiahao/vanetza/vanetza/net /home/jiahao/vanetza /home/jiahao/vanetza/vanetza/net /home/jiahao/vanetza/vanetza/net/CMakeFiles/net.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vanetza/net/CMakeFiles/net.dir/depend
