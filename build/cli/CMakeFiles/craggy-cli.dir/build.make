# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /home/marco/opt/cmake-3.23.0-rc4-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /home/marco/opt/cmake-3.23.0-rc4-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marco/GITS/craggy

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marco/GITS/craggy/build

# Include any dependencies generated for this target.
include cli/CMakeFiles/craggy-cli.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include cli/CMakeFiles/craggy-cli.dir/compiler_depend.make

# Include the progress variables for this target.
include cli/CMakeFiles/craggy-cli.dir/progress.make

# Include the compile flags for this target's objects.
include cli/CMakeFiles/craggy-cli.dir/flags.make

cli/CMakeFiles/craggy-cli.dir/base64.c.o: cli/CMakeFiles/craggy-cli.dir/flags.make
cli/CMakeFiles/craggy-cli.dir/base64.c.o: ../cli/base64.c
cli/CMakeFiles/craggy-cli.dir/base64.c.o: cli/CMakeFiles/craggy-cli.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marco/GITS/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object cli/CMakeFiles/craggy-cli.dir/base64.c.o"
	cd /home/marco/GITS/craggy/build/cli && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT cli/CMakeFiles/craggy-cli.dir/base64.c.o -MF CMakeFiles/craggy-cli.dir/base64.c.o.d -o CMakeFiles/craggy-cli.dir/base64.c.o -c /home/marco/GITS/craggy/cli/base64.c

cli/CMakeFiles/craggy-cli.dir/base64.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/craggy-cli.dir/base64.c.i"
	cd /home/marco/GITS/craggy/build/cli && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/marco/GITS/craggy/cli/base64.c > CMakeFiles/craggy-cli.dir/base64.c.i

cli/CMakeFiles/craggy-cli.dir/base64.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/craggy-cli.dir/base64.c.s"
	cd /home/marco/GITS/craggy/build/cli && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/marco/GITS/craggy/cli/base64.c -o CMakeFiles/craggy-cli.dir/base64.c.s

cli/CMakeFiles/craggy-cli.dir/main.c.o: cli/CMakeFiles/craggy-cli.dir/flags.make
cli/CMakeFiles/craggy-cli.dir/main.c.o: ../cli/main.c
cli/CMakeFiles/craggy-cli.dir/main.c.o: cli/CMakeFiles/craggy-cli.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marco/GITS/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object cli/CMakeFiles/craggy-cli.dir/main.c.o"
	cd /home/marco/GITS/craggy/build/cli && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT cli/CMakeFiles/craggy-cli.dir/main.c.o -MF CMakeFiles/craggy-cli.dir/main.c.o.d -o CMakeFiles/craggy-cli.dir/main.c.o -c /home/marco/GITS/craggy/cli/main.c

cli/CMakeFiles/craggy-cli.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/craggy-cli.dir/main.c.i"
	cd /home/marco/GITS/craggy/build/cli && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/marco/GITS/craggy/cli/main.c > CMakeFiles/craggy-cli.dir/main.c.i

cli/CMakeFiles/craggy-cli.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/craggy-cli.dir/main.c.s"
	cd /home/marco/GITS/craggy/build/cli && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/marco/GITS/craggy/cli/main.c -o CMakeFiles/craggy-cli.dir/main.c.s

# Object files for target craggy-cli
craggy__cli_OBJECTS = \
"CMakeFiles/craggy-cli.dir/base64.c.o" \
"CMakeFiles/craggy-cli.dir/main.c.o"

# External object files for target craggy-cli
craggy__cli_EXTERNAL_OBJECTS =

cli/craggy-cli: cli/CMakeFiles/craggy-cli.dir/base64.c.o
cli/craggy-cli: cli/CMakeFiles/craggy-cli.dir/main.c.o
cli/craggy-cli: cli/CMakeFiles/craggy-cli.dir/build.make
cli/craggy-cli: library/libcraggy.a
cli/craggy-cli: /usr/lib/x86_64-linux-gnu/libssl.so
cli/craggy-cli: /usr/lib/x86_64-linux-gnu/libssl.so
cli/craggy-cli: /usr/lib/x86_64-linux-gnu/libcrypto.so
cli/craggy-cli: /usr/lib/x86_64-linux-gnu/libcrypto.so
cli/craggy-cli: cli/CMakeFiles/craggy-cli.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/marco/GITS/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable craggy-cli"
	cd /home/marco/GITS/craggy/build/cli && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/craggy-cli.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cli/CMakeFiles/craggy-cli.dir/build: cli/craggy-cli
.PHONY : cli/CMakeFiles/craggy-cli.dir/build

cli/CMakeFiles/craggy-cli.dir/clean:
	cd /home/marco/GITS/craggy/build/cli && $(CMAKE_COMMAND) -P CMakeFiles/craggy-cli.dir/cmake_clean.cmake
.PHONY : cli/CMakeFiles/craggy-cli.dir/clean

cli/CMakeFiles/craggy-cli.dir/depend:
	cd /home/marco/GITS/craggy/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marco/GITS/craggy /home/marco/GITS/craggy/cli /home/marco/GITS/craggy/build /home/marco/GITS/craggy/build/cli /home/marco/GITS/craggy/build/cli/CMakeFiles/craggy-cli.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cli/CMakeFiles/craggy-cli.dir/depend

