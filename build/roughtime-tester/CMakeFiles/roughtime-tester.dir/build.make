# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /media/marco/cvsoc/craggy

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/marco/cvsoc/craggy/build

# Include any dependencies generated for this target.
include roughtime-tester/CMakeFiles/roughtime-tester.dir/depend.make

# Include the progress variables for this target.
include roughtime-tester/CMakeFiles/roughtime-tester.dir/progress.make

# Include the compile flags for this target's objects.
include roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make

roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o: ../roughtime-tester/others/log.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/others/log.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/others/log.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/others/log.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/others/log.c > CMakeFiles/roughtime-tester.dir/others/log.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/others/log.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/others/log.c -o CMakeFiles/roughtime-tester.dir/others/log.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o: ../roughtime-tester/serial_api/driver_ubx.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/serial_api/driver_ubx.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/serial_api/driver_ubx.c > CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/serial_api/driver_ubx.c -o CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o: ../roughtime-tester/serial_api/gpsd.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/serial_api/gpsd.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/serial_api/gpsd.c > CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/serial_api/gpsd.c -o CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o: ../roughtime-tester/serial_api/serial-driver.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/serial_api/serial-driver.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/serial_api/serial-driver.c > CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/serial_api/serial-driver.c -o CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o: ../roughtime-tester/serial_api/gpsutils.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/serial_api/gpsutils.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/serial_api/gpsutils.c > CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/serial_api/gpsutils.c -o CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o: ../roughtime-tester/serial_api/serial.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/serial_api/serial.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/serial_api/serial.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/serial_api/serial.c > CMakeFiles/roughtime-tester.dir/serial_api/serial.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/serial_api/serial.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/serial_api/serial.c -o CMakeFiles/roughtime-tester.dir/serial_api/serial.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o: ../roughtime-tester/serial_api/subframe.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/serial_api/subframe.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/serial_api/subframe.c > CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/serial_api/subframe.c -o CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o: ../roughtime-tester/base64.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/base64.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/base64.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/base64.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/base64.c > CMakeFiles/roughtime-tester.dir/base64.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/base64.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/base64.c -o CMakeFiles/roughtime-tester.dir/base64.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o


roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o: roughtime-tester/CMakeFiles/roughtime-tester.dir/flags.make
roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o: ../roughtime-tester/main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/roughtime-tester.dir/main.c.o   -c /media/marco/cvsoc/craggy/roughtime-tester/main.c

roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/roughtime-tester.dir/main.c.i"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /media/marco/cvsoc/craggy/roughtime-tester/main.c > CMakeFiles/roughtime-tester.dir/main.c.i

roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/roughtime-tester.dir/main.c.s"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /media/marco/cvsoc/craggy/roughtime-tester/main.c -o CMakeFiles/roughtime-tester.dir/main.c.s

roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.requires:

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.provides: roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.requires
	$(MAKE) -f roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.provides.build
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.provides

roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.provides.build: roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o


# Object files for target roughtime-tester
roughtime__tester_OBJECTS = \
"CMakeFiles/roughtime-tester.dir/others/log.c.o" \
"CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o" \
"CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o" \
"CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o" \
"CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o" \
"CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o" \
"CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o" \
"CMakeFiles/roughtime-tester.dir/base64.c.o" \
"CMakeFiles/roughtime-tester.dir/main.c.o"

# External object files for target roughtime-tester
roughtime__tester_EXTERNAL_OBJECTS =

roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/build.make
roughtime-tester/roughtime-tester: library/libcraggy.a
roughtime-tester/roughtime-tester: /usr/lib/x86_64-linux-gnu/libssl.so
roughtime-tester/roughtime-tester: /usr/lib/x86_64-linux-gnu/libcrypto.so
roughtime-tester/roughtime-tester: roughtime-tester/CMakeFiles/roughtime-tester.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/marco/cvsoc/craggy/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking C executable roughtime-tester"
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/roughtime-tester.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
roughtime-tester/CMakeFiles/roughtime-tester.dir/build: roughtime-tester/roughtime-tester

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/build

roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/others/log.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/driver_ubx.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsd.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial-driver.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/gpsutils.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/serial.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/serial_api/subframe.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/base64.c.o.requires
roughtime-tester/CMakeFiles/roughtime-tester.dir/requires: roughtime-tester/CMakeFiles/roughtime-tester.dir/main.c.o.requires

.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/requires

roughtime-tester/CMakeFiles/roughtime-tester.dir/clean:
	cd /media/marco/cvsoc/craggy/build/roughtime-tester && $(CMAKE_COMMAND) -P CMakeFiles/roughtime-tester.dir/cmake_clean.cmake
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/clean

roughtime-tester/CMakeFiles/roughtime-tester.dir/depend:
	cd /media/marco/cvsoc/craggy/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/marco/cvsoc/craggy /media/marco/cvsoc/craggy/roughtime-tester /media/marco/cvsoc/craggy/build /media/marco/cvsoc/craggy/build/roughtime-tester /media/marco/cvsoc/craggy/build/roughtime-tester/CMakeFiles/roughtime-tester.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : roughtime-tester/CMakeFiles/roughtime-tester.dir/depend
