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
CMAKE_SOURCE_DIR = /home/tao/Desktop/onboard

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tao/Desktop/onboard/build

# Include any dependencies generated for this target.
include CMakeFiles/onboard.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/onboard.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/onboard.dir/flags.make

CMakeFiles/onboard.dir/src/log_node.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/log_node.cpp.o: ../src/log_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/onboard.dir/src/log_node.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/log_node.cpp.o -c /home/tao/Desktop/onboard/src/log_node.cpp

CMakeFiles/onboard.dir/src/log_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/log_node.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/log_node.cpp > CMakeFiles/onboard.dir/src/log_node.cpp.i

CMakeFiles/onboard.dir/src/log_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/log_node.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/log_node.cpp -o CMakeFiles/onboard.dir/src/log_node.cpp.s

CMakeFiles/onboard.dir/src/log_node.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/log_node.cpp.o.requires

CMakeFiles/onboard.dir/src/log_node.cpp.o.provides: CMakeFiles/onboard.dir/src/log_node.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/log_node.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/log_node.cpp.o.provides

CMakeFiles/onboard.dir/src/log_node.cpp.o.provides.build: CMakeFiles/onboard.dir/src/log_node.cpp.o


CMakeFiles/onboard.dir/src/clock.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/clock.cpp.o: ../src/clock.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/onboard.dir/src/clock.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/clock.cpp.o -c /home/tao/Desktop/onboard/src/clock.cpp

CMakeFiles/onboard.dir/src/clock.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/clock.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/clock.cpp > CMakeFiles/onboard.dir/src/clock.cpp.i

CMakeFiles/onboard.dir/src/clock.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/clock.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/clock.cpp -o CMakeFiles/onboard.dir/src/clock.cpp.s

CMakeFiles/onboard.dir/src/clock.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/clock.cpp.o.requires

CMakeFiles/onboard.dir/src/clock.cpp.o.provides: CMakeFiles/onboard.dir/src/clock.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/clock.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/clock.cpp.o.provides

CMakeFiles/onboard.dir/src/clock.cpp.o.provides.build: CMakeFiles/onboard.dir/src/clock.cpp.o


CMakeFiles/onboard.dir/src/socket_nodes.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/socket_nodes.cpp.o: ../src/socket_nodes.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/onboard.dir/src/socket_nodes.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/socket_nodes.cpp.o -c /home/tao/Desktop/onboard/src/socket_nodes.cpp

CMakeFiles/onboard.dir/src/socket_nodes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/socket_nodes.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/socket_nodes.cpp > CMakeFiles/onboard.dir/src/socket_nodes.cpp.i

CMakeFiles/onboard.dir/src/socket_nodes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/socket_nodes.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/socket_nodes.cpp -o CMakeFiles/onboard.dir/src/socket_nodes.cpp.s

CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.requires

CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.provides: CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.provides

CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.provides.build: CMakeFiles/onboard.dir/src/socket_nodes.cpp.o


CMakeFiles/onboard.dir/src/topics.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/topics.cpp.o: ../src/topics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/onboard.dir/src/topics.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/topics.cpp.o -c /home/tao/Desktop/onboard/src/topics.cpp

CMakeFiles/onboard.dir/src/topics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/topics.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/topics.cpp > CMakeFiles/onboard.dir/src/topics.cpp.i

CMakeFiles/onboard.dir/src/topics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/topics.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/topics.cpp -o CMakeFiles/onboard.dir/src/topics.cpp.s

CMakeFiles/onboard.dir/src/topics.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/topics.cpp.o.requires

CMakeFiles/onboard.dir/src/topics.cpp.o.provides: CMakeFiles/onboard.dir/src/topics.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/topics.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/topics.cpp.o.provides

CMakeFiles/onboard.dir/src/topics.cpp.o.provides.build: CMakeFiles/onboard.dir/src/topics.cpp.o


CMakeFiles/onboard.dir/src/camera_node.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/camera_node.cpp.o: ../src/camera_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/onboard.dir/src/camera_node.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/camera_node.cpp.o -c /home/tao/Desktop/onboard/src/camera_node.cpp

CMakeFiles/onboard.dir/src/camera_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/camera_node.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/camera_node.cpp > CMakeFiles/onboard.dir/src/camera_node.cpp.i

CMakeFiles/onboard.dir/src/camera_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/camera_node.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/camera_node.cpp -o CMakeFiles/onboard.dir/src/camera_node.cpp.s

CMakeFiles/onboard.dir/src/camera_node.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/camera_node.cpp.o.requires

CMakeFiles/onboard.dir/src/camera_node.cpp.o.provides: CMakeFiles/onboard.dir/src/camera_node.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/camera_node.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/camera_node.cpp.o.provides

CMakeFiles/onboard.dir/src/camera_node.cpp.o.provides.build: CMakeFiles/onboard.dir/src/camera_node.cpp.o


CMakeFiles/onboard.dir/src/main.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/onboard.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/main.cpp.o -c /home/tao/Desktop/onboard/src/main.cpp

CMakeFiles/onboard.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/main.cpp > CMakeFiles/onboard.dir/src/main.cpp.i

CMakeFiles/onboard.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/main.cpp -o CMakeFiles/onboard.dir/src/main.cpp.s

CMakeFiles/onboard.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/main.cpp.o.requires

CMakeFiles/onboard.dir/src/main.cpp.o.provides: CMakeFiles/onboard.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/main.cpp.o.provides

CMakeFiles/onboard.dir/src/main.cpp.o.provides.build: CMakeFiles/onboard.dir/src/main.cpp.o


CMakeFiles/onboard.dir/src/control_node.cpp.o: CMakeFiles/onboard.dir/flags.make
CMakeFiles/onboard.dir/src/control_node.cpp.o: ../src/control_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/onboard.dir/src/control_node.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/onboard.dir/src/control_node.cpp.o -c /home/tao/Desktop/onboard/src/control_node.cpp

CMakeFiles/onboard.dir/src/control_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/onboard.dir/src/control_node.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tao/Desktop/onboard/src/control_node.cpp > CMakeFiles/onboard.dir/src/control_node.cpp.i

CMakeFiles/onboard.dir/src/control_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/onboard.dir/src/control_node.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tao/Desktop/onboard/src/control_node.cpp -o CMakeFiles/onboard.dir/src/control_node.cpp.s

CMakeFiles/onboard.dir/src/control_node.cpp.o.requires:

.PHONY : CMakeFiles/onboard.dir/src/control_node.cpp.o.requires

CMakeFiles/onboard.dir/src/control_node.cpp.o.provides: CMakeFiles/onboard.dir/src/control_node.cpp.o.requires
	$(MAKE) -f CMakeFiles/onboard.dir/build.make CMakeFiles/onboard.dir/src/control_node.cpp.o.provides.build
.PHONY : CMakeFiles/onboard.dir/src/control_node.cpp.o.provides

CMakeFiles/onboard.dir/src/control_node.cpp.o.provides.build: CMakeFiles/onboard.dir/src/control_node.cpp.o


# Object files for target onboard
onboard_OBJECTS = \
"CMakeFiles/onboard.dir/src/log_node.cpp.o" \
"CMakeFiles/onboard.dir/src/clock.cpp.o" \
"CMakeFiles/onboard.dir/src/socket_nodes.cpp.o" \
"CMakeFiles/onboard.dir/src/topics.cpp.o" \
"CMakeFiles/onboard.dir/src/camera_node.cpp.o" \
"CMakeFiles/onboard.dir/src/main.cpp.o" \
"CMakeFiles/onboard.dir/src/control_node.cpp.o"

# External object files for target onboard
onboard_EXTERNAL_OBJECTS =

onboard: CMakeFiles/onboard.dir/src/log_node.cpp.o
onboard: CMakeFiles/onboard.dir/src/clock.cpp.o
onboard: CMakeFiles/onboard.dir/src/socket_nodes.cpp.o
onboard: CMakeFiles/onboard.dir/src/topics.cpp.o
onboard: CMakeFiles/onboard.dir/src/camera_node.cpp.o
onboard: CMakeFiles/onboard.dir/src/main.cpp.o
onboard: CMakeFiles/onboard.dir/src/control_node.cpp.o
onboard: CMakeFiles/onboard.dir/build.make
onboard: /usr/local/lib/libopencv_gapi.so.4.1.1
onboard: /usr/local/lib/libopencv_stitching.so.4.1.1
onboard: /usr/local/lib/libopencv_freetype.so.4.1.1
onboard: /usr/local/lib/libopencv_face.so.4.1.1
onboard: /usr/local/lib/libopencv_line_descriptor.so.4.1.1
onboard: /usr/local/lib/libopencv_xphoto.so.4.1.1
onboard: /usr/local/lib/libopencv_xobjdetect.so.4.1.1
onboard: /usr/local/lib/libopencv_rgbd.so.4.1.1
onboard: /usr/local/lib/libopencv_surface_matching.so.4.1.1
onboard: /usr/local/lib/libopencv_img_hash.so.4.1.1
onboard: /usr/local/lib/libopencv_dnn_objdetect.so.4.1.1
onboard: /usr/local/lib/libopencv_stereo.so.4.1.1
onboard: /usr/local/lib/libopencv_structured_light.so.4.1.1
onboard: /usr/local/lib/libopencv_reg.so.4.1.1
onboard: /usr/local/lib/libopencv_xfeatures2d.so.4.1.1
onboard: /usr/local/lib/libopencv_fuzzy.so.4.1.1
onboard: /usr/local/lib/libopencv_bioinspired.so.4.1.1
onboard: /usr/local/lib/libopencv_saliency.so.4.1.1
onboard: /usr/local/lib/libopencv_bgsegm.so.4.1.1
onboard: /usr/local/lib/libopencv_ccalib.so.4.1.1
onboard: /usr/local/lib/libopencv_dpm.so.4.1.1
onboard: /usr/local/lib/libopencv_aruco.so.4.1.1
onboard: /usr/local/lib/libopencv_videostab.so.4.1.1
onboard: /usr/local/lib/libopencv_superres.so.4.1.1
onboard: /usr/local/lib/libopencv_hfs.so.4.1.1
onboard: /usr/local/lib/libopencv_shape.so.4.1.1
onboard: /usr/local/lib/libopencv_quality.so.4.1.1
onboard: /usr/lib/libmavsdk_action.so
onboard: /usr/lib/libmavsdk_offboard.so
onboard: /usr/lib/libmavsdk_telemetry.so
onboard: /usr/lib/libmavsdk.so
onboard: /usr/local/lib/libopencv_tracking.so.4.1.1
onboard: /usr/local/lib/libopencv_plot.so.4.1.1
onboard: /usr/local/lib/libopencv_datasets.so.4.1.1
onboard: /usr/local/lib/libopencv_text.so.4.1.1
onboard: /usr/local/lib/libopencv_dnn.so.4.1.1
onboard: /usr/local/lib/libopencv_phase_unwrapping.so.4.1.1
onboard: /usr/local/lib/libopencv_objdetect.so.4.1.1
onboard: /usr/local/lib/libopencv_highgui.so.4.1.1
onboard: /usr/local/lib/libopencv_photo.so.4.1.1
onboard: /usr/local/lib/libopencv_videoio.so.4.1.1
onboard: /usr/local/lib/libopencv_optflow.so.4.1.1
onboard: /usr/local/lib/libopencv_video.so.4.1.1
onboard: /usr/local/lib/libopencv_ximgproc.so.4.1.1
onboard: /usr/local/lib/libopencv_imgcodecs.so.4.1.1
onboard: /usr/local/lib/libopencv_calib3d.so.4.1.1
onboard: /usr/local/lib/libopencv_features2d.so.4.1.1
onboard: /usr/local/lib/libopencv_flann.so.4.1.1
onboard: /usr/local/lib/libopencv_imgproc.so.4.1.1
onboard: /usr/local/lib/libopencv_ml.so.4.1.1
onboard: /usr/local/lib/libopencv_core.so.4.1.1
onboard: CMakeFiles/onboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tao/Desktop/onboard/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable onboard"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/onboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/onboard.dir/build: onboard

.PHONY : CMakeFiles/onboard.dir/build

CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/log_node.cpp.o.requires
CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/clock.cpp.o.requires
CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/socket_nodes.cpp.o.requires
CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/topics.cpp.o.requires
CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/camera_node.cpp.o.requires
CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/main.cpp.o.requires
CMakeFiles/onboard.dir/requires: CMakeFiles/onboard.dir/src/control_node.cpp.o.requires

.PHONY : CMakeFiles/onboard.dir/requires

CMakeFiles/onboard.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/onboard.dir/cmake_clean.cmake
.PHONY : CMakeFiles/onboard.dir/clean

CMakeFiles/onboard.dir/depend:
	cd /home/tao/Desktop/onboard/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tao/Desktop/onboard /home/tao/Desktop/onboard /home/tao/Desktop/onboard/build /home/tao/Desktop/onboard/build /home/tao/Desktop/onboard/build/CMakeFiles/onboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/onboard.dir/depend

