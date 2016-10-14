# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/aayush/Downloads/OpenCV/SnakeOF

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aayush/Downloads/OpenCV/SnakeOF

# Include any dependencies generated for this target.
include CMakeFiles/OF.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/OF.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OF.dir/flags.make

CMakeFiles/OF.dir/OF.cpp.o: CMakeFiles/OF.dir/flags.make
CMakeFiles/OF.dir/OF.cpp.o: OF.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/aayush/Downloads/OpenCV/SnakeOF/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/OF.dir/OF.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/OF.dir/OF.cpp.o -c /home/aayush/Downloads/OpenCV/SnakeOF/OF.cpp

CMakeFiles/OF.dir/OF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OF.dir/OF.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/aayush/Downloads/OpenCV/SnakeOF/OF.cpp > CMakeFiles/OF.dir/OF.cpp.i

CMakeFiles/OF.dir/OF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OF.dir/OF.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/aayush/Downloads/OpenCV/SnakeOF/OF.cpp -o CMakeFiles/OF.dir/OF.cpp.s

CMakeFiles/OF.dir/OF.cpp.o.requires:
.PHONY : CMakeFiles/OF.dir/OF.cpp.o.requires

CMakeFiles/OF.dir/OF.cpp.o.provides: CMakeFiles/OF.dir/OF.cpp.o.requires
	$(MAKE) -f CMakeFiles/OF.dir/build.make CMakeFiles/OF.dir/OF.cpp.o.provides.build
.PHONY : CMakeFiles/OF.dir/OF.cpp.o.provides

CMakeFiles/OF.dir/OF.cpp.o.provides.build: CMakeFiles/OF.dir/OF.cpp.o

# Object files for target OF
OF_OBJECTS = \
"CMakeFiles/OF.dir/OF.cpp.o"

# External object files for target OF
OF_EXTERNAL_OBJECTS =

OF: CMakeFiles/OF.dir/OF.cpp.o
OF: CMakeFiles/OF.dir/build.make
OF: /usr/local/lib/libopencv_videostab.so.3.0.0
OF: /usr/local/lib/libopencv_videoio.so.3.0.0
OF: /usr/local/lib/libopencv_video.so.3.0.0
OF: /usr/local/lib/libopencv_superres.so.3.0.0
OF: /usr/local/lib/libopencv_stitching.so.3.0.0
OF: /usr/local/lib/libopencv_shape.so.3.0.0
OF: /usr/local/lib/libopencv_photo.so.3.0.0
OF: /usr/local/lib/libopencv_objdetect.so.3.0.0
OF: /usr/local/lib/libopencv_ml.so.3.0.0
OF: /usr/local/lib/libopencv_imgproc.so.3.0.0
OF: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
OF: /usr/local/lib/libopencv_highgui.so.3.0.0
OF: /usr/local/lib/libopencv_hal.a
OF: /usr/local/lib/libopencv_flann.so.3.0.0
OF: /usr/local/lib/libopencv_features2d.so.3.0.0
OF: /usr/local/lib/libopencv_core.so.3.0.0
OF: /usr/local/lib/libopencv_calib3d.so.3.0.0
OF: /usr/local/lib/libopencv_features2d.so.3.0.0
OF: /usr/local/lib/libopencv_ml.so.3.0.0
OF: /usr/local/lib/libopencv_highgui.so.3.0.0
OF: /usr/local/lib/libopencv_videoio.so.3.0.0
OF: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
OF: /usr/local/lib/libopencv_flann.so.3.0.0
OF: /usr/local/lib/libopencv_video.so.3.0.0
OF: /usr/local/lib/libopencv_imgproc.so.3.0.0
OF: /usr/local/lib/libopencv_core.so.3.0.0
OF: /usr/local/lib/libopencv_hal.a
OF: /usr/lib/x86_64-linux-gnu/libGLU.so
OF: /usr/lib/x86_64-linux-gnu/libGL.so
OF: /usr/lib/x86_64-linux-gnu/libSM.so
OF: /usr/lib/x86_64-linux-gnu/libICE.so
OF: /usr/lib/x86_64-linux-gnu/libX11.so
OF: /usr/lib/x86_64-linux-gnu/libXext.so
OF: /usr/local/share/OpenCV/3rdparty/lib/libippicv.a
OF: CMakeFiles/OF.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable OF"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OF.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OF.dir/build: OF
.PHONY : CMakeFiles/OF.dir/build

CMakeFiles/OF.dir/requires: CMakeFiles/OF.dir/OF.cpp.o.requires
.PHONY : CMakeFiles/OF.dir/requires

CMakeFiles/OF.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OF.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OF.dir/clean

CMakeFiles/OF.dir/depend:
	cd /home/aayush/Downloads/OpenCV/SnakeOF && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aayush/Downloads/OpenCV/SnakeOF /home/aayush/Downloads/OpenCV/SnakeOF /home/aayush/Downloads/OpenCV/SnakeOF /home/aayush/Downloads/OpenCV/SnakeOF /home/aayush/Downloads/OpenCV/SnakeOF/CMakeFiles/OF.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OF.dir/depend

