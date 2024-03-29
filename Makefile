# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pinaki/hack-arena/ROS/extractObjects

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pinaki/hack-arena/ROS/extractObjects

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@echo "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@echo "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pinaki/hack-arena/ROS/extractObjects/CMakeFiles /home/pinaki/hack-arena/ROS/extractObjects/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pinaki/hack-arena/ROS/extractObjects/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named ROSBUILD_gencfg_cpp

# Build rule for target.
ROSBUILD_gencfg_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gencfg_cpp
.PHONY : ROSBUILD_gencfg_cpp

# fast build rule for target.
ROSBUILD_gencfg_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gencfg_cpp.dir/build.make CMakeFiles/ROSBUILD_gencfg_cpp.dir/build
.PHONY : ROSBUILD_gencfg_cpp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_cpp

# Build rule for target.
ROSBUILD_genmsg_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_cpp
.PHONY : ROSBUILD_genmsg_cpp

# fast build rule for target.
ROSBUILD_genmsg_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_cpp.dir/build.make CMakeFiles/ROSBUILD_genmsg_cpp.dir/build
.PHONY : ROSBUILD_genmsg_cpp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_lisp

# Build rule for target.
ROSBUILD_genmsg_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_lisp
.PHONY : ROSBUILD_genmsg_lisp

# fast build rule for target.
ROSBUILD_genmsg_lisp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_lisp.dir/build.make CMakeFiles/ROSBUILD_genmsg_lisp.dir/build
.PHONY : ROSBUILD_genmsg_lisp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_gensrv_cpp

# Build rule for target.
ROSBUILD_gensrv_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gensrv_cpp
.PHONY : ROSBUILD_gensrv_cpp

# fast build rule for target.
ROSBUILD_gensrv_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gensrv_cpp.dir/build.make CMakeFiles/ROSBUILD_gensrv_cpp.dir/build
.PHONY : ROSBUILD_gensrv_cpp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_gensrv_lisp

# Build rule for target.
ROSBUILD_gensrv_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gensrv_lisp
.PHONY : ROSBUILD_gensrv_lisp

# fast build rule for target.
ROSBUILD_gensrv_lisp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gensrv_lisp.dir/build.make CMakeFiles/ROSBUILD_gensrv_lisp.dir/build
.PHONY : ROSBUILD_gensrv_lisp/fast

#=============================================================================
# Target rules for targets named clean-test-results

# Build rule for target.
clean-test-results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 clean-test-results
.PHONY : clean-test-results

# fast build rule for target.
clean-test-results/fast:
	$(MAKE) -f CMakeFiles/clean-test-results.dir/build.make CMakeFiles/clean-test-results.dir/build
.PHONY : clean-test-results/fast

#=============================================================================
# Target rules for targets named gopalEstimatePoseICP

# Build rule for target.
gopalEstimatePoseICP: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gopalEstimatePoseICP
.PHONY : gopalEstimatePoseICP

# fast build rule for target.
gopalEstimatePoseICP/fast:
	$(MAKE) -f CMakeFiles/gopalEstimatePoseICP.dir/build.make CMakeFiles/gopalEstimatePoseICP.dir/build
.PHONY : gopalEstimatePoseICP/fast

#=============================================================================
# Target rules for targets named gopalExtractMultipleROI

# Build rule for target.
gopalExtractMultipleROI: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gopalExtractMultipleROI
.PHONY : gopalExtractMultipleROI

# fast build rule for target.
gopalExtractMultipleROI/fast:
	$(MAKE) -f CMakeFiles/gopalExtractMultipleROI.dir/build.make CMakeFiles/gopalExtractMultipleROI.dir/build
.PHONY : gopalExtractMultipleROI/fast

#=============================================================================
# Target rules for targets named gopalExtractObjectClusters

# Build rule for target.
gopalExtractObjectClusters: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gopalExtractObjectClusters
.PHONY : gopalExtractObjectClusters

# fast build rule for target.
gopalExtractObjectClusters/fast:
	$(MAKE) -f CMakeFiles/gopalExtractObjectClusters.dir/build.make CMakeFiles/gopalExtractObjectClusters.dir/build
.PHONY : gopalExtractObjectClusters/fast

#=============================================================================
# Target rules for targets named gopalExtractROI

# Build rule for target.
gopalExtractROI: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gopalExtractROI
.PHONY : gopalExtractROI

# fast build rule for target.
gopalExtractROI/fast:
	$(MAKE) -f CMakeFiles/gopalExtractROI.dir/build.make CMakeFiles/gopalExtractROI.dir/build
.PHONY : gopalExtractROI/fast

#=============================================================================
# Target rules for targets named gopalSelectHSVLimits

# Build rule for target.
gopalSelectHSVLimits: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gopalSelectHSVLimits
.PHONY : gopalSelectHSVLimits

# fast build rule for target.
gopalSelectHSVLimits/fast:
	$(MAKE) -f CMakeFiles/gopalSelectHSVLimits.dir/build.make CMakeFiles/gopalSelectHSVLimits.dir/build
.PHONY : gopalSelectHSVLimits/fast

#=============================================================================
# Target rules for targets named rosbuild_precompile

# Build rule for target.
rosbuild_precompile: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_precompile
.PHONY : rosbuild_precompile

# fast build rule for target.
rosbuild_precompile/fast:
	$(MAKE) -f CMakeFiles/rosbuild_precompile.dir/build.make CMakeFiles/rosbuild_precompile.dir/build
.PHONY : rosbuild_precompile/fast

#=============================================================================
# Target rules for targets named rosbuild_premsgsrvgen

# Build rule for target.
rosbuild_premsgsrvgen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_premsgsrvgen
.PHONY : rosbuild_premsgsrvgen

# fast build rule for target.
rosbuild_premsgsrvgen/fast:
	$(MAKE) -f CMakeFiles/rosbuild_premsgsrvgen.dir/build.make CMakeFiles/rosbuild_premsgsrvgen.dir/build
.PHONY : rosbuild_premsgsrvgen/fast

#=============================================================================
# Target rules for targets named rospack_gencfg

# Build rule for target.
rospack_gencfg: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_gencfg
.PHONY : rospack_gencfg

# fast build rule for target.
rospack_gencfg/fast:
	$(MAKE) -f CMakeFiles/rospack_gencfg.dir/build.make CMakeFiles/rospack_gencfg.dir/build
.PHONY : rospack_gencfg/fast

#=============================================================================
# Target rules for targets named rospack_gencfg_real

# Build rule for target.
rospack_gencfg_real: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_gencfg_real
.PHONY : rospack_gencfg_real

# fast build rule for target.
rospack_gencfg_real/fast:
	$(MAKE) -f CMakeFiles/rospack_gencfg_real.dir/build.make CMakeFiles/rospack_gencfg_real.dir/build
.PHONY : rospack_gencfg_real/fast

#=============================================================================
# Target rules for targets named rospack_genmsg

# Build rule for target.
rospack_genmsg: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg
.PHONY : rospack_genmsg

# fast build rule for target.
rospack_genmsg/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg.dir/build.make CMakeFiles/rospack_genmsg.dir/build
.PHONY : rospack_genmsg/fast

#=============================================================================
# Target rules for targets named rospack_genmsg_libexe

# Build rule for target.
rospack_genmsg_libexe: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg_libexe
.PHONY : rospack_genmsg_libexe

# fast build rule for target.
rospack_genmsg_libexe/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg_libexe.dir/build.make CMakeFiles/rospack_genmsg_libexe.dir/build
.PHONY : rospack_genmsg_libexe/fast

#=============================================================================
# Target rules for targets named rospack_gensrv

# Build rule for target.
rospack_gensrv: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_gensrv
.PHONY : rospack_gensrv

# fast build rule for target.
rospack_gensrv/fast:
	$(MAKE) -f CMakeFiles/rospack_gensrv.dir/build.make CMakeFiles/rospack_gensrv.dir/build
.PHONY : rospack_gensrv/fast

#=============================================================================
# Target rules for targets named test

# Build rule for target.
test: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test
.PHONY : test

# fast build rule for target.
test/fast:
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/build
.PHONY : test/fast

#=============================================================================
# Target rules for targets named test-future

# Build rule for target.
test-future: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-future
.PHONY : test-future

# fast build rule for target.
test-future/fast:
	$(MAKE) -f CMakeFiles/test-future.dir/build.make CMakeFiles/test-future.dir/build
.PHONY : test-future/fast

#=============================================================================
# Target rules for targets named test-results

# Build rule for target.
test-results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-results
.PHONY : test-results

# fast build rule for target.
test-results/fast:
	$(MAKE) -f CMakeFiles/test-results.dir/build.make CMakeFiles/test-results.dir/build
.PHONY : test-results/fast

#=============================================================================
# Target rules for targets named test-results-run

# Build rule for target.
test-results-run: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-results-run
.PHONY : test-results-run

# fast build rule for target.
test-results-run/fast:
	$(MAKE) -f CMakeFiles/test-results-run.dir/build.make CMakeFiles/test-results-run.dir/build
.PHONY : test-results-run/fast

#=============================================================================
# Target rules for targets named tests

# Build rule for target.
tests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tests
.PHONY : tests

# fast build rule for target.
tests/fast:
	$(MAKE) -f CMakeFiles/tests.dir/build.make CMakeFiles/tests.dir/build
.PHONY : tests/fast

# target to build an object file
src/estimatePoseICP.o:
	$(MAKE) -f CMakeFiles/gopalEstimatePoseICP.dir/build.make CMakeFiles/gopalEstimatePoseICP.dir/src/estimatePoseICP.o
.PHONY : src/estimatePoseICP.o

# target to preprocess a source file
src/estimatePoseICP.i:
	$(MAKE) -f CMakeFiles/gopalEstimatePoseICP.dir/build.make CMakeFiles/gopalEstimatePoseICP.dir/src/estimatePoseICP.i
.PHONY : src/estimatePoseICP.i

# target to generate assembly for a file
src/estimatePoseICP.s:
	$(MAKE) -f CMakeFiles/gopalEstimatePoseICP.dir/build.make CMakeFiles/gopalEstimatePoseICP.dir/src/estimatePoseICP.s
.PHONY : src/estimatePoseICP.s

# target to build an object file
src/extractMultipleROI.o:
	$(MAKE) -f CMakeFiles/gopalExtractMultipleROI.dir/build.make CMakeFiles/gopalExtractMultipleROI.dir/src/extractMultipleROI.o
.PHONY : src/extractMultipleROI.o

# target to preprocess a source file
src/extractMultipleROI.i:
	$(MAKE) -f CMakeFiles/gopalExtractMultipleROI.dir/build.make CMakeFiles/gopalExtractMultipleROI.dir/src/extractMultipleROI.i
.PHONY : src/extractMultipleROI.i

# target to generate assembly for a file
src/extractMultipleROI.s:
	$(MAKE) -f CMakeFiles/gopalExtractMultipleROI.dir/build.make CMakeFiles/gopalExtractMultipleROI.dir/src/extractMultipleROI.s
.PHONY : src/extractMultipleROI.s

# target to build an object file
src/extractObjectClusters.o:
	$(MAKE) -f CMakeFiles/gopalExtractObjectClusters.dir/build.make CMakeFiles/gopalExtractObjectClusters.dir/src/extractObjectClusters.o
.PHONY : src/extractObjectClusters.o

# target to preprocess a source file
src/extractObjectClusters.i:
	$(MAKE) -f CMakeFiles/gopalExtractObjectClusters.dir/build.make CMakeFiles/gopalExtractObjectClusters.dir/src/extractObjectClusters.i
.PHONY : src/extractObjectClusters.i

# target to generate assembly for a file
src/extractObjectClusters.s:
	$(MAKE) -f CMakeFiles/gopalExtractObjectClusters.dir/build.make CMakeFiles/gopalExtractObjectClusters.dir/src/extractObjectClusters.s
.PHONY : src/extractObjectClusters.s

# target to build an object file
src/extractROI.o:
	$(MAKE) -f CMakeFiles/gopalExtractROI.dir/build.make CMakeFiles/gopalExtractROI.dir/src/extractROI.o
.PHONY : src/extractROI.o

# target to preprocess a source file
src/extractROI.i:
	$(MAKE) -f CMakeFiles/gopalExtractROI.dir/build.make CMakeFiles/gopalExtractROI.dir/src/extractROI.i
.PHONY : src/extractROI.i

# target to generate assembly for a file
src/extractROI.s:
	$(MAKE) -f CMakeFiles/gopalExtractROI.dir/build.make CMakeFiles/gopalExtractROI.dir/src/extractROI.s
.PHONY : src/extractROI.s

# target to build an object file
src/selectHSVLimits.o:
	$(MAKE) -f CMakeFiles/gopalSelectHSVLimits.dir/build.make CMakeFiles/gopalSelectHSVLimits.dir/src/selectHSVLimits.o
.PHONY : src/selectHSVLimits.o

# target to preprocess a source file
src/selectHSVLimits.i:
	$(MAKE) -f CMakeFiles/gopalSelectHSVLimits.dir/build.make CMakeFiles/gopalSelectHSVLimits.dir/src/selectHSVLimits.i
.PHONY : src/selectHSVLimits.i

# target to generate assembly for a file
src/selectHSVLimits.s:
	$(MAKE) -f CMakeFiles/gopalSelectHSVLimits.dir/build.make CMakeFiles/gopalSelectHSVLimits.dir/src/selectHSVLimits.s
.PHONY : src/selectHSVLimits.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... ROSBUILD_gencfg_cpp"
	@echo "... ROSBUILD_genmsg_cpp"
	@echo "... ROSBUILD_genmsg_lisp"
	@echo "... ROSBUILD_gensrv_cpp"
	@echo "... ROSBUILD_gensrv_lisp"
	@echo "... clean-test-results"
	@echo "... edit_cache"
	@echo "... gopalEstimatePoseICP"
	@echo "... gopalExtractMultipleROI"
	@echo "... gopalExtractObjectClusters"
	@echo "... gopalExtractROI"
	@echo "... gopalSelectHSVLimits"
	@echo "... rebuild_cache"
	@echo "... rosbuild_precompile"
	@echo "... rosbuild_premsgsrvgen"
	@echo "... rospack_gencfg"
	@echo "... rospack_gencfg_real"
	@echo "... rospack_genmsg"
	@echo "... rospack_genmsg_libexe"
	@echo "... rospack_gensrv"
	@echo "... test"
	@echo "... test-future"
	@echo "... test-results"
	@echo "... test-results-run"
	@echo "... tests"
	@echo "... src/estimatePoseICP.o"
	@echo "... src/estimatePoseICP.i"
	@echo "... src/estimatePoseICP.s"
	@echo "... src/extractMultipleROI.o"
	@echo "... src/extractMultipleROI.i"
	@echo "... src/extractMultipleROI.s"
	@echo "... src/extractObjectClusters.o"
	@echo "... src/extractObjectClusters.i"
	@echo "... src/extractObjectClusters.s"
	@echo "... src/extractROI.o"
	@echo "... src/extractROI.i"
	@echo "... src/extractROI.s"
	@echo "... src/selectHSVLimits.o"
	@echo "... src/selectHSVLimits.i"
	@echo "... src/selectHSVLimits.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

