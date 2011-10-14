cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()



# add library directories (-L)
LINK_DIRECTORIES(
    /home/pinaki/hack-arena/BRICS_3D/lib/
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     /home/pinaki/hack-arena/BRICS_3D/src
     /home/pinaki/hack-arena/BRICS_3D/external
)



rosbuild_add_executable(gopalSelectHSVLimits src/selectHSVLimits.cpp)
#rosbuild_add_executable(gopalSetTraingData src/setTraingData.cpp)

rosbuild_add_executable(gopalExtractROI src/extractROI.cpp)
rosbuild_add_executable(gopalExtractMultipleROI src/extractMultipleROI.cpp)

rosbuild_add_executable(gopalExtractObjectClusters src/extractObjectClusters.cpp)

#rosbuild_add_executable(gopalEstimatePoseVfh src/estimatePoseVfh.cpp)

rosbuild_add_executable(gopalEstimatePoseICP src/estimatePoseICP.cpp)
target_link_libraries(gopalEstimatePoseICP libbrics_3dpm_algorithm.a)
