cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

link_directories(/usr/lib/x86_64-linux-gnu)
include_directories(/usr/include/gstreamer-0.10 /usr/include/glib-2.0 /usr/lib/glib-2.0/include /usr/lib/i386-linux-gnu/glib-2.0/include /usr/lib/x86_64-linux-gnu/glib-2.0/include /usr/include/libxml2)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(record_rgbd_service src/record_rgbd_video.cpp)
target_link_libraries(record_rgbd_service ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} boost_program_options)

rosbuild_add_executable(record_rgb_service src/record_rgb_video.cpp)
target_link_libraries(record_rgb_service gstreamer-0.10 gobject-2.0 gmodule-2.0 gthread-2.0 rt xml2 glib-2.0 gstapp-0.10 pthread ${OpenCV_LIBRARIES})

