# CMakeLists.txt to build project using cmake
# run $cmake CMakeLists.txt
# Author : François Gavrel

cmake_minimum_required(VERSION 2.6)

project(kinect-exemple)

include_directories("/home/francois/kinect/NiTE-Linux-x64-2.2/Include")
include_directories("/home/francois/kinect/OpenNI-Linux-x64-2.2/Include")

# OPENNI2_INCLUDE:FILEPATH=/home/francois/kinect/NiTE-Linux-x64-2.2/Include
# NITE2_INCLUDE:FILEPATH="/home/francois/kinect/OpenNI-Linux-x64-2.2/Include"
# 
# OPENNI2_REDIST:FILEPATH="/home/francois/kinect/NiTE-Linux-x64-2.2/Redist"
# NITE2_REDIST64:FILEPATH="/home/francois/kinect/OpenNI-Linux-x64-2.2/Redist"

# include_directories($(OPENNI2_INCLUDE))
# include_directories($(NITE2_INCLUDE))

link_directories("/home/francois/kinect/NiTE-Linux-x64-2.2/Redist")
link_directories("/home/francois/kinect/OpenNI-Linux-x64-2.2/Redist")

# link_directories($(OPENNI2_REDIST))
# link_directories($(NITE2_REDIST64))


#set( SOURCE_FILES
#src/main.cpp
#)

file(
GLOB_RECURSE
source_files
src/*.[cht]pp
src/*.[cht]
#src/CommonDefs.mak
#src/CommonCppMakefile
)

add_executable(kinect-exemple ${source_files})

set(CMAKE_BUILD_TYPE Debug)
add_definitions( -O2 -Wall -Wextra -std=c++11 -Wno-comment -Wno-unused-parameter -Wno-unused-function-g)

target_link_libraries(kinect-exemple OpenNI2 NiTE2)
