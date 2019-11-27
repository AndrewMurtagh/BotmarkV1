# *****************************************************************
#
#  This file is part of the Botmark benchmark.
#
#  Copyright (c) 2019 Andrew Murtagh, Patrick Lynch,
#  and Conor McGinn.
#
#  This work is licensed under the "Creative Commons
#  (Attribution-NonCommercial-ShareAlike 4.0 International)
#  License" and is copyrighted by Andrew Murtagh, Patrick Lynch,
#  and Conor McGinn.
#
#  To view a copy of this license, visit
#  http://creativecommons.org/licenses/by-nc-sa/4.0/ or
#  send a letter to Creative Commons, PO Box 1866,
#  Mountain View, CA 94042, USA.
#
#  Botmark is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied
#  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
# *****************************************************************/


cmake_minimum_required(VERSION 3.2)

project(Botmark)

set(Botmark_VERSION_MAJOR "1")
set(Botmark_VERSION_MINOR "0")
set(Botmark_VERSION_PATCH "0")
set(ROOT_DIRECTORY "${CMAKE_SOURCE_DIR}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )

add_definitions("-Wall")

configure_file (
  "${PROJECT_SOURCE_DIR}/res/BotmarkConfig.h.in"
  "${PROJECT_SOURCE_DIR}/src/BotmarkConfig.h"
)



find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Festival REQUIRED)
find_package(EST REQUIRED)
find_package(Sphinxbase REQUIRED)
find_package(Pocketsphinx REQUIRED)
find_package(Caffe REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(MRPT COMPONENTS gui nav REQUIRED)
find_package(Gmapping REQUIRED)
find_package(OpenRAVE REQUIRED)
find_package(ALSA REQUIRED)

add_definitions(${Caffe_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



set(build_persontracking TRUE)
set(build_speechsynthesis TRUE)
set(build_voicerecognition TRUE)
set(build_slam TRUE)
set(build_objectrecognition TRUE)
set(build_pathplanning TRUE)
set(build_motionplanning TRUE)
set(build_facialrecognition TRUE)
set(build_tools TRUE)




add_subdirectory(src)
