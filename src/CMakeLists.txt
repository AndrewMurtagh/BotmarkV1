# *****************************************************************
# *
# * This file is part of the Botmark benchmark.
# *
# * Copyright (c) 2019 Andrew Murtagh, Patrick Lynch,
# * and Conor McGinn.
# *
# * This work is licensed under the "Creative Commons
# * (Attribution-NonCommercial-ShareAlike 4.0 International)
# * License" and is copyrighted by Andrew Murtagh, Patrich Lynch,
# * and Conor McGinn.
# *
# * To view a copy of this license, visit
# * http://creativecommons.org/licenses/by-nc-sa/4.0/ or
# * send a letter to Creative Commons, PO Box 1866,
# * Mountain View, CA 94042, USA.
# *
# * Botmark is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied
# * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
# * PURPOSE.
# *
# *****************************************************************/



########################################################
#tools
########################################################
if(build_tools)
add_executable(tool_trainFisherface trainFisherface.cpp)

target_include_directories(tool_trainFisherface PUBLIC
	${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tool_trainFisherface PUBLIC
	${OpenCV_LIBS}
)
endif()


########################################################
#person tracking
########################################################
if(build_persontracking)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PersonTracking persontracking.cpp)

target_include_directories(PersonTracking PUBLIC
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	)

target_link_libraries(PersonTracking PUBLIC
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
)
endif()


########################################################
#speech synthesis
########################################################
if(build_speechsynthesis)
find_package(Festival REQUIRED)
find_package(EST REQUIRED)
set (CMAKE_CXX_FLAGS "-fopenmp")
add_executable(Speechsynthesis speechsynthesis.cpp)
target_include_directories(Speechsynthesis PUBLIC ${EST_INCLUDE_DIR} ${Festival_INCLUDE_DIR})
target_link_libraries(Speechsynthesis PUBLIC ${Festival_LIBRARIES} ${EST_LIBRARIES})

endif()




########################################################
#voice recognition
########################################################
if(build_voicerecognition)
add_executable(VoiceRecognition voicerecognition.cpp)
target_include_directories(VoiceRecognition PUBLIC
	${Sphinxbase_INCLUDE_DIR}
	${Pocketsphinx_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${ALSA_INCLUDE_DIRS}
)

target_link_libraries(VoiceRecognition PUBLIC
	${Boost_SYSTEM_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
	#${Boost_LIBRARIES}
	${Sphinxbase_LIBRARIES}
	${Pocketsphinx_LIBRARIES}
	${ALSA_LIBRARIES}
)
endif()



########################################################
#slam
########################################################
if(build_slam)
find_package(Gmapping REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(Slam slam.cpp)
target_include_directories(Slam PUBLIC
	${OpenCV_INCLUDE_DIRS}
	${Gmapping_INCLUDE_DIR}
)
target_link_libraries(Slam PUBLIC
	${Gmapping_LIBRARIES}
	${OpenCV_LIBS}
)
endif()




########################################################
#object recognition
########################################################
if(build_objectrecognition)
add_executable(ObjectRecognition objectrecognition.cpp)

target_include_directories(ObjectRecognition PUBLIC
	${Caffe_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(ObjectRecognition PUBLIC
	${Caffe_LIBRARIES}
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
)
endif()


########################################################
# path planning
########################################################
if(build_pathplanning)
add_executable(PathPlanning pathplanning.cpp)
target_include_directories(PathPlanning PUBLIC ${MRPT_INCLUDE_DIR})

target_link_libraries(PathPlanning PUBLIC ${MRPT_LIBS})

endif()

########################################################
# motion planning
########################################################
if(build_motionplanning)
add_executable(MotionPlanning motionplanning.cpp)
target_include_directories(MotionPlanning PUBLIC
	${OpenRAVE_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)
target_link_libraries(MotionPlanning PUBLIC
	${OpenRAVE_LIBRARIES}
	${OpenRAVE_CORE_LIBRARIES}
	${Boost_LIBRARIES}
	${Boost_THREAD_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	##${Boost_LIBRARIES}
)

endif()



########################################################
# facial recognition
########################################################
if(build_facialrecognition)
add_executable(FacialRecognition facialrecognition.cpp)
target_include_directories(FacialRecognition PUBLIC
	${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(FacialRecognition PUBLIC
	${OpenCV_LIBS}
)

endif()
