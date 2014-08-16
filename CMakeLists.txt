cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#for debug
IF(CMAKE_COMPILER_IS_GNUCXX)
   SET(CMAKE_CXX_FLAGS "-g3 -ggdb -DTIXML_USE_TICPP") 
#SET(CMAKE_BUILD_TYPE DEBUG)
ENDIF()
#put executables in bin directory
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin/")
###########################3rd party library##########################
#for armadillo lib
if(EXISTS /usr/local/lib64/libarmadillo.so)
    set( ARMADILLO_LIBRARIES /usr/local/lib64/libarmadillo.so )
else()
    find_package(Armadillo REQUIRED)
endif()
#for geographic lib
find_package(GeographicLib 1.9 REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
rosbuild_add_boost_directories()
#for mavlink msg, will deal with it later
#set(MAVLINK_DIRECTORY "mavlink/include/mavlink/v1.0")
#include_directories(${MAVLINK_DIRECTORY})

##############################codes part##############################
include_directories(src) #for general
include_directories(src/adsb) #for adsb
include_directories(src/MavLink) #for mavlink
include_directories(src/Planner) #for Planner
include_directories(src/common)#for common use
##############################ads-b part################################
###library for adsb
rosbuild_add_library(adsb
src/adsb/UserTypes/ObsMonitor.cpp
src/adsb/UserTypes/SageDecoder.cpp
src/adsb/UserTypes/MsgSender.cpp
src/adsb/Utils/AdsbDecode.cpp
src/adsb/Utils/HeartDecode.cpp
src/adsb/Utils/StatusDecode.cpp
src/adsb/Utils/AdsbMsgToQGC.cpp
)

####library for other utilities
rosbuild_add_library(common_utils
          src/common/Utils/GetTimeUTC.cpp
	  src/common/Utils/UTMtransform.cpp
	  src/common/Utils/GetTimeNow.cpp
	  src/common/Utils/systemtime.cpp
	  src/common/Utils/AdsbToObs.cpp
	  src/common/Utils/PortOperates.cpp
          src/common/Utils/GeoUtils.cpp
          src/common/Utils/MathUtils.cpp
          src/common/Utils/YcLogger.cpp
          src/common/UserTypes/MavlinkReceiver.cpp
	  src/common/UserTypes/MavlinkTCP.cpp
	  src/common/UserTypes/MavlinkSender.cpp
          src/common/UserTypes/ObsFromFile.cpp
  )

#########library for the path planner
rosbuild_add_library(planner
 src/Planner/UserTypes/L1ControlSim.cpp
 src/Planner/UserTypes/TECsSim.cpp
 src/Planner/UserTypes/StateUpdateSim.cpp
 src/Planner/UserTypes/NavigatorSim.cpp
 src/Planner/UserTypes/PathGenerator.cpp
 src/Planner/UserTypes/Sampler/Sampler.cpp
 src/Planner/UserTypes/Sampler/SamplerPole.cpp
 src/Planner/UserTypes/Sampler/SamplerRect.cpp
 src/Planner/Utils/CheckCollision.cpp
 src/Planner/Utils/PointInPoly.cpp
 src/Planner/Utils/NotInRadius.cpp
 src/Planner/Utils/DubinsLength3D.cpp
 src/Planner/Utils/DubinsLength2.cpp
 src/Planner/Utils/DubinsLength3D.cpp
 src/Planner/UserStructs/SpaceLimit.cpp
 src/Planner/UserStructs/PlaneStateSim.cpp
 src/Planner/UserStructs/MissionSimFlagPt.cpp
 src/Planner/UserStructs/PredictColliReturn.cpp
 )

##########library for nodes
rosbuild_add_library(nodes
src/nodes/UserTypes/PlanNode.cpp
src/nodes/UserTypes/PlanNode2.cpp
src/nodes/UserTypes/MavlinkRecNode.cpp
src/nodes/UserTypes/MavlinkSendNode.cpp
)

 #########other libs
rosbuild_add_library(otherlibs
   src/OtherLibs/Dubins2D/dubins.cpp
   )

 ###################executables#################
#test adsb 
rosbuild_add_executable(adsb_test src/adsb/UserTypes/adsb_test.cpp)
target_link_libraries(adsb_test adsb common_utils ${GeographicLib_LIBRARIES}  )
#test dubin length
rosbuild_add_executable(test_dubin_length src/Planner/Utils/test_dubin_length.cpp)
target_link_libraries(test_dubin_length planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES} )  
#test l1_control
rosbuild_add_executable(l1_test src/Planner/UserTypes/test_l1.cpp)
target_link_libraries(l1_test planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES} ) 
#test TECS
rosbuild_add_executable(tecs_test src/Planner/UserTypes/test_tecs.cpp)
target_link_libraries(tecs_test planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})
#test updater
rosbuild_add_executable(update_test src/Planner/UserTypes/test_update.cpp)
target_link_libraries(update_test planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})
#test navigator
rosbuild_add_executable(propagate_test src/Planner/UserTypes/test_navigator.cpp)
target_link_libraries(propagate_test planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})
#test path generator
rosbuild_add_executable(path_test src/Planner/UserTypes/test_pathgen.cpp)
target_link_libraries(path_test planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})
#test mavlink receiver
rosbuild_add_executable(receiver_test src/common/UserTypes/test_receiver.cpp)
target_link_libraries(receiver_test common_utils ${GeographicLib_LIBRARIES})
#test mavlink tcp
rosbuild_add_executable(mavlinktcp_test src/common/UserTypes/test_tcpmavlink.cpp)
target_link_libraries(mavlinktcp_test common_utils ${GeographicLib_LIBRARIES})  
#test mavlink sender tcp
rosbuild_add_executable(mavlinksend_test src/common/UserTypes/mavlinksend_test.cpp)
target_link_libraries(mavlinksend_test common_utils ${GeographicLib_LIBRARIES}) 
#test mavlinktcp_log
rosbuild_add_executable(mavlinktcp_log src/common/UserTypes/mavlinktcp_log.cpp)
target_link_libraries(mavlinktcp_log common_utils ${GeographicLib_LIBRARIES}) 
#test in poly
#rosbuild_add_executable(polyin_test src/Planner/Utils/test_pointinpoly.cpp)
#target_link_libraries(polyin_test planner)

#test obs from file
rosbuild_add_executable(test_obsfile src/common/UserTypes/test_obsfromfile.cpp)
target_link_libraries(test_obsfile common_utils ${GeographicLib_LIBRARIES})

#test plan_node
rosbuild_add_executable(test_PlanNode src/nodes/UserTypes/test_PlanNode.cpp)
target_link_libraries(test_PlanNode nodes planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})

#test PlanNode2
rosbuild_add_executable(test_PlanNode2 src/nodes/UserTypes/test_PlanNode2.cpp)
target_link_libraries(test_PlanNode2 nodes planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})

#test MavlinkTCP_node
rosbuild_add_executable(test_MavRecNode src/nodes/UserTypes/test_MavRecNode.cpp)
target_link_libraries(test_MavRecNode nodes planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES}) 

#test MavlinkSendNode
rosbuild_add_executable(test_MavSendNode src/nodes/UserTypes/test_MavSendNode.cpp)
target_link_libraries(test_MavSendNode nodes planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})

#test predict in time limit
rosbuild_add_executable(test_LoopPredict src/nodes/UserTypes/test_LoopPredict.cpp)
target_link_libraries(test_LoopPredict nodes planner common_utils otherlibs ${ARMADILLO_LIBRARIES} ${GeographicLib_LIBRARIES})

#other test
rosbuild_add_executable(other_test src/common/Test/main_test.cpp)
rosbuild_add_library(gettimenow src/common/Utils/GetTimeNow.cpp)
target_link_libraries(other_test gettimenow) 
