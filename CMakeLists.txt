cmake_minimum_required(VERSION 2.8.3)

IF(CMAKE_COMPILER_IS_GNUCXX)
   SET(CMAKE_CXX_FLAGS "-g3 -ggdb")
#SET(CMAKE_BUILD_TYPE DEBUG)
ENDIF()

project(uascode)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 mavros
)

find_package(Boost REQUIRED COMPONENTS 
 filesystem
 #shared_ptr
 #property_tree
 random
)

find_package(Armadillo REQUIRED)
find_package (GeographicLib 1.9 REQUIRED)
# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

add_message_files(
  FILES
  AccelXYZ.msg
  ColliPoint.msg
  GlobalPos.msg
  IfCollision.msg
  IfMavlinkGood.msg
  IfRecMsg.msg
  MultiObsMsg.msg
  ObsMsg.msg
  PlaneAttitude.msg
  PosSetPointFlag.msg
  PosSetPoint.msg
  WpCurrent.msg
  WpNumber.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES  #TBD
  CATKIN_DEPENDS roscpp mavros std_msgs
  DEPENDS Boost mavlink Armadillo GeographicLib 
)

include_directories(
  src
  src/adsb
  src/MavLink
  src/Planner
  src/common
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

#########other libs
add_library(otherlibs
   src/OtherLibs/Dubins2D/dubins.cpp
   )

####library for other utilities
add_library(common_utils
          src/common/Utils/GetTimeUTC.cpp
	  src/common/Utils/UTMtransform.cpp
	  src/common/Utils/GetTimeNow.cpp
	  src/common/Utils/systemtime.cpp
	  src/common/Utils/AdsbToObs.cpp
	  src/common/Utils/PortOperates.cpp
          src/common/Utils/GeoUtils.cpp
          src/common/Utils/MathUtils.cpp
          src/common/Utils/FindPath.cpp
          src/common/Utils/YcLogger.cpp
          src/common/Utils/AeroCoefficients.cpp
          src/common/UserTypes/MavlinkReceiver.cpp
	  src/common/UserTypes/MavlinkTCP.cpp
	  src/common/UserTypes/MavlinkSender.cpp
          src/common/UserTypes/ObsFromFile.cpp
          src/common/UserTypes/AdsbFromFile.cpp      
	  )
add_dependencies(common_utils
          uascode_generate_messages_cpp
	  otherlibs
)
target_link_libraries(common_utils
	  otherlibs
          ${catkin_LIBRARIES}
          ${Boost_LIBRARIES}
	  ${ARMADILLO_LIBRARIES}
	  ${GeographicLib_LIBRARIES}
	)

###library for adsb
add_library(adsb
	  src/adsb/UserTypes/ObsMonitor.cpp
	  src/adsb/UserTypes/SageDecoder.cpp
	  src/adsb/UserTypes/MsgSender.cpp
	  src/adsb/Utils/AdsbDecode.cpp
	  src/adsb/Utils/HeartDecode.cpp
	  src/adsb/Utils/StatusDecode.cpp
	  src/adsb/Utils/AdsbMsgToQGC.cpp
)
add_dependencies(adsb
        uascode_generate_messages_cpp
        common_utils
)
target_link_libraries(adsb
	common_utils
        ${catkin_LIBRARIES}
	${GeographicLib_LIBRARIES}
)

#########library for the path planner
add_library(planner
	src/Planner/UserTypes/L1ControlSim.cpp
	src/Planner/UserTypes/TECsSim.cpp
	src/Planner/UserTypes/StateUpdateSim.cpp
	src/Planner/UserTypes/NavigatorSim.cpp
	src/Planner/UserTypes/PathGenerator.cpp
        src/Planner/UserTypes/ReachabilitySet.cpp
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
add_dependencies(planner
        common_utils
)
target_link_libraries(planner
        common_utils
        ${catkin_LIBRARIES}
	${ARMADILLO_LIBRARIES}
	${GeographicLib_LIBRARIES}
)

##########library for nodes
add_library(nodes
	src/nodes/UserTypes/PlanNode.cpp
	src/nodes/UserTypes/PlanNode2.cpp
	src/nodes/UserTypes/MavlinkRecNode.cpp
	src/nodes/UserTypes/MavlinkSendNode.cpp
)
add_dependencies(nodes
        common_utils
	planner
)
target_link_libraries(nodes
        common_utils
	planner
        ${catkin_LIBRARIES}
	${ARMADILLO_LIBRARIES}
	${GeographicLib_LIBRARIES}
)

####build executables
add_executable(adsb_test src/adsb/UserTypes/adsb_test.cpp)
add_dependencies(adsb_test adsb)
target_link_libraries(adsb_test 
	adsb 
	common_utils 
	${catkin_LIBRARIES}
	${GeographicLib_LIBRARIES}
	)

#test obs from file
add_executable(test_obsfile src/common/UserTypes/test_obsfromfile.cpp)
add_dependencies(test_obsfile common_utils)
target_link_libraries(test_obsfile 
	common_utils 
        ${catkin_LIBRARIES}
	${GeographicLib_LIBRARIES}
)

#test adsb from file
add_executable(test_adsbfile src/common/UserTypes/test_AdsbFromFile.cpp)
add_dependencies(test_adsbfile common_utils)
target_link_libraries(test_adsbfile 
	common_utils 
        ${catkin_LIBRARIES}
	${GeographicLib_LIBRARIES}
)

#test plan_node
add_executable(test_PlanNode src/nodes/UserTypes/test_PlanNode.cpp)
add_dependencies(test_PlanNode nodes)
target_link_libraries(test_PlanNode 
	nodes 
	${catkin_LIBRARIES}
	${ARMADILLO_LIBRARIES} 
	${GeographicLib_LIBRARIES}
	)

#test PlanNode2
add_executable(test_PlanNode2 src/nodes/UserTypes/test_PlanNode2.cpp)
add_dependencies(test_PlanNode2 nodes)
target_link_libraries(test_PlanNode2 
	nodes 
	${catkin_LIBRARIES}
	${ARMADILLO_LIBRARIES} 
	${GeographicLib_LIBRARIES}
	)

#test MavlinkTCP_node
add_executable(test_MavRecNode src/nodes/UserTypes/test_MavRecNode.cpp)
add_dependencies(test_MavRecNode nodes)
target_link_libraries(test_MavRecNode 
	nodes 
	${catkin_LIBRARIES}
	${ARMADILLO_LIBRARIES} 
	${GeographicLib_LIBRARIES}
	) 

#test MavlinkSendNode
add_executable(test_MavSendNode src/nodes/UserTypes/test_MavSendNode.cpp)
add_dependencies(test_MavSendNode nodes)
target_link_libraries(test_MavSendNode 
	nodes 
	${catkin_LIBRARIES}
	${ARMADILLO_LIBRARIES} 
	${GeographicLib_LIBRARIES}
	)

#read the smallest distance
add_executable(read_dis src/common/Test/read_dis.cpp)
add_dependencies(read_dis common_utils)
target_link_libraries(read_dis
        common_utils
        ${catkin_LIBRARIES}
        ${GeographicLib_LIBRARIES}
)

#test navigator
add_executable(propagate_test src/Planner/UserTypes/test_navigator.cpp)
add_dependencies(propagate_test planner common_utils otherlibs)
target_link_libraries(propagate_test
              planner
              common_utils
              otherlibs
              ${ARMADILLO_LIBRARIES}
              ${GeographicLib_LIBRARIES}
)

#test Reachability set
add_executable(reachset_test src/Planner/UserTypes/test_ReachSet.cpp)
add_dependencies(reachset_test planner common_utils otherlibs)
target_link_libraries(reachset_test
              planner
              common_utils
              otherlibs
              ${ARMADILLO_LIBRARIES}
              ${GeographicLib_LIBRARIES}
)


#test PathGenerator
add_executable(pathgen_test src/Planner/UserTypes/test_pathgen.cpp)
add_dependencies(pathgen_test planner common_utils otherlibs)
target_link_libraries(pathgen_test
              planner
              common_utils
              otherlibs
              ${ARMADILLO_LIBRARIES}
              ${GeographicLib_LIBRARIES}
)
