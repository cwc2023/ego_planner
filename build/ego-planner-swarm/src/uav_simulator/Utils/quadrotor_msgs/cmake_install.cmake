# Install script for directory: /home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cwc/ego_planner/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/msg" TYPE FILE FILES
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/AuxCommand.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Corrections.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Gains.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/GoalSet.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/OutputData.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/PPROutputData.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Serial.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/SO3Command.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/StatusData.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/TRPYCommand.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Odometry.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/PolynomialTrajectory.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/OptimalTimeAllocator.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/TrajectoryMatrix.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/ReplanCheck.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/SpatialTemporalTrajectory.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Bspline.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/SwarmCommand.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Replan.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/SwarmOdometry.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/SwarmInfo.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/Px4ctrlDebug.msg"
    "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/msg/TakeoffLand.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/home/cwc/ego_planner/build/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cwc/ego_planner/devel/include/quadrotor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cwc/ego_planner/devel/share/roseus/ros/quadrotor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cwc/ego_planner/devel/share/common-lisp/ros/quadrotor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cwc/ego_planner/devel/share/gennodejs/ros/quadrotor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/cwc/ego_planner/devel/lib/python3/dist-packages/quadrotor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/cwc/ego_planner/devel/lib/python3/dist-packages/quadrotor_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cwc/ego_planner/build/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/home/cwc/ego_planner/build/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES
    "/home/cwc/ego_planner/build/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig.cmake"
    "/home/cwc/ego_planner/build/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs" TYPE FILE FILES "/home/cwc/ego_planner/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs/package.xml")
endif()

