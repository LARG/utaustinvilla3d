cmake_minimum_required(VERSION 2.8)
project(UTAustinVillaBase CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)

find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Rcssnet3d REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set (CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused")
endif(NOT CMAKE_CXX_FLAGS)

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-new-dtags")

set(CORE_PATH utwalk)

include_directories(
${CORE_PATH}
${RCSSNET3D_INCLUDE_DIR}
)

set(CORE_FILES
  ${CORE_PATH}/MotionCore.cpp
  ${CORE_PATH}/Module.cpp  
  ${CORE_PATH}/common/NMatrix.cpp 
  ${CORE_PATH}/math/MVTools.cpp 
  ${CORE_PATH}/math/RotationMatrix.cpp 
  ${CORE_PATH}/memory/Lock.cpp 
  ${CORE_PATH}/memory/Logger.cpp   
  ${CORE_PATH}/memory/Memory.cpp  
  ${CORE_PATH}/memory/MemoryBlock.cpp  
  ${CORE_PATH}/memory/PrivateMemory.cpp  
  ${CORE_PATH}/memory/SharedMemory.cpp 
  ${CORE_PATH}/motion/MotionModule.cpp
  ${CORE_PATH}/motion/UTWalkEngine.cpp 
  ${CORE_PATH}/sensor/SensorModule.cpp  
  ${CORE_PATH}/sensor/InertialFilter.cpp 
  ${CORE_PATH}/kinematics/InverseKinematics.cpp 
  ${CORE_PATH}/kinematics/ForwardKinematics.cpp  
  ${CORE_PATH}/kinematics/KinematicsModule.cpp  
  ${CORE_PATH}/math/Geometry.cpp  
  ${CORE_PATH}/common/PIDController.cpp 
  ${CORE_PATH}/motion/WalkEngineParameters.cpp 
)

set(SRCS
  main.cc 
  behaviors/behavior.cc
  behaviors/naobehavior.cc
  behaviors/checkfall.cc 
  behaviors/kicking.cc 
  behaviors/strategy.cc
  behaviors/pkbehaviors.cc
  behaviors/gazebobehavior.cc
  behaviors/simplesoccer.cc
  servercomm/primitives.cc
  parser/parser.cc 
  math/hctmatrix.cc
  math/vecposition.cc
  math/Geometry.cc
  worldmodel/worldmodel.cc
  bodymodel/bodymodel.cc
  particlefilter/PFLocalization.cc
  particlefilter/Particle.cc
  skills/skill.cc 
  skills/curve3d.cc
  ikfast/ikfast.cpp
  headers/headers.cc
  audio/audio.cc 
  rvdraw/rvdraw.cc 
  ${CORE_FILES}
  kalman/BallKF.cpp
  kalman/PlayerKF.cpp 
  kalman/OrigKalmanFilter.cpp
  optimization/optimizationbehaviors.cc
  stats/recordstatsbehavior.cc
)

set(BINARY agentspark)

add_executable(${BINARY}
${SRCS}
)

target_link_libraries(${BINARY}
${RCSSNET3D_LIBRARY}
${Boost_SYSTEM_LIBRARY}
dl
${CMAKE_THREAD_LIBS_INIT}
rt 
)

# Automatically copy linked shared objects to libs directory after evey build
#add_custom_command(TARGET ${BINARY}
#  POST_BUILD
#  COMMAND ${CMAKE_SOURCE_DIR}/scripts/copy_libs.sh ARGS ${BINARY} libs
#)
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES libs)