cmake_minimum_required(VERSION 2.8)
project(UTAustinVillaBase CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)

find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Rcssnet3d REQUIRED)

set(CORE_PATH core_utwalk)

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
  behavior.cc
  naobehavior.cc
  checkfall.cc 
  primitives.cc
  strategy.cc
  parser.cc 
  hctmatrix.cc
  vecposition.cc
  worldmodel.cc
  bodymodel.cc
  PFLocalization.cc
  Particle.cc
  Geometry.cc
  skill.cc 
  headers.cc
  audio.cc 
  pkbehaviors.cc
  ikfast/ikfast.cpp
  curve3d.cc
  rvdraw.cc 
  ${CORE_FILES}
  BallKF.cpp
  PlayerKF.cpp 
  OrigKalmanFilter.cpp
  kicking.cc 
  optimizationbehaviors.cc
)

add_executable(agentspark 
${SRCS}
)

target_link_libraries(agentspark
${RCSSNET3D_LIBRARY}
${Boost_SYSTEM_LIBRARY}
dl
${CMAKE_THREAD_LIBS_INIT}
rt 
)

