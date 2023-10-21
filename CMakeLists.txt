cmake_minimum_required(VERSION 3.0.2)
project(copMatlab)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)


# Coppliasim
set(COPPELIASIM_INCLUDE_DIR "/home/dknt/Software/coppliasim/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04/programming/include")
set(COPPELIASIM_ROOT_DIR "/home/dknt/Software/coppliasim/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(CoppeliaSim REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES copMatlab
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  "include/copMatlab"
  ${COPPELIASIM_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# copCar
coppeliasim_add_executable(copTest
    SOURCES
    src/copTest.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/shared_memory.c
    src/extApi.c
    src/extApiPlatform.c
)
target_link_libraries(copTest
    Boost::system
    Threads::Threads
    rt
)
