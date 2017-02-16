cmake_minimum_required(VERSION 2.8.3)
project(RA3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(matrixnode src/matrixnode.cpp)
add_executable(subscriber src/subscriber.cpp)

target_link_libraries(matrixnode ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

add_dependencies(matrixnode RA3_generate_messages_cpp)

target_link_libraries(subscriber ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})

add_dependencies(matrixnode RA3_generate_messages_cpp)
