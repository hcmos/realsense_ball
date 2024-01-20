CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
project(realsense_ball CXX)

message("project name is automaically set as: ${PROJECT_NAME}")

set(PROJECT_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)


set(SRC_FILES
  src/main.cpp
  #src/samplers.cpp
)

set(HEADER_FILES
)

add_executable(main ${SRC_FILES})

target_include_directories(main PUBLIC
  include/rs
  ${BOOST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

target_link_libraries(main PUBLIC
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)
