cmake_minimum_required(VERSION 3.16)
project(location_triggered_wrench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(LocationTriggeredWrench SHARED
  src/LocationTriggeredWrench.cc
)

target_link_libraries(LocationTriggeredWrench PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-math7::gz-math7
)

install(TARGETS LocationTriggeredWrench
  LIBRARY DESTINATION lib
)
