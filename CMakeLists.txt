
cmake_minimum_required (VERSION 2.8.11)
project (planner)

file(GLOB planner_SRC
        src/compiler/*.cpp
        src/runtime/*.cpp
        include/compiler/*.h
        include/runtime/*.h
        examples/travel.cpp
        examples/travel.h
)

include_directories(include)

add_executable(hello_planner examples/hello_planner.cpp)
add_executable(travel_planner examples/travel_main.cpp  ${planner_SRC})


