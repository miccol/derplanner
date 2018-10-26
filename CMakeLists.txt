
cmake_minimum_required (VERSION 2.8.11)
project (planner)

file(GLOB planner_SRC
        src/compiler/*.cpp
        src/runtime/*.cpp
        include/compiler/*.h
        include/runtime/*.h
        examples/travel.cpp
        examples/travel.h
        examples/grasp.cpp
        examples/grasp.h

)

include_directories(include)

add_executable(compiler app/main.compiler.cpp ${planner_SRC})

add_executable(travel_planner examples/travel_main.cpp  ${planner_SRC})
add_executable(grasp_planner examples/grasp_main.cpp  ${planner_SRC})


