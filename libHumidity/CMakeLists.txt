cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME_LOCAL Humidity)

#Find source files
file(GLOB_RECURSE SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

#Target
add_library(${PROJECT_NAME_LOCAL} STATIC ${SRC})

#Include header files
target_include_directories(${PROJECT_NAME_LOCAL} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

#Dependencies
target_link_libraries(${PROJECT_NAME_LOCAL} Interfaces)
