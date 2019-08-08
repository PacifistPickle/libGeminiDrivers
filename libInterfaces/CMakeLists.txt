cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME_LOCAL Interfaces)

# Find source files
# file(GLOB_RECURSE SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

# Target
add_library(${PROJECT_NAME_LOCAL} STATIC ${SRC})

# Need to tell linker to use CXX, since no source files
set_target_properties(${PROJECT_NAME_LOCAL} PROPERTIES LINKER_LANGUAGE CXX)

# Include header files
target_include_directories(${PROJECT_NAME_LOCAL} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

# Dependencies - DO NOT ADD DEPENDENCIES FOR THIS
#target_link_libraries(${PROJECT_NAME_LOCAL} addHereIfNeeded)
