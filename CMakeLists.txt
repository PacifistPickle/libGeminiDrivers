cmake_minimum_required(VERSION 3.5)

project(libGeminiDrivers LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 11)

add_subdirectory(libHumidity)
add_subdirectory(libInterfaces)
