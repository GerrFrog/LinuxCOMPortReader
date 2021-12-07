cmake_minimum_required(VERSION 3.1.0)

project(COMReader VERSION 1.0.0 LANGUAGES CXX)

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

add_executable(
    ${PROJECT_NAME}
    core/src/main.cpp
    core/inc/main.hpp
)






