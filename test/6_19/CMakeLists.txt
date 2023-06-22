cmake_minimum_required(VERSION 3.0)
project(6_19demo)

include_directories("/usr/include/eigen3")

set(var "") #确var是空的。可能继承上级变量。
message(STATUS ${var})

aux_source_directory(. var) 

ADD_EXECUTABLE(${PROJECT_NAME} ${var})