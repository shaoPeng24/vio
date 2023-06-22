cmake_minimum_required(VERSION 3.0)
project(622demo)


#找opencv库
find_package( OpenCV  REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV library: ${OpenCV_INSTALL_PATH}")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
    if(${OpenCV_VERSION_MAJOR} GREATER 3)
        set(CMAKE_CXX_STANDARD 11)
    endif()
else()
    message(FATAL_ERROR "Error! OpenCV not found!")
endif()
find_package(Pangolin REQUIRED)

#包含头文件目录
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")


set(var "") #确var是空的。可能继承上级变量。
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR} )


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()


#输出可执行文件
aux_source_directory(. var) 
ADD_EXECUTABLE(${PROJECT_NAME} ${var})

#链接库文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})