#---1.配置工程
PROJECT("vio")
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
##SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo , Debug

#---2.可执行文件生成路径
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#

#子项目
#add_subdirectory(test/6_19)
#add_subdirectory(test/6_21)
add_subdirectory(test/6_22)

#---3.生成可执行文件
aux_source_directory(./main var) 
#message(STATUS ${var})
ADD_EXECUTABLE(${PROJECT_NAME} ${var})


