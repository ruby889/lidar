
cmake_minimum_required(VERSION 2.8)
# PROJECT(ydlidar_test)
add_compile_options(-std=c++11) # Use C++11

#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/YDLidar/
     ${CMAKE_CURRENT_BINARY_DIR}
)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(curdir ${CMAKE_CURRENT_SOURCE_DIR})
FILE(GLOB APP_LIST "${curdir}/*.cpp")
foreach(child ${APP_LIST})
  string(REPLACE "${curdir}/" "" app_main ${child})
  string(REPLACE ".cpp" "" APP_NAME ${app_main})
  ADD_EXECUTABLE(${APP_NAME} ${app_main})
  TARGET_LINK_LIBRARIES(${APP_NAME} ${PROJECT_NAME} matplotlib_cpp)

  INSTALL(TARGETS ${APP_NAME}
    RUNTIME DESTINATION bin
  )
endforeach()


