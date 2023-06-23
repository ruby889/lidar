cmake_minimum_required(VERSION 3.2)
project(lidar)

# Architecture
if (CMAKE_CL_64) 
    set(ADDRESS_MODEL 64)
    set(NODE_TARGET x64)
else()
    set(ADDRESS_MODEL 32)
    set(NODE_TARGET x86)
endif()

include_directories(${PROJECT_SOURCE_DIR}/serial)
include_directories(${PROJECT_SOURCE_DIR}/src)

# OS
if (WIN32)
    message("OS: Windows")
    add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/serial/serial.h
        ${PROJECT_SOURCE_DIR}/serial/serial.c
        ${PROJECT_SOURCE_DIR}/serial_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}) 
elseif (UNIX AND NOT APPLE) 
    message("OS: Linux")
    add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/serial/serial.h
        ${PROJECT_SOURCE_DIR}/serial/serial.c
        ${PROJECT_SOURCE_DIR}/serial_test.cpp
    )
    target_link_libraries(${PROJECT_NAME} "-lm -lcurses -lpthread -lrt -Wall")
endif()