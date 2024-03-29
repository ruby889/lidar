cmake_minimum_required(VERSION 2.8)
project(lidar_sdk C CXX)

############################################################################
# matplotlib's cmake 
include(GNUInstallDirs)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)
############################################################################

include_directories(.)
include_directories(YDLidar)
add_subdirectory(YDLidar)
include_directories(src)

FILE(GLOB src_files "./src/*.cpp" "./src/*.h" "./src/*.c")
set(APP_NAME "lidar")
add_executable(${APP_NAME}
  ${src_files}
)
target_link_libraries(${APP_NAME} ${PROJECT_NAME} matplotlib_cpp)