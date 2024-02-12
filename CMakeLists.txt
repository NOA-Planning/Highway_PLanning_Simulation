cmake_minimum_required(VERSION 3.12)
project(bspline_lattice_planner)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE "Debug")
set(BUILD_SHARED_LIBS on)

SET(INCLUDE_PATH ./include)
INCLUDE_DIRECTORIES(${INCLUDE_PATH})

AUX_SOURCE_DIRECTORY(./src/ SRC)

# 使用FindPython模块查找Python
find_package(Python 2.7 REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Python Include Dirs: ${Python_INCLUDE_DIRS}")

add_executable(${PROJECT_NAME} ./test/main.cpp ${SRC})

# 创建一个名为planner的静态库
add_library(planner STATIC ${SRC})
target_include_directories(${PROJECT_NAME} PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Python_LIBRARIES})
