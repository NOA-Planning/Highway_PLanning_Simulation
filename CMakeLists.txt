# cmake_minimum_required(VERSION 3.12)
# project(bspline_lattice_planner)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE "Release")
# set(BUILD_SHARED_LIBS on)

# find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)

# set(INCLUDE_PATH ./include ${PYTHON_INCLUDE_DIRS} ./third_party/matplotlib-cpp)
# include_directories(${INCLUDE_PATH})
# aux_source_directory(./src/ SRC)


# add_executable(${PROJECT_NAME} ./test/main.cpp ${SRC})
# target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)

# # 创建一个名为planner的静态库
# add_library(planner STATIC ${SRC})
# target_include_directories(${PROJECT_NAME} PRIVATE ${Python_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)


# 指定CMake的最小版本要求，确保使用的CMake具有我们需要的功能。
cmake_minimum_required(VERSION 3.12)

# 定义项目名称和版本，同时指定项目使用的编程语言。
project(bspline_lattice_planner VERSION 1.0.0 LANGUAGES CXX)

# 设置可执行文件和库文件的输出目录，以便统一管理构建产物。
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 设置C++标准为C++11，并确保不使用特定编译器的扩展，增加代码的移植性。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 允许通过命令行或GUI选项指定构建类型（例如Debug或Release）。
# 默认为Debug模式。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# 查找Python3和相关组件，包括NumPy，确保项目可以链接和使用这些库。
find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 REQUIRED)
# 添加nlohmann/json库
#find_package(nlohmann_json REQUIRED)
find_package(pybind11 REQUIRED)


# 定义项目的包含目录路径，这些路径中的头文件可以在项目中直接包含。
set(INCLUDE_PATH 
    ${CMAKE_SOURCE_DIR}/include 
    ${Python3_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/third_party/matplotlib-cpp 
    ${EIGEN3_INCLUDE_DIR}
    ${PYBIND11_INCLUDE_DIRS}

)

# 自动查找当前项目源目录下的所有cpp文件，以便编译。
# 注意：使用GLOB来收集文件会使得CMake在新增或删除文件时不自动重新运行。
file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)
# aux_source_directory(src SRC_FILES)
# 添加一个可执行目标到项目中，并指定构建这个目标需要的源文件。
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/test/main.cpp ${SRC_FILES})
add_executable(park_data ${CMAKE_SOURCE_DIR}/test/park_data_test.cpp ${SRC_FILES})
add_executable(test ${CMAKE_SOURCE_DIR}/test/plot_test.cpp ${SRC_FILES})

# 为可执行目标设置私有包含目录，这意味着这些目录只对这个目标可见。
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_PATH})
target_include_directories(park_data PRIVATE ${INCLUDE_PATH})
target_include_directories(test PRIVATE ${INCLUDE_PATH})

# 链接Python库和NumPy库到可执行目标，确保目标可以使用这些外部库的功能。
target_link_libraries(${PROJECT_NAME} PRIVATE Python3::Python Python3::NumPy)
target_link_libraries(park_data
                      ${Python3_LIBRARIES} 
                      Python3::Python 
                      Python3::NumPy 
                      pybind11::module 
                      #nlohmann_json::nlohmann_json
)
target_link_libraries(test PRIVATE 
                      Python3::Python 
                      Python3::NumPy)


# 添加一个名为planner的静态库目标到项目中，指定构建这个库需要的源文件。
add_library(planner STATIC ${SRC_FILES})

# 设置planner库的公共包含目录。公共包含目录会传递给链接了这个库的其他目标。
target_include_directories(planner PUBLIC ${INCLUDE_PATH})

# 将Python库和NumPy库链接到planner库。由于是公共链接，任何链接了planner的目标也会自动链接这些库。
target_link_libraries(planner PUBLIC Python3::Python Python3::NumPy)
