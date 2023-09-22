cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("./cpp")
include_directories("./matplotlib-cpp/")

find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
add_executable(example fmt_example.cpp)
target_link_libraries(example ${PYTHON_LIBRARIES})


# test google test 
# enable_testing()
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})
# add_executable(test_di tests/test_double_integrator.cpp)
# target_link_libraries(test_di ${GTEST_LIBRARIES} pthread)
# 
# add_executable(test_rrt tests/test_rrt.cpp)
# target_link_libraries(test_rrt ${GTEST_LIBRARIES} pthread)
