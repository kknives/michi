cmake_minimum_required(VERSION 3.1.0)

project(Michi VERSION 0.1 LANGUAGES CXX)

set(default_build_type Debug)
set(EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_library(Michi lib/realsense_generator.cpp)
target_compile_options(Michi PUBLIC -fcoroutines)
target_include_directories(Michi PUBLIC lib ${realsense_INCLUDE_DIR}
    PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(Michi PRIVATE ${PCL_LIBRARIES} ${OpenCV_LIBS} 
    PUBLIC ${realsense2_LIBRARY} Eigen3::Eigen)

option(BUILD_RS_PCL_SCRIPT "Build rs-pcl-color.cpp script for prototyping" OFF)
if (BUILD_RS_PCL_SCRIPT)
    find_package(glfw3 3.2 REQUIRED)
    find_package(OpenGL REQUIRED)
    add_executable (rs-pcl-color rs-pcl-color.cpp)
    message("rs-pcl-color requires C++11, it does not compile with C++20")
    set_property(TARGET rs-pcl-color PROPERTY CXX_STANDARD 11)
    target_include_directories(rs-pcl-color PUBLIC ${realsense_INCLUDE_DIR}
        PRIVATE ${PCL_INCLUDE_DIRS})
    include(CMakePrintHelpers)
    link_directories(${PCL_LIBRARY_DIRS})
    cmake_print_variables("${PCL_LIBRARY_DIRS}")
    target_link_directories(rs-pcl-color PRIVATE ${PCL_LIBRARY_DIRS})
    target_compile_definitions(rs-pcl-color PRIVATE ${PCL_DEFINITIONS})
    target_link_libraries(rs-pcl-color ${PCL_LIBRARIES} ${OpenCV_LIBS} 
        ${realsense2_LIBRARY} Eigen3::Eigen glfw ${OPENGL_LIBRARIES})
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
add_executable(test_realsense_generator tests/test_realsense_generator.cpp)
add_dependencies(test_realsense_generator Michi)
target_link_libraries(test_realsense_generator PRIVATE Michi GTest::gtest_main)

install(
    TARGETS
    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)

