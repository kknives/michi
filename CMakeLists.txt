cmake_minimum_required(VERSION 3.1.0)

project(Michi VERSION 0.1 LANGUAGES CXX)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
message(FATAL_ERROR “In-source build detected!”)
endif()

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(spdlog REQUIRED)
pkg_check_modules(MAVLink mavlink_c)
pkg_check_modules(Asio asio)

add_library(Michi lib/realsense_generator.cpp lib/mobilenet_arrow.cpp lib/ardupilot_interface.cpp lib/yolov8_arrow.cpp)
target_compile_options(Michi PUBLIC -fcoroutines)
target_include_directories(Michi PUBLIC lib ${Asio_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${realsense_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS} PRIVATE ${MAVLink_INCLUDE_DIRS})
target_link_libraries(Michi PRIVATE ${PCL_LIBRARIES} ${OpenCV_LIBS}
    libonnxruntime.so
    PUBLIC ${realsense2_LIBRARY} Eigen3::Eigen spdlog::spdlog)

option(BUILD_RS_PCL_SCRIPT "Build rs-pcl-color.cpp script for prototyping" OFF)
if (BUILD_RS_PCL_SCRIPT)
    find_package(glfw3 3.2 REQUIRED)
    find_package(OpenGL REQUIRED)
    add_executable (rs-pcl-color bin/rs-pcl-color.cpp)
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

pkg_search_module(GTEST 1.12 REQUIRED gtest_main)
add_executable(test_realsense_generator tests/test_realsense_generator.cpp)
add_dependencies(test_realsense_generator Michi)
target_link_libraries(test_realsense_generator PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)
add_executable(test_mobilenet_arrow tests/test_mobilenet_arrow.cpp)
add_dependencies(test_mobilenet_arrow Michi)
target_link_libraries(test_mobilenet_arrow PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)
add_executable(test_yolov8_arrow tests/test_yolov8_arrow.cpp)
add_dependencies(test_yolov8_arrow Michi)
target_link_libraries(test_yolov8_arrow PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)
add_executable(test_ardupilot_interface tests/test_ardupilot_interface.cpp)
add_dependencies(test_ardupilot_interface Michi)
target_link_libraries(test_ardupilot_interface PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)

find_package(argparse REQUIRED)
add_executable(arar_planner bin/arrow_ardupilot_planner.cpp)
add_dependencies(arar_planner Michi)
target_include_directories(arar_planner PRIVATE argparse)
target_link_libraries(arar_planner PRIVATE Michi -fsanitize=address)

option(BUILD_MOBILENET_SCRIPT "Build run_mobilenet_arrow.cpp for running models" OFF)
if (BUILD_MOBILENET_SCRIPT)
    add_executable(run_mobilenet_arrow bin/run_mobilenet_arrow.cpp)
    add_dependencies(run_mobilenet_arrow Michi)
    target_link_libraries(run_mobilenet_arrow PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)
endif()

option(BUILD_ANNOTATE_ARROW_SCRIPT "Build annotate_arrow.cpp for annotating arrow detection" ON)
if (BUILD_ANNOTATE_ARROW_SCRIPT)
    add_executable(annotate_arrow bin/annotate_arrow.cpp)
    add_dependencies(annotate_arrow Michi)
    target_link_libraries(annotate_arrow PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)
endif()
option(BUILD_YOLO_SCRIPT "Build run_yolo.cpp for running YOLO ONNX models" ON)
if (BUILD_YOLO_SCRIPT )
    add_executable(run_yolo bin/run_yolo.cpp)
    add_dependencies(run_yolo Michi)
    target_link_libraries(run_yolo PRIVATE Michi ${GTEST_LDFLAGS} -fsanitize=address)
endif()
install(
    TARGETS
    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)

