cmake_minimum_required(VERSION 3.20)
project(diyRobotCpp VERSION 1.0.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(cmake/CompilerWarnings.cmake)

option(DIYROBOT_BUILD_TESTS "Build offline tests" ON)
option(DIYROBOT_BUILD_APPS "Build command-line and desktop applications" ON)
option(DIYROBOT_BUILD_QT "Build the optional Qt dashboard" ON)
option(DIYROBOT_BUILD_OPENCV "Build the optional OpenCV camera demo" ON)
option(DIYROBOT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(DIYROBOT_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(diyrobot
    src/protocol.cpp
    src/motion.cpp
    src/kinematics.cpp
    src/calibration.cpp
    src/vision.cpp
    src/grasp.cpp
    src/lerobot_pipeline.cpp
)
add_library(diyrobot::diyrobot ALIAS diyrobot)
target_include_directories(diyrobot PUBLIC include)
diyrobot_set_project_options(diyrobot)

if(WIN32)
    target_compile_definitions(diyrobot PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

if(DIYROBOT_BUILD_APPS)
    add_executable(diyrobot_cli apps/diyrobot_cli.cpp)
    target_link_libraries(diyrobot_cli PRIVATE diyrobot)
    diyrobot_set_project_options(diyrobot_cli)

    if(DIYROBOT_BUILD_OPENCV)
        find_package(OpenCV QUIET COMPONENTS core highgui videoio imgproc)
        if(OpenCV_FOUND)
            add_executable(d4_camera apps/d4_camera_opencv.cpp)
            target_include_directories(d4_camera PRIVATE ${OpenCV_INCLUDE_DIRS})
            target_link_libraries(d4_camera PRIVATE diyrobot ${OpenCV_LIBS})
            diyrobot_set_project_options(d4_camera)
        else()
            message(STATUS "OpenCV C++ not found: d4_camera will not be built")
        endif()
    endif()

    if(DIYROBOT_BUILD_QT)
        find_package(Qt6 COMPONENTS Widgets QUIET)
        if(Qt6_FOUND)
            qt_standard_project_setup()
            qt_add_executable(d3_dashboard apps/d3_dashboard_qt.cpp)
            target_link_libraries(d3_dashboard PRIVATE diyrobot Qt6::Widgets)
            diyrobot_set_project_options(d3_dashboard)
        else()
            message(STATUS "Qt6 Widgets not found: d3_dashboard will not be built")
        endif()
    endif()
endif()

include(CTest)
if(BUILD_TESTING AND DIYROBOT_BUILD_TESTS)
    add_executable(diyrobot_tests tests/test_main.cpp)
    target_link_libraries(diyrobot_tests PRIVATE diyrobot)
    diyrobot_set_project_options(diyrobot_tests)
    add_test(NAME diyrobot_offline_tests COMMAND diyrobot_tests)
endif()

install(TARGETS diyrobot
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
if(DIYROBOT_BUILD_APPS)
    install(TARGETS diyrobot_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
