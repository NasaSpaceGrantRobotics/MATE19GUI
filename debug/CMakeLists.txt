##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(debug)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp std_msgs sensor_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#SET(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/resources/*.qrc)


#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    )

add_definitions(-std=c++11 -fPIC)
##############################################################################
# Sources
##############################################################################

set(QT_SOURCES
    src/src/main.cpp
    src/src/debug.cpp
    src/src/qnode.cpp

    src/include/Debug/debug.h
    src/include/Debug/qnode.h
    )

##############################################################################
# Binaries
##############################################################################

# ${QT_RESOURCES_CPP}
add_executable(debug ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(debug
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
)
install(TARGETS debug RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
