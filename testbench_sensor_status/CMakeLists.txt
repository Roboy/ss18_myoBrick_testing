cmake_minimum_required(VERSION 2.8.3)
project( testbench_sensor_status)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_communication_middleware )
find_package(OpenCV REQUIRED)
catkin_package(CATKIN_DEPENDS roboy_communication_middleware)

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_communication_middleware)
catkin_package(DEPENDS opencv)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(testbench_sensor_status_MOCS include/testbench_sensor_status/testbench_sensor_status.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(testbench_sensor_status_UIS_H src/testbench_sensor_status.ui)
else()
  qt4_wrap_cpp(testbench_sensor_status_MOCS include/testbench_sensor_status/testbench_sensor_status.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(testbench_sensor_status_UIS_H src/testbench_sensor_status.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library( testbench_sensor_status src/testbench_sensor_status.cpp ${testbench_sensor_status_MOCS} ${testbench_sensor_status_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(testbench_sensor_status roboy_communication_middleware_gencpp)

target_link_libraries(testbench_sensor_status ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(testbench_sensor_status Qt5::Widgets ${OpenCV_LIBRARIES})
else()
  target_link_libraries(testbench_sensor_status ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${OpenCV_LIBRARIES})
endif()

#############
## Install ##
#############

install(TARGETS testbench_sensor_status
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )