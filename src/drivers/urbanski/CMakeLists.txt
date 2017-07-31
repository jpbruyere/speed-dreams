INCLUDE(../../../cmake/macros.cmake)

SET(ROBOT_NAME "urbanski")

SET(ROBOT_SOURCES ${ROBOT_NAME}.cpp driver.cpp driver.h geoutil.cpp geoutil.h trackproc.cpp trackproc.h)

ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 1.0.0 SOVERSION 1.0.0
             INTERFACE LEGACY WELCOME
             SOURCES ${ROBOT_SOURCES})

# For data associated to the robot module, see data/drivers/urbanski/CMakeLists.txt
