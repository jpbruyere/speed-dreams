INCLUDE(../../../cmake/macros.cmake)

SET(ROBOT_NAME "replay")

SET(ROBOT_SOURCES ${ROBOT_NAME}.cpp)

ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 1.0.0 SOVERSION 1.0.0
             INTERFACE LEGACY_MIN
             SOURCES ${ROBOT_SOURCES})

# For data associated to the robot module, see data/drivers/hymie/CMakeLists.txt
