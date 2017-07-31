INCLUDE(../../../cmake/macros.cmake)

SET(ROBOT_NAME "hymie")

SET(ROBOT_SOURCES ${ROBOT_NAME}.cpp cardata.cpp  correct.cpp  driver.cpp
	learn.cpp  opponent.cpp  pit.cpp  spline.cpp  strategy.cpp cardata.h
	correct.h  driver.h  learn.h  linalg.h  name.h  opponent.h  pit.h
	spline.h  strategy.h)

ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 1.0.0 SOVERSION 1.0.0
             INTERFACE LEGACY_MIN
             SOURCES ${ROBOT_SOURCES})

# For data associated to the robot module, see data/drivers/hymie/CMakeLists.txt
