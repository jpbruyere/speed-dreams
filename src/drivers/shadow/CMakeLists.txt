INCLUDE(../../../cmake/macros.cmake)

# The ubiquitous robot module and its clones.
SET(ROBOT_NAME "shadow")

# Official-only shadow interface.
SET(ROBOT_INTERFACE LEGACY WELCOME)

SET(ROBOT_SOURCES src/Array.h
				src/AveragedData.cpp
				src/AveragedData.h
				src/Avoidance.cpp
				src/Avoidance.h
				src/CarModel.cpp
				src/CarModel.h
				src/ClothoidPath.cpp
				src/ClothoidPath.h
				src/Cubic.cpp
				src/Cubic.h
				src/CubicSpline.cpp
				src/CubicSpline.h
				src/GenericAvoidance.cpp
				src/GenericAvoidance.h
				src/LearnedGraph.cpp
				src/LearnedGraph.h
				src/LinearRegression.cpp 
				src/LinearRegression.h
				src/LinePath.cpp
				src/LinePath.h
				src/Driver.cpp
				src/Driver.h
				src/MyTrack.cpp
				src/MyTrack.h
				src/Opponent.cpp
				src/Opponent.h
				src/OptimisedPath.cpp 
				src/OptimisedPath.h
				src/ParametricCubic.cpp
				src/ParametricCubic.h
				src/Path.cpp 
				src/Path.h
				src/PathRecord.cpp 
				src/PathRecord.h
				src/PidController.cpp 
				src/PidController.h
				src/PitPath.cpp 
				src/PitPath.h
				src/PtInfo.cpp 
				src/PtInfo.h
				src/Quadratic.cpp 
				src/Quadratic.h
				src/Seg.cpp 
				src/Seg.h
				src/Shadow.cpp
				src/Shared.cpp
				src/Shared.h
				src/Span.cpp 
				src/Span.h
				src/Strategy.cpp
				src/Strategy.h
				src/TeamInfo.cpp 
				src/TeamInfo.h
				src/Utils.cpp 
				src/Utils.h
				src/Vec2d.h
				src/Vec3d.h)

# Official-only shadow instances.
SET(ROBOT_CLONES shadow_36GP shadow_67GP shadow_sc shadow_srw shadow_trb1 shadow_ls1 shadow_ls2 shadow_mp5
			shadow_mpa1 shadow_mpa11 shadow_mpa12 shadow_lp1 shadow_ref)

# The ubiquitous robot module and its clones.
ROBOT_MODULE(NAME ${ROBOT_NAME} VERSION 3.3.0 SOVERSION 1.0.0
             INTERFACE ${ROBOT_INTERFACE}
             SOURCES ${ROBOT_SOURCES}
             CLONENAMES ${ROBOT_CLONES})

# For data associated to the ubiquitous robot module and its clones,
# see data/drivers/shadow*/CMakeLists.txt
