cmake_minimum_required(VERSION 3.12)

project(Robot_kinematic)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

# add CMAKE_CURRENT_BINARY_DIR and CMAKE_CURRENT_SOURCE_DIR
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find aris
if(UNIX)
	add_definitions(-DUNIX)
        find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
#            find_package(aris REQUIRED PATHS C:/aris/aris-2.2.1.220620/share/aris/cmake)
find_package(aris REQUIRED PATHS C:/aris/aris-2.0.0.220225/share/aris/cmake)
endif(WIN32)

include_directories(${aris_INCLUDE_DIRS})

file(GLOB HEADERS
        src/*.h
        )
file(GLOB SOURCES
        src/*.cpp
        )

add_executable(Robot_kinematic ${SOURCES} ${HEADERS})

if(UNIX)
    target_link_libraries(Robot_kinematic ${aris_LIBRARIES} atomic)
endif(UNIX)
IF(WIN32)
    target_link_libraries(Robot_kinematic ${aris_LIBRARIES})
endif(WIN32)

