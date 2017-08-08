cmake_minimum_required (VERSION 2.6)
project (MotionPlanning)

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g")


#add_library(OMPL STATIC IMPORTED)
add_executable(MotionPlanning main.cpp)

target_compile_definitions(MotionPlanning PRIVATE IKFAST_NO_MAIN)
#target_compile_definitions(MoreMotionPlanning PRIVATE STREAM_GRAPHICS IKFAST_NO_MAIN)

find_package(OMPL REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(LAPACK REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(FCL REQUIRED fcl)

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.50 COMPONENTS system REQUIRED)

include_directories(
	${OMPL_INCLUDE_DIRS}
	${ASSIMP_LIBRARY_DIRS}
)




target_link_libraries(MotionPlanning
	${OMPL_LIBRARIES}
	${OMPLAPP_LIBRARIES}
	${ASSIMP_LIBRARIES}
	${FCL_LIBRARIES}
	${Boost_LIBRARIES}
	${LAPACK_LIBRARIES}
)
