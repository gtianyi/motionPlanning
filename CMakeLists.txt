cmake_minimum_required(VERSION 2.6)
project(MotionPlanning)

set(CMAKE_MODULE_PATH
        "${CMAKE_MODULE_PATH}"
        "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g")

set(SOURCE_FILES
        domains/acrobot.hpp
        domains/AcrobotPlanning.hpp
        domains/AppBase.hpp
        domains/blimp.hpp
        domains/BlimpPlanning.hpp
        domains/carsetup.hpp
        domains/config.hpp
        domains/DynamicCarPlanning.hpp
        domains/hovercraft.hpp
        domains/HovercraftPlanning.hpp
        domains/KinematicCarPlanning.hpp
        domains/kinematicchainhelpers.hpp
        domains/linkage.hpp
        domains/quadrotor.hpp
        domains/QuadrotorPlanning.hpp
        domains/robotarm.hpp
        domains/RobotArmPlanning.hpp
        domains/SE2RigidBodyPlanning.hpp
        domains/SE3RigidBodyPlanning.hpp
        domains/straightline.hpp
        domains/StraightLinePlanning.hpp
        planners/anytimebeastcostplanner.hpp
        planners/anytimebeastplanner.hpp
        planners/anytimebeastplannernew.hpp
        planners/atemptsplanner.hpp
        planners/beastplanner.hpp
        planners/beastplannergeometric.hpp
        planners/beastplannernew.hpp
        planners/beastplannerUpdateRightEdge.hpp
        planners/beats_planner.hpp
        planners/fbiasedrrt.hpp
        planners/fbiasedshellrrt.hpp
        planners/KPIECE.hpp
        planners/plakurrt.hpp
        planners/restartingrrtwithpruning.hpp
        planners/RRT.hpp
        planners/SST.hpp
        planners/sststar.hpp
        planners/uctplanner.hpp
        planners/modules/costpruningmodule.hpp
        planners/modules/sstpruningmodule.hpp
        samplers/abstractionbasedsampler.hpp
        samplers/anytimebeastsampler.hpp
        samplers/anytimebeastsamplershim.hpp
        samplers/beastsampler_dijkstra.hpp
        samplers/beastsampler_dstar.hpp
        samplers/beastsampler_dstarDis.hpp
        samplers/beastsampler_dstarUpdateRightEdge.hpp
        samplers/beastsamplerbase.hpp
        samplers/beats_sampler.hpp
        samplers/fbiasedshellstatesampler.hpp
        samplers/fbiasedstatesampler.hpp
        samplers/plakustatesampler.hpp
        samplers/uctsampler.hpp
        #		samplers/guststatesmapler.hpp
        #		samplers/abstractions/prm_gust.hpp
        samplers/abstractions/grid.hpp
        samplers/abstractions/abstraction.hpp
        samplers/abstractions/prmlite.hpp
        samplers/abstractions/prm_lite_halton.hpp
        samplers/abstractionbasedsampler.hpp samplers/integrated_beast.hpp dependencies/httplib.hpp dependencies/catch.hpp configuration/configuration.hpp)

set(TEST_FILES
        test/samplers/integrated_beast_test.hpp)

#add_library(OMPL STATIC IMPORTED)
add_executable(MotionPlanning ${SOURCE_FILES} main.cpp)
add_executable(MotionPlanningTest ${TEST_FILES} test/test.cpp)

target_compile_definitions(MotionPlanning PRIVATE IKFAST_NO_MAIN)
target_compile_definitions(MotionPlanningTest PRIVATE IKFAST_NO_MAIN)

add_definitions(-DSTREAM_GRAPH)
#target_compile_definitions(MoreMotionPlanning PRIVATE STREAM_GRAPHICS IKFAST_NO_MAIN)

find_package(OMPL REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(LAPACK REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(fcl)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.66 COMPONENTS system REQUIRED)

include_directories(
        ${OMPL_INCLUDE_DIRS}
        ${ASSIMP_LIBRARY_DIRS}
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(MotionPlanning
        ${OMPL_LIBRARIES}
        ${OMPLAPP_LIBRARIES}
        ${ASSIMP_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${LAPACK_LIBRARIES}
        )

target_link_libraries(MotionPlanningTest
        ${OMPL_LIBRARIES}
        ${OMPLAPP_LIBRARIES}
        ${ASSIMP_LIBRARIES}
        ${FCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${LAPACK_LIBRARIES}
        )


