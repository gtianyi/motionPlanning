#pragma once


#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
// #include <ompl/control/planners/sst/SST.h>

#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/geometric/planners/rrt/RRT.h>


#include "../domains/DynamicCarPlanning.hpp"
#include "../domains/KinematicCarPlanning.hpp"
#include "../domains/blimp.hpp"
#include "../domains/carsetup.hpp"
#include "../domains/hovercraft.hpp"
#include "../domains/linkage.hpp"
#include "../domains/quadrotor.hpp"
#include "../domains/straightline.hpp"
// #include "domains/robotarm.hpp"
// #include "domains/acrobot.hpp"

#include "../samplers/beastsampler_dijkstra.hpp"
#include "../samplers/beastsampler_dstar.hpp"
#include "../samplers/beastsampler_dstarNewBonus.hpp"
#include "../samplers/beastsampler_dstarNoGeometricTest.hpp"
#include "../samplers/integrated_beast_prm.hpp"

#include "../planners/KPIECE.hpp"
#include "../planners/RRT.hpp"
#include "../planners/SST.hpp"
#include "../planners/beastplanner.hpp"
#include "../planners/beastplannerUpdateRightEdge.hpp"
#include "../planners/beastplannergeometric.hpp"
#include "../planners/beastplannernew.hpp"
#include "../planners/beats_planner.hpp"
#include "../planners/fbiasedrrt.hpp"
#include "../planners/fbiasedshellrrt.hpp"
#include "../planners/gust.hpp"
#include "../planners/plakurrt.hpp"

#include "../planners/anytimebeastcostplanner.hpp"
#include "../planners/anytimebeastplanner.hpp"
#include "../planners/anytimebeastplannernew.hpp"
#include "../planners/restartingrrtwithpruning.hpp"
#include "../planners/sststar.hpp"
#include "../planners/uctplanner.hpp"
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include "../structs/utils.hpp"
#include "../structs/filemap.hpp"

ompl::base::PlannerPtr getPlanner(const BenchmarkData& benchmarkData,
                                  const FileMap& params) {
    auto planner = params.stringVal("Planner");

    const auto spaceInformation =
        benchmarkData.simplesetup->getSpaceInformation();

    ompl::base::PlannerPtr plannerPointer;
    if (planner.compare("RRT") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::RRT(spaceInformation));
//        // plannerPointer = ompl::base::PlannerPtr(new
//        // ompl::control::RRTLocal(benchmarkData.simplesetup->getSpaceInformation()));
//    } else if (planner.compare("KPIECE") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::KPIECE1(spaceInformation));
//        // plannerPointer = ompl::base::PlannerPtr(new
//        // ompl::control::KPIECELocal(benchmarkData.simplesetup->getSpaceInformation()));
//    } else if (planner.compare("EST") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::EST(spaceInformation));
//    } else if (planner.compare("SyclopRRT") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(
//            spaceInformation, benchmarkData.decomposition));
//    } else if (planner.compare("SyclopEST") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopEST(
//            spaceInformation, benchmarkData.decomposition));
//    } else if (planner.compare("PDST") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::PDST(spaceInformation));
//    } else if (planner.compare("FBiasedRRT") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::FBiasedRRT(spaceInformation, params));
//    } else if (planner.compare("FBiasedShellRRT") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::FBiasedShellRRT(spaceInformation, params));
//    } else if (planner.compare("PlakuRRT") == 0) {
//        plannerPointer = ompl::base::PlannerPtr(
//            new ompl::control::PlakuRRT(spaceInformation, params));
    } else if (planner.compare("BEAST") == 0) {
        auto whichSearch = params.stringVal("WhichSearch");
        if (whichSearch.compare("D*") == 0) {
            plannerPointer =
                ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
                    ompl::base::BeastSampler_dstar>(
                    spaceInformation, params));
//        } else if (whichSearch.compare("Dijkstra") == 0) {
//            plannerPointer =
//                ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
//                    ompl::base::BeastSampler_dijkstra>(
//                    spaceInformation, params));
//        } else if (whichSearch.compare("D*BONUS") == 0) {
//            plannerPointer =
//                ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
//                    ompl::base::BeastSampler_dstarNewBonus>(
//                    spaceInformation, params));
//        } else if (whichSearch.compare("D*NOGEOMETRIC") == 0) {
//            plannerPointer =
//                ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
//                    ompl::base::BeastSampler_dstarNoGeometricTest>(
//                    spaceInformation, params));
        } else if (whichSearch.compare("IntegratedPRM") == 0) {
            plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::BeastPlanner<IntegratedBeastPRM>(
                    spaceInformation, params));
        } else {
            throw ompl::Exception(
                "Unrecognized best first search type", whichSearch.c_str());
        }

    }
    else if (planner.compare("BEASTnew") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::BeastPlannernew(spaceInformation, params));
    } else if (planner.compare("BEASTUpdateRightEdge") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::BeastPlannerUpdateRightEdge(
                spaceInformation, params));
    } else if (planner.compare("UCTtest") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::UCTPlanner(spaceInformation, params));
    } else if (planner.compare("BEATS") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::BeatsPlanner(spaceInformation, params));
    } else if (planner.compare("GUST") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::gust(spaceInformation, params));
    }

        /* anytime planners */
    else if (planner.compare("SST") == 0) {
        // plannerPointer = ompl::base::PlannerPtr(new
        // ompl::control::SST(benchmarkData.simplesetup->getSpaceInformation()));;
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::SSTLocal(spaceInformation, params));
        ;
    } else if (planner.compare("SST*") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::SSTStar(spaceInformation, params));
    } else if (planner.compare("RestartingRRTWithPruning") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
            new ompl::control::RestartingRRTWithPruning(spaceInformation));
        //  } else if(planner.compare("AnytimeBEAST") == 0) {
        //    plannerPointer = ompl::base::PlannerPtr(new
        //    ompl::control::AnytimeBeastPlanner(benchmarkData.simplesetup->getSpaceInformation(),
        //    params));
        //  } else if(planner.compare("AnytimeBEASTCost") == 0) {
        //    plannerPointer = ompl::base::PlannerPtr(new
        //    ompl::control::AnytimeBeastCostPlanner(benchmarkData.simplesetup->getSpaceInformation(),
        //    params));
        //  } else if(planner.compare("AnytimeBEASTnew") == 0) {
        //    plannerPointer = ompl::base::PlannerPtr(new
        //    ompl::control::AnytimeBeastPlannernew(benchmarkData.simplesetup->getSpaceInformation(),
        //    params));
        //  } else if(planner.compare("Atempts") == 0) {
        //    plannerPointer = ompl::base::PlannerPtr(new
        //    ompl::control::AtemptsPlanner(benchmarkData.simplesetup->getSpaceInformation(),
        //    params));
    } else {
        throw ompl::Exception("Unknown planner");
    }

    return plannerPointer;
}



BenchmarkData getBenchmarkData(const FileMap& params) {
    auto domain = params.stringVal("Domain");

    BenchmarkData benchmarkData;

    if (domain.compare("Blimp") == 0) {
        benchmarkData = blimpBenchmark(params);
        streamPoint = stream3DPoint;
    } else if (domain.compare("Quadrotor") == 0) {
        benchmarkData = quadrotorBenchmark(params);
        streamPoint = stream3DPoint;
    } else if (domain.compare("KinematicCar") == 0) {
        benchmarkData = carBenchmark<ompl::app::KinematicCarPlanning>(params);
        streamPoint = stream2DPoint2;
    } else if (domain.compare("DynamicCar") == 0) {
        benchmarkData = carBenchmark<ompl::app::DynamicCarPlanning>(params);
        streamPoint = stream2DPoint;
    } else if (domain.compare("StraightLine") == 0) {
        benchmarkData = straightLineBenchmark(params);
        streamPoint = stream2DPoint2;
        streamLine = stream2DLine2;
    } else if (domain.compare("Hovercraft") == 0) {
        benchmarkData = hovercraftBenchmark(params);
        streamPoint = stream2DPoint;
    } else if (domain.compare("Linkage") == 0) {
        benchmarkData = linkageBenchmark(params);
        streamPoint = stream2DPoint;
    }
        // else if(domain.compare("RobotArm") == 0) {
        // 	auto benchmarkData = robotArmBenchmark(params);
        // 	streamPoint = stream3DPoint;
        // } else if(domain.compare("Acrobot") == 0) {
        // 	auto benchmarkData = acrobotBenchmark(params);
        // 	streamPoint = stream2DPoint;
        // }
    else {
        throw ompl::Exception("Unknown domain");
    }

    return benchmarkData;
}


