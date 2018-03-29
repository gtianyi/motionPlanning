#include <boost/bind.hpp>
#include "structs/utils.hpp"

GlobalParameters globalParameters;

#include <flann/flann.h>

#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
// #include <ompl/control/planners/sst/SST.h>

#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/geometric/planners/rrt/RRT.h>

#include "structs/filemap.hpp"

#include "domains/DynamicCarPlanning.hpp"
#include "domains/KinematicCarPlanning.hpp"
#include "domains/blimp.hpp"
#include "domains/carsetup.hpp"
#include "domains/hovercraft.hpp"
#include "domains/linkage.hpp"
#include "domains/quadrotor.hpp"
#include "domains/straightline.hpp"
// #include "domains/robotarm.hpp"
// #include "domains/acrobot.hpp"

#include "samplers/beastsampler_dijkstra.hpp"
#include "samplers/beastsampler_dstar.hpp"
#include "samplers/beastsampler_dstarNewBonus.hpp"
#include "samplers/beastsampler_dstarNoGeometricTest.hpp"
#include "samplers/integrated_beast.hpp"

#include "planners/KPIECE.hpp"
#include "planners/RRT.hpp"
#include "planners/SST.hpp"
#include "planners/beastplanner.hpp"
#include "planners/beastplannerUpdateRightEdge.hpp"
#include "planners/beastplannergeometric.hpp"
#include "planners/beastplannernew.hpp"
#include "planners/beats_planner.hpp"
#include "planners/fbiasedrrt.hpp"
#include "planners/fbiasedshellrrt.hpp"
#include "planners/gust.hpp"
#include "planners/plakurrt.hpp"

#include "planners/anytimebeastcostplanner.hpp"
#include "planners/anytimebeastplanner.hpp"
#include "planners/anytimebeastplannernew.hpp"
#include "planners/restartingrrtwithpruning.hpp"
#include "planners/sststar.hpp"
#include "planners/uctplanner.hpp"
//#include "planners/atemptsplanner.hpp"

void doBenchmarkRun(BenchmarkData benchmarkData, const FileMap& params) {
    auto planner = params.stringVal("Planner");

    ompl::base::PlannerPtr plannerPointer;
    const auto spaceInformation =
            benchmarkData.simplesetup->getSpaceInformation();
    if (planner.compare("RRT") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::RRT(spaceInformation));
        // plannerPointer = ompl::base::PlannerPtr(new
        // ompl::control::RRTLocal(benchmarkData.simplesetup->getSpaceInformation()));
    } else if (planner.compare("KPIECE") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::KPIECE1(spaceInformation));
        // plannerPointer = ompl::base::PlannerPtr(new
        // ompl::control::KPIECELocal(benchmarkData.simplesetup->getSpaceInformation()));
    } else if (planner.compare("EST") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::EST(spaceInformation));
    } else if (planner.compare("SyclopRRT") == 0) {
        plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopRRT(
                spaceInformation, benchmarkData.decomposition));
    } else if (planner.compare("SyclopEST") == 0) {
        plannerPointer = ompl::base::PlannerPtr(new ompl::control::SyclopEST(
                spaceInformation, benchmarkData.decomposition));
    } else if (planner.compare("PDST") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::PDST(spaceInformation));
    } else if (planner.compare("FBiasedRRT") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::FBiasedRRT(spaceInformation, params));
    } else if (planner.compare("FBiasedShellRRT") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::FBiasedShellRRT(spaceInformation, params));
    } else if (planner.compare("PlakuRRT") == 0) {
        plannerPointer = ompl::base::PlannerPtr(
                new ompl::control::PlakuRRT(spaceInformation, params));
    } else if (planner.compare("BEAST") == 0) {
        auto whichSearch = params.stringVal("WhichSearch");
        if (whichSearch.compare("D*") == 0) {
            plannerPointer =
                    ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
                            ompl::base::BeastSampler_dstar>(
                            spaceInformation, params));
        } else if (whichSearch.compare("Dijkstra") == 0) {
            plannerPointer =
                    ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
                            ompl::base::BeastSampler_dijkstra>(
                            spaceInformation, params));
        } else if (whichSearch.compare("D*BONUS") == 0) {
            plannerPointer =
                    ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
                            ompl::base::BeastSampler_dstarNewBonus>(
                            spaceInformation, params));
        } else if (whichSearch.compare("D*NOGEOMETRIC") == 0) {
            plannerPointer =
                    ompl::base::PlannerPtr(new ompl::control::BeastPlanner<
                            ompl::base::BeastSampler_dstarNoGeometricTest>(
                            spaceInformation, params));
        } else if (whichSearch.compare("Integrated") == 0) {
            plannerPointer = ompl::base::PlannerPtr(
                    new ompl::control::BeastPlanner<IntegratedBeast>(
                            spaceInformation, params));
        } else {
            throw ompl::Exception(
                    "Unrecognized best first search type", whichSearch.c_str());
        }

    } else if (planner.compare("BEASTnew") == 0) {
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
        plannerPointer = ompl::base::PlannerPtr(new ompl::control::gust(
                benchmarkData.simplesetup->getSpaceInformation(), params));
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
        fprintf(stderr, "unrecognized planner\n");
        return;
    }

    // allow unpenalized time for precomputation -- which is logged to the
    // output file
    //  plannerPointer->setProblemDefinition(benchmarkData.simplesetup->getProblemDefinition());
    // plannerPointer->setProblemDefinition(benchmarkData.gsetup->getProblemDefinition());
    //  plannerPointer->solve(0);

    if (params.boolVal("AddIntermediateStates") &&
            plannerPointer->params().hasParam("intermediate_states")) {
        plannerPointer->params().setParam("intermediate_states", "true");
    }
    benchmarkData.benchmark->addPlanner(plannerPointer);
    ompl::tools::Benchmark::Request req;
    req.maxTime = params.doubleVal("Timeout");
    req.maxMem = params.doubleVal("Memory");
    req.runCount = params.doubleVal("Runs");
    req.displayProgress = true;
    req.saveConsoleOutput = false;

    std::cout << "Do benchmark\n";

    benchmarkData.benchmark->benchmark(req);

    // save ompl output file to some place,
    // we don't need this for new experiment system
    // Mar - 11 - 2018,  by Tianyi Gu
    // benchmarkData.benchmark->saveResultsToFile(params.stringVal("Output").c_str());

    std::cout << "Results: \n";

    benchmarkData.benchmark->saveResultsToStream();

    std::cout << std::endl;

    // If there were multiple solutions being logged to global parameters,
    // append them to the output file
    // for some reason the OMPL benchmarking doesn't include all the solutions
    // added, just the final one
    if (globalParameters.solutionStream.solutions.size() > 0) {
        std::ofstream outfile;
        outfile.open(params.stringVal("Output").c_str(), std::ios_base::app);

        outfile << "Solution Stream\n";

        for (const auto& solution : globalParameters.solutionStream.solutions) {
            outfile << solution.second << " " << solution.first.value() << "\n";
        }
        outfile.close();
    }
}

int main(int argc, char* argv[]) {
    FileMap params;
    if (argc > 1) {
        params.append(argv[1]);
    } else {
        params.append(std::cin);
    }

    srand(params.integerVal("Seed"));
    flann::seed_random(params.integerVal("Seed"));
    ompl::RNG::setSeed(params.integerVal("Seed"));

    if (params.exists("NumControls"))
        howManyControls = params.integerVal("NumControls");

    auto domain = params.stringVal("Domain");
    if (domain.compare("Blimp") == 0) {
        auto benchmarkData = blimpBenchmark(params);
        streamPoint = stream3DPoint;
        doBenchmarkRun(benchmarkData, params);
    } else if (domain.compare("Quadrotor") == 0) {
        auto benchmarkData = quadrotorBenchmark(params);
        streamPoint = stream3DPoint;
        doBenchmarkRun(benchmarkData, params);
    } else if (domain.compare("KinematicCar") == 0) {
        auto benchmarkData =
                carBenchmark<ompl::app::KinematicCarPlanning>(params);
        streamPoint = stream2DPoint2;
        doBenchmarkRun(benchmarkData, params);
    } else if (domain.compare("DynamicCar") == 0) {
        auto benchmarkData =
                carBenchmark<ompl::app::DynamicCarPlanning>(params);
        streamPoint = stream2DPoint;
        doBenchmarkRun(benchmarkData, params);
    } else if (domain.compare("StraightLine") == 0) {
        auto benchmarkData = straightLineBenchmark(params);
        streamPoint = stream2DPoint2;
        streamLine = stream2DLine2;
        doBenchmarkRun(benchmarkData, params);
    } else if (domain.compare("Hovercraft") == 0) {
        auto benchmarkData = hovercraftBenchmark(params);
        streamPoint = stream2DPoint;
        doBenchmarkRun(benchmarkData, params);
    } else if (domain.compare("Linkage") == 0) {
        auto benchmarkData = linkageBenchmark(params);
        streamPoint = stream2DPoint;
        // doBenchmarkRun(benchmarkData, params);
    }
    // else if(domain.compare("RobotArm") == 0) {
    // 	auto benchmarkData = robotArmBenchmark(params);
    // 	streamPoint = stream3DPoint;
    // 	doBenchmarkRun(benchmarkData, params);
    // } else if(domain.compare("Acrobot") == 0) {
    // 	auto benchmarkData = acrobotBenchmark(params);
    // 	streamPoint = stream2DPoint;
    // 	doBenchmarkRun(benchmarkData, params);
    // }
    else {
        fprintf(stderr, "unrecognized domain\n");
    }

    return 0;
}
