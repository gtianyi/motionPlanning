#pragma once

#include "../../configuration/configuration.hpp"
#include "../../dependencies/catch.hpp"
#include "../../planners/beastplanner.hpp"
#include "../../samplers/integrated_beast.hpp"

namespace {

const std::string configuration = "Timeout ? 6000\n"
                                  "Memory ? 1000\n"
                                  "AddIntermediateStates ? true\n"
                                  "Seed ? 2\n"
                                  "Runs ? 1\n"
                                  "Domain ? Quadrotor\n"
                                  "PropagationStepSize ? 0.05\n"
                                  "MinControlDuration ? 1\n"
                                  "MaxControlDuration ? 100\n"
                                  "GoalRadius ? 1\n"
                                  "Start ? 3 -5\n"
                                  "Goal ? 0 -25\n"
                                  "EnvironmentBounds ? -30 30 -30 30 \n"
                                  "EnvironmentMesh ? forest.dae\n"
                                  "AgentMesh ? car2_planar_robot.dae\n"
                                  "Planner ? BEAST\n"
                                  "AbstractionType ? PRM\n"
                                  "NumEdges ? 3\n"
                                  "PRMSize ? 2000\n"
                                  "StateRadius ? 6\n"
                                  "WhichSearch ? Integrated\n"
                                  "ValidEdgeDistributionAlpha ? 10\n"
                                  "ValidEdgeDistributionBeta ? 1\n"
                                  "InvalidEdgeDistributionAlpha ? 1\n"
                                  "InvalidEdgeDistributionBeta ? 10\n"
                                  "SelectionRadius ? 2.0\n"
                                  "PruningRadius ? 1.0\n"
                                  "N0 ? 100000\n"
                                  "Xi ? 0.5\n"
                                  "SSTStyle ? SSTStar\n"
                                  "CostPruningStyle ? G\n"
                                  "Sampler ? BEAST\n"
                                  "Output ? outfile-beast-small\n"
                                  "RegionCount ? 1000\n"
                                  "InitialAlpha ? 1\n"
                                  "InitialBeta ? 1\n"
                                  "PRMResizeFactor ? 2";

using Region = IntegratedBeast::Region;

const std::shared_ptr<ompl::control::BeastPlanner<IntegratedBeast>>
generateIntegratedBeastInstance() {
    FileMap params;
    std::istringstream configurationStream(configuration);

    params.append(configurationStream);

    const auto benchmarkData = getBenchmarkData(params);
    auto planner = getPlanner(benchmarkData, params);

    auto beastPlanner = std::static_pointer_cast<
            ompl::control::BeastPlanner<IntegratedBeast>>(planner);

    beastPlanner->setProblemDefinition(
            benchmarkData.simplesetup->getProblemDefinition());

    return beastPlanner;
}

TEST_CASE("Connect regions (one way)", "[IntegratedBeast]") {
    auto beastPlanner = generateIntegratedBeastInstance();
    IntegratedBeast* integratedBeast = beastPlanner->sampler.get();
    Region region1(1, nullptr);
    Region region2(2, nullptr);

    integratedBeast->connectRegions(&region1, &region2);

    REQUIRE(region1.outEdges.size() == 1);
    REQUIRE(region2.outEdges.size() == 0);
    REQUIRE(region1.inEdges.size() == 0);
    REQUIRE(region2.inEdges.size() == 1);

    auto outEdge = integratedBeast->edges[region1.outEdges[0]];
    auto inEdge = integratedBeast->edges[region2.inEdges[0]];
    
    REQUIRE(outEdge == inEdge);
    REQUIRE(outEdge->sourceRegionId == 1);
    REQUIRE(outEdge->targetRegionId == 2);
    
    REQUIRE(outEdge->getInEdgeSourceRegionId() == 2);
    REQUIRE(outEdge->getInEdgeTargetRegionId() == 1);
    
    region1.outEdges.clear();
    region2.inEdges.clear();
}

TEST_CASE("Connect regions (bidirectional)", "[IntegratedBeast]") {
    auto beastPlanner = generateIntegratedBeastInstance();
    IntegratedBeast* integratedBeast = beastPlanner->sampler.get();
    Region region1(1, nullptr);
    Region region2(2, nullptr);

    integratedBeast->connectRegions(&region1, &region2);
    integratedBeast->connectRegions(&region2, &region1);

    REQUIRE(region1.outEdges.size() == 1);
    REQUIRE(region2.outEdges.size() == 1);
    REQUIRE(region1.inEdges.size() == 1);
    REQUIRE(region2.inEdges.size() == 1);
    
    region1.outEdges.clear();
    region1.inEdges.clear();
    region2.outEdges.clear();
    region2.inEdges.clear();
}
}
