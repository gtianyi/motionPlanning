#pragma once

#include <math.h>
#include <ompl/base/Goal.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <vector>
#include "../structs/filemap.hpp"
#include "../structs/inplacebinaryheap.hpp"
#include "../structs/utils.hpp"
#include "halton_sampler.hpp"
#include "integrated_beast_base.hpp"

class IntegratedBeastPRM: public IntegratedBeastBase {
public:
    IntegratedBeastPRM(const ompl::base::SpaceInformation* spaceInformation,
            const ompl::base::State* start,
            const ompl::base::GoalPtr& goalPtr,
            ompl::base::GoalSampleableRegion* goalSampleableRegion,
            const FileMap& params)
            : startRegionId{0},
              goalRegionId{1},
              abstractStateRadius{params.doubleVal("AbstractStateRadius")},
              neighborEdgeCount{static_cast<const unsigned int>(
                      params.integerVal("NumEdges"))},
              resizeFactor{params.doubleVal("PRMResizeFactor")},
              haltonSampling{params.boolVal("HaltonSampling")},
              nearestRegions{},
              haltonSampler{globalParameters.abstractBounds},
              IntegratedBeastBase(spaceInformation,
                      start,
                      goalPtr,
                      goalSampleableRegion,
                      params) {
        if (spaceInformation->getStateSpace()->isMetricSpace()) {
            nearestRegions.reset(
                    new ompl::NearestNeighborsGNATNoThreadSafety<Region*>());
        } else {
            nearestRegions.reset(
                    new ompl::NearestNeighborsSqrtApprox<Region*>());
        }

        nearestRegions->setDistanceFunction(
                [](const Region* lhs, const Region* rhs) {
                    return globalParameters.globalAbstractAppBaseGeometric
                            ->getStateSpace()
                            ->distance(lhs->state, rhs->state);
                });
    }

    virtual void initialize() {
        goalEdgeTimer = new Timer("Goal edge timer");

        std::cout << "IntBeast::Full::Seed: " << ompl::RNG::getSeed()
                  << std::endl;

        fullStateSampler = spaceInformation->allocStateSampler();

        std::cout << "IntBeast::Abstract::Seed: " << ompl::RNG::getSeed()
                  << std::endl;
        abstractSpace = globalParameters.globalAbstractAppBaseGeometric
                                ->getStateSpace();
        abstractSampler = globalParameters.globalAbstractAppBaseGeometric
                                  ->getSpaceInformation()
                                  ->allocValidStateSampler();

        initializeRegions(initialRegionCount);

        regions[goalRegionId]->rhs = 0;
        regions[goalRegionId]->key = regions[goalRegionId]->calculateKey();
        inconsistentRegions.push(regions[goalRegionId]);

        computeShortestPath();
        addOutgoingEdgesToOpen(regions[startRegionId]);
    }

private:

    virtual void initializeRegions(const size_t regionCount) {
        if (regionCount <= 2) {
            throw ompl::Exception("IntegratedBeastBase::initializeRegions",
                    "Region count must be at least 3");
        }

        generateStartGoalRegions();
        generateRegions(regionCount);

        connectAllRegions();

        // Remove goal region out edges
        regions[goalRegionId]->outEdges.clear();
        regions[startRegionId]->inEdges.clear();

        ensureStartGoalConnectivity();

#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
    }

    void ensureStartGoalConnectivity() {
        while (!isStartGoalConnected()) {
            growAbstraction();
        }
    }

    bool isStartGoalConnected() {
        Timer("connectivity check");

        RegionId index = 0;
        std::vector<RegionId> open;
        std::unordered_set<RegionId> closed;

        open.emplace_back(startRegionId);
        while (index < open.size()) {
            RegionId currentRegionId = open[index];

            if (currentRegionId == goalRegionId) {
                return true;
            }

            auto currentRegion = regions[currentRegionId];

            for (auto outEdge : currentRegion->outEdges) {
                const RegionId neighborRegionId = outEdge->targetRegion->id;

                if (closed.find(neighborRegionId) != closed.end())
                    continue;

                closed.insert(neighborRegionId);
                open.emplace_back(neighborRegionId);
            }

            index++;
        }

        return false;
    }

    void growAbstraction() {
        auto newRegionCount = regions.size() * resizeFactor;
        generateRegions(static_cast<const size_t>(newRegionCount));
        clearAllEdges();
        connectAllRegions();
    }

    virtual void sampleFullState(const Region* samplingRegion, State* to) {
        ompl::base::ScopedState<> regionCenter(
                globalParameters.globalAppBaseControl
                        ->getGeometricComponentStateSpace());

        regionCenter = samplingRegion->state;
        ompl::base::ScopedState<> fullState =
                globalParameters.globalAppBaseControl
                        ->getFullStateFromGeometricComponent(regionCenter);

        while (true) {
            fullStateSampler->sampleUniformNear(
                    to, fullState.get(), stateRadius);

            auto sampleHostRegion = findRegion(to);
            if (sampleHostRegion->id == samplingRegion->id) {
                break;
            }
        }
    }

    State* sampleAbstractState(const Region* samplingRegion,
            const double samplingRadius) {
        auto state = abstractSpace->allocState();
        abstractSampler->sampleNear(state, samplingRegion->state, stateRadius);

        auto sampleHostRegion = findRegion(state);

        return state;
    }

    virtual Region* addStateToClosestRegion(State* state) {
        ompl::base::ScopedState<> incomingState(
                spaceInformation->getStateSpace());
        incomingState = state;
        auto region = findRegion(incomingState);

        region->addState(state);

        return region;
    }

    virtual void generateRegions(const size_t newRegionCount) {
        auto currentRegionCount = static_cast<unsigned int>(regions.size());

        if (newRegionCount <= currentRegionCount) {
            throw ompl::Exception("IntegratedBeastPRM::generateRegions",
                    "Can't generate less regions than the current count");
        }

        regions.reserve(newRegionCount);

        for (unsigned int i = currentRegionCount; i < newRegionCount; ++i) {
            auto state = abstractSpace->allocState();
            if (haltonSampling) {
                auto haltonSample = haltonSampler.sampleVector(i);
                globalParameters.copyVectorToAbstractState(state, haltonSample);
            } else {
                abstractSampler->sample(state);
            }

            auto region = allocateRegion(i, state);
            nearestRegions->add(region);
        }
    }

    void generateStartGoalRegions() {
        // Add start
        auto startState = abstractSpace->allocState();
        abstractSpace->copyState(startState, abstractStartState);
        auto startRegion = allocateRegion(startRegionId, startState);
        nearestRegions->add(startRegion);

        // Add startState to startRegion as a seed for the motion tree
        startRegion->addState(fullStartState);

        // Add goal
        auto goalState = abstractSpace->allocState();
        abstractSpace->copyState(goalState, abstractGoalState);
        auto goalRegion = allocateRegion(goalRegionId, goalState);
        nearestRegions->add(goalRegion);
    }

    virtual void connectAllRegions() {
        clearAllEdges();

        for (auto& region : regions) {
            region->outEdges.clear();
            region->inEdges.clear();
        }

        for (auto& region : regions) {
            addKNeighbors(region, neighborEdgeCount);
        }
    }

    void addKNeighbors(Region* sourceRegion,
            size_t edgeCount,
            bool bidirectional = false) {
        std::vector<Region*> neighbors;

        // Ignore goal region as a source
        if (goalRegionId == sourceRegion->id)
            return;

        // Add plus one for the current node.
        nearestRegions->nearestK(sourceRegion, edgeCount + 1, neighbors);

        for (auto neighborRegion : neighbors) {
            // Ignore the current node as a neighbor
            if (sourceRegion == neighborRegion)
                continue;

            // Ignore start region as a target
            if (startRegionId == neighborRegion->id)
                continue;

            connectRegions(sourceRegion, neighborRegion);
			// if we do bidirection, there could be duplicate
			// because we could do bidirection twice if two regions are 
			// in each other's k cloest neighbors
			// on the other hand, if we don't do bidirecion, we could 
			// miss some edges in case which one edge is another's neighbors
			// but not the other way around. 
            if (bidirectional) {
                connectRegions(neighborRegion, sourceRegion);
            }
        }
    }

    /**
     * Steps of region splitting:
     *
     * 1. Sample new region around the old region
     * 2. Redistribute the states
     * 3.
     * @param originalRegion
     */
    virtual void splitRegion(Region* originalRegion) {
        auto distanceFunction = nearestRegions->getDistanceFunction();

        double minDistance = std::numeric_limits<double>::max();
        for (auto outEdge : originalRegion->outEdges) {
            auto distance =
                    distanceFunction(originalRegion, outEdge->targetRegion);

            minDistance = std::min(minDistance, distance);
        }

        auto state =
                sampleAbstractState(originalRegion, std::max(1.0, minDistance));
        auto newRegion = allocateRegion(state);

        addKNeighbors(newRegion, neighborEdgeCount, true);
        nearestRegions->add(newRegion);
        inconsistentRegions.push(newRegion);
    }

    Region* findRegion(State* state) const {
        Region region(std::numeric_limits<RegionId>::max(), state);
        return nearestRegions->nearest(&region);
    }

    Region* findRegion(const ompl::base::ScopedState<>& s) const {
        //- 1 is intentional overflow on unsigned int
        auto ss = globalParameters.globalAppBaseControl
                          ->getGeometricComponentState(s, -1);

        Region center(std::numeric_limits<RegionId>::max(), ss.get());
        return nearestRegions->nearest(&center);
    }

    const RegionId startRegionId;
    const RegionId goalRegionId;

    const double abstractStateRadius;
    const double resizeFactor;
    const unsigned int neighborEdgeCount;
    const bool haltonSampling;

    std::unique_ptr<ompl::NearestNeighbors<Region*>> nearestRegions;

    const HaltonSampler haltonSampler;
};
