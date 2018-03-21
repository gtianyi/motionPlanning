#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <vector>
#include "../structs/filemap.hpp"

using RegionId = unsigned int;
using EdgeId = unsigned int;

class IntegratedBeast {
public:
    struct Region {
        Region(unsigned int id, ompl::base::State* state) : id{id}, state{state}, outEdges{}, inEdges{} {}

        const ompl::base::State* state;
        const RegionId id;

        std::vector<const EdgeId> outEdges;
        std::vector<const EdgeId> inEdges;
    };

    struct Edge {
        Edge(const RegionId sourceRegion, const RegionId targetRegion, unsigned int alpha, unsigned int beta)
                : sourceRegion{sourceRegion}, targetRegion{targetRegion}, alpha{alpha}, beta{beta} {}

        const unsigned int sourceRegion;
        const unsigned int targetRegion;
        unsigned int alpha;
        unsigned int beta;
    };

    IntegratedBeast(const ompl::base::SpaceInformation* spaceInformation,
            const ompl::base::State* start,
            const ompl::base::State* goal,
            const FileMap& params)
            : start{start},
              goal{goal},
              regions{},
              edges{},
              nearestRegions{},
              distanceFunction{[&globalParameters](const RegionId& lhs, const RegionId& rhs) {
                  return globalParameters.globalAbstractAppBaseGeometric->getStateSpace()->distance(
                          regions[lhs].state, regions[rhs].state);
              }},
              abstractSpace{globalParameters.globalAbstractAppBaseGeometric->getStateSpace()},
              abstractSampler{
                      globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler()},
              spaceInformation{spaceInformation} {
        if (spaceInformation->getStateSpace()->isMetricSpace()) {
            nearestRegions.reset(new ompl::NearestNeighborsGNATNoThreadSafety<RegionId>());
        } else {
            nearestRegions.reset(new ompl::NearestNeighborsSqrtApprox<RegionId>());
        }

        nearestRegions->setDistanceFunction(distanceFunction);
    }

    IntegratedBeast(const IntegratedBeast&) = delete;
    IntegratedBeast(IntegratedBeast&&) = delete;

    void initialize(){
        // TODO: get region counts from parameter file
        int regionCount = 100;
        initializeRegions(regionCount);

        // dijkstra or D *
        // push outgoing edges of start region into open
    }
    
    void initializeRegions(const size_t regionCount) {
        if (regionCount <= 2) {
            throw ompl::Exception("IntegratedBeast::initializeRegions", "Region count must be at least 3");
        }

        generateRegions(regionCount);
        connectRegions();
    }

    void splitRegion() {}

    bool sample(ompl::base::State* from, ompl::base::State* to){
        computerShortestPath();
        auto targetEdge = open.top();

        if(targetEdge->sourceId ==  targetEdge->targetId &&  targetEdge->sourceId ==  goalID) {
            spaceInformation->copyState(from, regions[targetEdge->sourceId].sampleState());
            goalSampler->sampleGoal(to);
        } else {
            spaceInformation->copyState(from,  vertices[targetEdge->sourceId].sampleState());
           
            auto regionCenter = regions[targetEdge->targetId].state;
            fullStateSampler->sampleUniformNear(to, regionCenter, stateRadius);
        }
        
        return false;
    }

    void reached(ompl::base::State *state) {
        return;
    }

private:
    virtual void generateRegions(const size_t regionCount) {
        regions.reserve(regionCount);

        // Add start
        auto startState = abstractSpace->allocState();
        abstractSpace->copyState(startState, start);
        regions.emplace_back(0, startState);
        nearestRegions->add(0);

        // Add goal
        auto goalState = abstractSpace->allocState();
        abstractSpace->copyState(goalState, goal);
        regions.emplace_back(1, goalState);
        nearestRegions->add(1);

        for (unsigned int i = 2; i < regionCount; ++i) {
            auto state = abstractSpace->allocState();
            abstractSampler->sample(state);
            regions.emplace_back(i, goalState);
            nearestRegions->add(i);
        }
    }

    void connectRegions() {
        for (auto& region : regions) {
            // Add k neighbors
            // TODO initialize edge count
            int edgeCount = 5;
            addKNeighbors(region, edgeCount);
        }
    }

    void connectRegions(Region& sourceRegion, Region& targetRegion) {
        const EdgeId edgeId = edges.size();
        // TODO initialize alpha and beta from parameters
        int alpha = 1;
        int beta = 1;

        edges.emplace_back(sourceRegion.id, targetRegion.id, alpha, beta);
        sourceRegion.outEdges.push_back(edgeId);
        targetRegion.inEdges.push_back(edgeId);
    }

    void addKNeighbors(Region& sourceRegion, size_t edgeCount) {
        std::vector<RegionId> neighborIds;
        nearestRegions->nearestK(sourceRegion.id, edgeCount, neighborIds);

        for (RegionId neighborId : neighborIds) {
            if (sourceRegion.id == neighborId)
                continue;

            auto neighborRegion = regions[neighborId];

            connectRegions(sourceRegion, neighborRegion);
            connectRegions(neighborRegion, sourceRegion);
        }
    }

    const ompl::base::State* start;
    const ompl::base::State* goal;
    std::vector<Region> regions;
    std::vector<Edge> edges;
    std::unique_ptr<ompl::NearestNeighbors<RegionId>> nearestRegions;
    const std::function<double(const RegionId, const RegionId)> distanceFunction;
    const ompl::base::StateSpacePtr abstractSpace;
    const ompl::base::ValidStateSamplerPtr abstractSampler;
    const ompl::base::SpaceInformation* spaceInformation;
};
