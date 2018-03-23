#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <vector>
#include "../structs/filemap.hpp"
#include "../structs/inplacebinaryheap.hpp"

using RegionId = unsigned int;
using EdgeId = unsigned int;

class IntegratedBeast {
public:
    struct Region {
        Region(unsigned int id, ompl::base::State* state)
                : id{id}, state{state}, outEdges{}, inEdges{} {}

        static bool pred(const Region* lhs, const Region* rhs) {
            const double lhsPrimary = std::min(lhs->g, lhs->rhs);
            const double lhsSecondary = std::min(lhs->g, lhs->rhs);

            const double rhsPrimary = std::min(rhs->g, rhs->rhs);
            const double rhsSecondary = std::min(rhs->g, rhs->rhs);

            if (lhsPrimary == rhsPrimary) {
                return lhsSecondary < rhsSecondary;
            }
            return lhsPrimary < rhsPrimary;
        }

        static unsigned int getHeapIndex(const Region* region) {
            return region->heapIndex;
        }

        static void setHeapIndex(Region* region, unsigned int heapIndex) {
            region->heapIndex = heapIndex;
        }

        double calculateKey() { return std::min(g, rhs); }

        const ompl::base::State* state;
        const RegionId id;

        std::vector<const EdgeId> outEdges;
        std::vector<const EdgeId> inEdges;

        double key;

        double g = std::numeric_limits<double>::infinity();
        double rhs = std::numeric_limits<double>::infinity();

    private:
        unsigned int heapIndex;
    };

    struct Edge {
        Edge(const RegionId sourceRegion,
                const RegionId targetRegion,
                unsigned int alpha,
                unsigned int beta)
                : sourceRegion{sourceRegion},
                  targetRegion{targetRegion},
                  alpha{alpha},
                  beta{beta},
                  effort{std::numeric_limits<double>::infinity()},
                  heapIndex{std::numeric_limits<unsigned int>::max()} {}

        static bool pred(const Edge* lhs, const Edge* rhs) {
            return lhs->effort < rhs->effort;
        }

        static unsigned int getHeapIndex(const Edge* edge) {
            return edge->heapIndex;
        }

        static void setHeapIndex(Edge* edge, unsigned int heapIndex) {
            edge->heapIndex = heapIndex;
        }

        const unsigned int sourceRegion;
        const unsigned int targetRegion;
        unsigned int alpha;
        unsigned int beta;
        double effort;
        bool interior;
        unsigned int heapIndex;
        double getEstimatedRequiredSamples() {
            return 0; // TODO
        }
    };

    IntegratedBeast(const ompl::base::SpaceInformation* spaceInformation,
            const ompl::base::State* start,
            const ompl::base::State* goal,
            ompl::base::GoalSampleableRegion* goalSampleableRegion,
            const FileMap& params)
            : stateRadius{0}, // TODO get from params
              regionCount{0}, // TODO get from params
              start{start},
              goal{goal},
              regions{},
              edges{},
              nearestRegions{},
              distanceFunction{},
              spaceInformation{spaceInformation},
              fullStateSampler{spaceInformation->allocStateSampler()},
              abstractSpace{globalParameters.globalAbstractAppBaseGeometric
                                    ->getStateSpace()},
              abstractSampler{globalParameters.globalAbstractAppBaseGeometric
                                      ->getSpaceInformation()
                                      ->allocValidStateSampler()},
              goalRegionSampler{goalSampleableRegion} {
        if (spaceInformation->getStateSpace()->isMetricSpace()) {
            nearestRegions.reset(
                    new ompl::NearestNeighborsGNATNoThreadSafety<RegionId>());
        } else {
            nearestRegions.reset(
                    new ompl::NearestNeighborsSqrtApprox<RegionId>());
        }

        distanceFunction = [&globalParameters, this](
                const RegionId& lhs, const RegionId& rhs) {
            return globalParameters.globalAbstractAppBaseGeometric
                    ->getStateSpace()
                    ->distance(this->regions[lhs]->state,
                            this->regions[rhs]->state);
        };

        nearestRegions->setDistanceFunction(distanceFunction);
    }

    IntegratedBeast(const IntegratedBeast&) = delete;
    IntegratedBeast(IntegratedBeast&&) = delete;

    ~IntegratedBeast() {
        // Free all regions and edges
        for (auto region : regions) {
            delete region;
        }

        for (auto edge : edges) {
            delete edge;
        }
    }

    void initialize() {
        // TODO: get region counts from parameter file
        int regionCount = 100;
        initializeRegions(regionCount);

        // dijkstra or D *
        // push outgoing edges of start region into open
    }

    void initializeRegions(const size_t regionCount) {
        if (regionCount <= 2) {
            throw ompl::Exception("IntegratedBeast::initializeRegions",
                    "Region count must be at least 3");
        }

        generateRegions(regionCount);
        connectRegions();
    }

    void splitRegion() {}

    bool sample(ompl::base::State* from, ompl::base::State* to) {
        computeShortestPath();
        auto targetEdge = open.top();

        if (targetEdge->sourceId == targetEdge->targetId &&
                targetEdge->sourceId == goalRegionId) {
            spaceInformation->copyState(
                    from, regions[targetEdge->sourceId].sampleState());
            goalRegionSampler->sampleGoal(to);
        } else {
            spaceInformation->copyState(
                    from, regions[targetEdge->sourceId].sampleState());

            auto regionCenter = regions[targetEdge->targetId].state;
            fullStateSampler->sampleUniformNear(to, regionCenter, stateRadius);
        }

        return false;
    }

    void reached(ompl::base::State* state) { return; }

private:
    virtual void generateRegions(const size_t regionCount) {
        regions.reserve(regionCount);

        // Add start
        auto startState = abstractSpace->allocState();
        abstractSpace->copyState(startState, start);
        regions.push_back(new Region(startRegionId, startState));
        nearestRegions->add(startRegionId);

        // Add goal
        auto goalState = abstractSpace->allocState();
        abstractSpace->copyState(goalState, goal);
        regions.push_back(new Region(goalRegionId, startState));
        nearestRegions->add(goalRegionId);

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

    void connectRegions(Region* sourceRegion, Region* targetRegion) {
        const EdgeId edgeId = edges.size();
        // TODO initialize alpha and beta from parameters
        int alpha = 1;
        int beta = 1;

        edges.push_back(
                new Edge(sourceRegion->id, targetRegion->id, alpha, beta));

        sourceRegion->outEdges.push_back(edgeId);
        targetRegion->inEdges.push_back(edgeId);
    }

    void addKNeighbors(Region* sourceRegion, size_t edgeCount) {
        std::vector<RegionId> neighborIds;
        nearestRegions->nearestK(sourceRegion->id, edgeCount, neighborIds);

        for (RegionId neighborId : neighborIds) {
            if (sourceRegion->id == neighborId)
                continue;

            auto neighborRegion = regions[neighborId];

            connectRegions(sourceRegion, neighborRegion);
            connectRegions(neighborRegion, sourceRegion);
        }
    }

    double getInteriorEdgeEffort(Edge* edge) {
        // TODO
    }

    void updateRegion(const unsigned int region) {
        // TODO
    }

    void updateEdgeEffort(Edge* edge, double effort, bool b) {
        // TODO
    }

    void computeShortestPath() {
        while (!inconsistentRegions.isEmpty()) {
            Region* u = regions[inconsistentRegions.pop()->id];
            auto oldKey = u->key;
            auto newKey = u->calculateKey();

            if (oldKey < newKey) {
                u->key = newKey; // update key
                inconsistentRegions.push(u);
            } else if (u->g > u->rhs) {
                u->g = u->rhs;
                for (auto edgeId : u->inEdges) {
                    Edge* edge = edges[edgeId];

                    double effort = edge->interior ?
                            getInteriorEdgeEffort(edge) :
                            u->g +
                                    edge->getEstimatedRequiredSamples()
                                            updateEdgeEffort(
                                                    edge, effort, false)
                }

                for (auto outEdgeId : u->outEdges) {
                    updateRegion(edges[outEdgeId]->targetRegion);
                }
            } else {
                u->g = std::numeric_limits<double>::infinity();

                for (auto edgeId : u->inEdges) {
                    Edge* edge = edges[edgeId];
                    const double effort =
                            edge->interior ? getInteriorEdgeEffort(edge) : u->g;
                    updateEdgeEffort(edge, effort, false);
                }

                // Update this region
                updateRegion(u->id);

                for (auto outEdgeId : u->outEdges) {
                    updateRegion(edges[outEdgeId]->targetRegion);
                }
            }
        }
    }

    static constexpr RegionId startRegionId{0};
    static constexpr RegionId goalRegionId{1};

    const double stateRadius;
    const unsigned int regionCount;

    const ompl::base::State* start;
    const ompl::base::State* goal;
    std::vector<Region*> regions;
    std::vector<Edge*> edges;
    std::unique_ptr<ompl::NearestNeighbors<RegionId>> nearestRegions;
    std::function<double(const RegionId&, const RegionId&)> distanceFunction;
    const ompl::base::SpaceInformation* spaceInformation;
    const ompl::base::StateSamplerPtr fullStateSampler;
    const ompl::base::StateSpacePtr abstractSpace;
    const ompl::base::ValidStateSamplerPtr abstractSampler;
    const ompl::base::GoalSampleableRegion* goalRegionSampler;

    InPlaceBinaryHeap<Region, Region> inconsistentRegions;
    InPlaceBinaryHeap<Edge, Edge> open;
};
