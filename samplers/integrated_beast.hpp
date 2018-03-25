#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <vector>
#include "../structs/filemap.hpp"
#include "../structs/httplib.hpp"
#include "../structs/inplacebinaryheap.hpp"

using RegionId = unsigned int;
using EdgeId = unsigned int;

class IntegratedBeast {
public:
    struct Region {
        struct StateWrapper {
            StateWrapper(ompl::base::State* state) : state(state) {}
            bool operator<(const StateWrapper& w) const {
                return selected < w.selected;
            }

            ompl::base::State* state;
            unsigned int selected = 0;
        };

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

        void addState(ompl::base::State* state) {
            states.emplace_back(state);
            std::push_heap(states.begin(), states.end());
        }

        ompl::base::State* sampleState() {
            auto state = states.front();
            std::pop_heap(states.begin(), states.end());
            states.pop_back();

            state.selected++;

            states.emplace_back(state);
            std::push_heap(states.begin(), states.end());

            return state.state;
        }

        const ompl::base::State* state;
        const RegionId id;

        std::vector<EdgeId> outEdges;
        std::vector<EdgeId> inEdges;

        double key;

        double g = std::numeric_limits<double>::infinity();
        double rhs = std::numeric_limits<double>::infinity();

        unsigned int statesCount;

    private:
        std::vector<StateWrapper> states;
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
                  totalEffort{std::numeric_limits<double>::infinity()},
                  heapIndex{std::numeric_limits<unsigned int>::max()} {}

        static bool pred(const Edge* lhs, const Edge* rhs) {
            return lhs->totalEffort < rhs->totalEffort;
        }

        static unsigned int getHeapIndex(const Edge* edge) {
            return edge->heapIndex;
        }

        static void setHeapIndex(Edge* edge, unsigned int heapIndex) {
            edge->heapIndex = heapIndex;
        }

        const RegionId sourceRegion;
        const RegionId targetRegion;
        unsigned int alpha;
        unsigned int beta;
        double totalEffort;
        bool interior;
        unsigned int heapIndex;
        double getEffort() const {
            if (alpha <= 0) {
                throw ompl::Exception("IntegratedBeast::Edge::getEffort",
                        "Alpha value must be greater than 0");
            }
            return (alpha + beta) / alpha;
        }

        double getBonusEffort(unsigned int numberOfStates) const {
            if (alpha <= 0) {
                throw ompl::Exception("IntegratedBeast::Edge::getBonusEffort",
                        "Alpha value must be greater than 0");
            }

            double probability = (alpha / (alpha + beta)) *
                            ((numberOfStates - 1) / numberOfStates) +
                    (1. / numberOfStates);
            double estimate = 1. / probability;

            return estimate;
        }
    };

    IntegratedBeast(const ompl::base::SpaceInformation* spaceInformation,
            const ompl::base::State* start,
            const ompl::base::GoalPtr& goalPtr,
            ompl::base::GoalSampleableRegion* goalSampleableRegion,
            const FileMap& params)
            : startRegionId{0},
              goalRegionId{1},
              stateRadius{params.doubleVal("StateRadius")},
              regionCount{static_cast<const unsigned int>(
                      params.integerVal("RegionCount"))},
              neighborEdgeCount{static_cast<const unsigned int>(
                      params.integerVal("NumEdges"))},
              initialAlpha{static_cast<const unsigned int>(
                      params.integerVal("InitialAlpha"))},
              initialBeta{static_cast<const unsigned int>(
                      params.integerVal("InitialBeta"))},
              start{start},
              goal{goalPtr.get()->as<ompl::base::GoalState>()->getState()},
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
                    new ompl::NearestNeighborsGNATNoThreadSafety<Region*>());
        } else {
            nearestRegions.reset(
                    new ompl::NearestNeighborsSqrtApprox<Region*>());
        }

        distanceFunction = [this](const Region* lhs, const Region* rhs) {
            return globalParameters.globalAbstractAppBaseGeometric
                    ->getStateSpace()
                    ->distance(lhs->state, rhs->state);
        };

        nearestRegions->setDistanceFunction(distanceFunction);
    }

    IntegratedBeast(const IntegratedBeast&) = delete;
    IntegratedBeast(IntegratedBeast&&) = delete;

    ~IntegratedBeast() {
        // Free all regions and edges
        for (auto region : regions) {
            abstractSpace->freeState(
                    const_cast<ompl::base::State*>(region->state));
            delete region;
        }

        for (auto edge : edges) {
            delete edge;
        }
    }

    void initialize() {
        initializeRegions(regionCount);
        computeShortestPath();
        addOutgoingEdgesToOpen(startRegionId);
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

        auto targetEdge = open.pop();

        const RegionId sourceRegionId = targetEdge->sourceRegion;
        const RegionId targetRegionId = targetEdge->targetRegion;

        auto sourceRegionSample = regions[sourceRegionId]->sampleState();
        spaceInformation->copyState(from, sourceRegionSample);

        if (sourceRegionId == targetRegionId &&
                sourceRegionId == goalRegionId) {
            goalRegionSampler->sampleGoal(to);
        } else {
            // sample near is wrong,  it may not inside boundary,
            // we the need sampling technique wheeler introduct
            auto targetRegionCenter = regions[targetRegionId]->state;
            fullStateSampler->sampleUniformNear(
                    to, targetRegionCenter, stateRadius);
        }

        return true;
    }

    void reached(ompl::base::State* state) {
        //      ompl::base::ScopedState <>  incomingState(si_->getStateSpace());
        //      incomingState =  state;
        //      RegionId regionId =  stateToRegionId(incomingState);
        //
        //      regions[regionId].addState(state);
        //
        //      if (targetEdge != NULL && regionId == targetEdge->endID) {
        //          targetSuccess = true;
        //      } else {
        //          addOutgoingEdgesToOpen(regionId);
        //      }
        return;
    }

private:
    virtual void generateRegions(const size_t regionCount) {
        regions.reserve(regionCount);

        // Add start
        auto startState = abstractSpace->allocState();
        abstractSpace->copyState(startState, start);
        Region* startRegion = new Region(startRegionId, startState);
        regions.push_back(startRegion);
        nearestRegions->add(startRegion);

        // Add goal
        auto goalState = abstractSpace->allocState();
        abstractSpace->copyState(goalState, goal);
        Region* goalRegion = new Region(goalRegionId, startState);
        regions.push_back(goalRegion);
        nearestRegions->add(goalRegion);

        for (unsigned int i = 2; i < regionCount; ++i) {
            auto state = abstractSpace->allocState();
            abstractSampler->sample(state);
            Region* region = new Region(i, state);
            regions.push_back(region);
            nearestRegions->add(region);
        }
    }

    void connectRegions() {
        for (auto& region : regions) {
            // Add k neighbors
            addKNeighbors(region, neighborEdgeCount);
        }
    }

    void connectRegions(Region* sourceRegion, Region* targetRegion) {
        const EdgeId edgeId = static_cast<const EdgeId>(edges.size());

        edges.push_back(new Edge(
                sourceRegion->id, targetRegion->id, initialAlpha, initialBeta));

        sourceRegion->outEdges.push_back(edgeId);
        targetRegion->inEdges.push_back(edgeId);
    }

    void addKNeighbors(Region* sourceRegion, size_t edgeCount) {
        std::vector<Region*> neighbors;
        nearestRegions->nearestK(sourceRegion, edgeCount, neighbors);

        for (auto neighborRegion : neighbors) {
            if (sourceRegion == neighborRegion)
                continue;
            
            connectRegions(sourceRegion, neighborRegion);
            connectRegions(neighborRegion, sourceRegion);
        }
    }

    double getInteriorEdgeEffort(Edge* edge) {
        double numberOfStates = regions[edge->targetRegion]->statesCount;

        double bestValue = std::numeric_limits<double>::infinity();
        std::vector<EdgeId> outEdgeIds = regions[edge->targetRegion]->outEdges;

        for (auto n : outEdgeIds) {
            Edge* e = edges[n];
            double value = e->getBonusEffort(numberOfStates) + regions[n]->g;

            if (value < bestValue) {
                bestValue = value;
            }
        }

        return bestValue;
    }

    void addOutgoingEdgesToOpen(RegionId region) {
        for (auto edgeId : regions[region]->outEdges) {
            if (!open.inHeap(edges[edgeId])) {
                open.push(edges[edgeId]);
            }
        }
    }

    void insertOrUpdateOpen(Edge* edge) {
        if (!open.inHeap(edge)) {
            open.push(edge);
        } else {
            open.siftFromItem(edge);
        }
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

                    double totalEffort = edge->interior ?
                            getInteriorEdgeEffort(edge) :
                            u->g + edge->getEffort();

                    edge->totalEffort = totalEffort;
                }

                for (auto outEdgeId : u->outEdges) {
                    updateRegion(edges[outEdgeId]->targetRegion);
                }
            } else {
                u->g = std::numeric_limits<double>::infinity();

                for (auto edgeId : u->inEdges) {
                    Edge* edge = edges[edgeId];

                    const double totalEffort = edge->interior ?
                            getInteriorEdgeEffort(edge) :
                            u->g + edge->getEffort();

                    edge->totalEffort = totalEffort;
                }

                // Update this region
                updateRegion(u->id);

                for (auto outEdgeId : u->outEdges) {
                    updateRegion(edges[outEdgeId]->targetRegion);
                }
            }
        }
    }

public:
    void publishAbstractGraph() {
        std::cout << "Graph test" << std::endl;
        httplib::Client cli("localhost", 8080, 300, httplib::HttpVersion::v1_1);

        std::ostringstream commandBuilder;

        for (auto* region : regions) {
            auto se3state =
                    region->state
                            ->as<ompl::base::CompoundStateSpace::StateType>()
                            ->as<ompl::base::SE3StateSpace::StateType>(0);

            commandBuilder << "{\"an\":{\"" << region->id
                           << "\":{\"label\":\"Streaming\",\"size\":2}}}"
                           << std::endl;
        }

        for (auto* edge : edges) {
            commandBuilder << "{\"ae\":{\"" << edge << "\":{"
                           << "\"source\":\"" << edge->sourceRegion << "\","
                           << "\"target\":\"" << edge->targetRegion << "\"}}}"
                           << std::endl;
        }

        std::string commandString = commandBuilder.str();

        std::cout << commandString << std::endl;
        auto res = cli.post("/workspace1?operation=updateGraph",
                commandString,
                "plain/text");
        //        auto res2 = cli.post("/workspace1?operation=updateGraph",
        //        commandString, "application/x-www-form-urlencoded");

        std::cout << res->status << std::endl;
        std::cout << res->body << std::endl;
        std::cout << "Graph test end" << std::endl;
    }

    void updateRegion(const unsigned int region) {
        Region* s = regions[region];
        if (s->id != goalRegionId) {
            double minValue = std::numeric_limits<double>::infinity();

            std::vector<EdgeId> outEdgeIds = s->outEdges;

            for (auto n : outEdgeIds) {
                Edge* e = edges[n];
                double value = regions[n]->g + e->getEffort();
                if (value < minValue) {
                    minValue = value;
                }
            }
            s->rhs = minValue;
        }

        if (inconsistentRegions.inHeap(regions[region])) {
            inconsistentRegions.remove(regions[region]);
        }

        if (s->g != s->rhs) {
            s->key = s->calculateKey();
            inconsistentRegions.push(regions[region]);
        }
    }

    RegionId stateToRegionId(const ompl::base::ScopedState<>& s) {
        //- 1 is intentional overflow on unsigned int
        //        auto ss = globalParameters.globalAppBaseControl
        //                          ->getGeometricComponentState(s, -1);
        //
        //        Region center(10000,ss.get());
        //        return nearestRegions->nearest(&center)->id;
        return 0;
    }

    const RegionId startRegionId;
    const RegionId goalRegionId;

    const double stateRadius;
    const unsigned int regionCount;
    const unsigned int neighborEdgeCount;
    const unsigned int initialAlpha;
    const unsigned int initialBeta;

    const ompl::base::State* start;
    const ompl::base::State* goal;
    std::vector<Region*> regions;
    std::vector<Edge*> edges;
    std::unique_ptr<ompl::NearestNeighbors<Region*>> nearestRegions;
    std::function<double(const Region*, const Region*)> distanceFunction;
    const ompl::base::SpaceInformation* spaceInformation;
    const ompl::base::StateSamplerPtr fullStateSampler;
    const ompl::base::StateSpacePtr abstractSpace;
    const ompl::base::ValidStateSamplerPtr abstractSampler;
    const ompl::base::GoalSampleableRegion* goalRegionSampler;

    InPlaceBinaryHeap<Region, Region> inconsistentRegions;
    InPlaceBinaryHeap<Edge, Edge> open;
};
