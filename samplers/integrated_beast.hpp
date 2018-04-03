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

#ifdef STREAM_GRAPH
#include "../dependencies/httplib.hpp"
#endif

using RegionId = unsigned int;
using EdgeId = unsigned int;
using State = ompl::base::State;

class IntegratedBeast {
public:
    struct Edge;
    struct Region {
        struct StateWrapper {
            explicit StateWrapper(ompl::base::State* state) : state(state) {}
            bool operator<(const StateWrapper& w) const {
                return selected < w.selected;
            }

            ompl::base::State* state;
            unsigned int selected = 0;
        };

        Region(unsigned int id, ompl::base::State* state)
                : id{id}, state{state}, outEdges{}, inEdges{} {}

        Region(const Region&) = delete;
        Region(Region&&) = delete;

        static bool pred(const Region* lhs, const Region* rhs) {
            return lhs->key < rhs->key;
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

        int getTotalTries() const {
            int totalTries = 0;

            for (auto inEdge : inEdges) {
                totalTries += inEdge->alpha + inEdge->beta - 2;
            }

            return totalTries;
        }

        const ompl::base::State* state;
        const RegionId id;

        std::vector<Edge*> outEdges;
        std::vector<Edge*> inEdges;

        double key;

        double g = std::numeric_limits<double>::infinity();
        double rhs = std::numeric_limits<double>::infinity();

        std::vector<StateWrapper> states;

#ifdef STREAM_GRAPH
        bool alreadyVisualized = false;
#endif

    private:
        unsigned int heapIndex;
    };

    struct Edge {
        Edge(const EdgeId edgeId,
                Region* sourceRegion,
                Region* targetRegion,
                unsigned int alpha,
                unsigned int beta)
                : edgeId{edgeId},
                  sourceRegion{sourceRegion},
                  targetRegion{targetRegion},
                  alpha{alpha},
                  beta{beta},
                  totalEffort{std::numeric_limits<double>::infinity()},
                  heapIndex{std::numeric_limits<unsigned int>::max()},
                  interior{false} {}

        Edge(const Edge&) = delete;
        Edge(Edge&&) = delete;

        static bool pred(const Edge* lhs, const Edge* rhs) {
            // return (lhs->totalEffort + lhs->getPenalty()) < (rhs->totalEffort
            //    + rhs->getPenalty());
            if (lhs->totalEffort != rhs->totalEffort) {
                return lhs->totalEffort < rhs->totalEffort;
            }
            if (lhs->targetRegion->id != rhs->targetRegion->id) {
                return lhs->targetRegion->id < rhs->targetRegion->id;
            }
            if (lhs->sourceRegion->id != rhs->sourceRegion->id) {
                return lhs->sourceRegion->id < rhs->sourceRegion->id;
            }
            return false;
        }

        static unsigned int getHeapIndex(const Edge* edge) {
            return edge->heapIndex;
        }

        static void setHeapIndex(Edge* edge, unsigned int heapIndex) {
            edge->heapIndex = heapIndex;
        }

        double getPenalty() const {
            int totalTries = targetRegion->getTotalTries();
            if (totalTries < 10)
                return 0;

            double gamma = 0.01;
            const double penalty =
                    sqrt(gamma * (alpha + beta) / log(totalTries));
            return penalty;
        }

        double getEffort() const {
            if (alpha <= 0) {
                throw ompl::Exception("IntegratedBeast::Edge::getEffort",
                        "Alpha value must be greater than 0");
            }

            int totalTries = 0;
            for (auto inEdge : targetRegion->inEdges) {
                totalTries += inEdge->alpha;
            }

            const double penalty = sqrt((alpha + beta) / log(totalTries));
            const double expectedEffort =
                    (double)(alpha + beta) / (double)alpha;
            return expectedEffort;
        }

        double getBonusEffort(unsigned int numberOfStates) const {
            if (alpha <= 0) {
                throw ompl::Exception("IntegratedBeast::Edge::getBonusEffort",
                        "Alpha value must be greater than 0");
            }

            double probability =
                    ((double)alpha / ((double)alpha + (double)beta)) *
                            ((double)(numberOfStates - 1) /
                                    (double)numberOfStates) +
                    (1. / numberOfStates);
            double estimate = 1. / probability;
            return estimate;
        }

        double getScottBonusEffort(unsigned int numberOfStates) const {
            double additive = 1. / numberOfStates;
            //            double additive = (alpha + beta) /
            //            (double)numberOfStates;
            double probability = (alpha + additive) / (alpha + additive + beta);
            double estimate = 1. / probability;

            const double penalty = sqrt((alpha + beta) / log(numberOfStates));
            return estimate;
        }

        double getBonusEffort2(unsigned int numberOfStates) const {
            double additive = 1;
            //            double additive = (alpha + beta) /
            //            (double)numberOfStates;
            double probability = (alpha + additive) / (alpha + additive + beta);
            double estimate = 1. / probability;

            return estimate;
        }

        double getBonusEffort3(unsigned int numberOfStates) const { return 1; }

        Region* getInEdgeTargetRegion() const { return sourceRegion; }

        Region* getInEdgeSourceRegion() const { return targetRegion; }

        const EdgeId edgeId;
        Region* sourceRegion;
        Region* targetRegion;

        unsigned int alpha;
        unsigned int beta;

        double totalEffort;

        bool interior;

        unsigned int heapIndex;

#ifdef STREAM_GRAPH
        bool alreadyVisualized = false;
#endif
    };

    IntegratedBeast(const ompl::base::SpaceInformation* spaceInformation,
            const ompl::base::State* start,
            const ompl::base::GoalPtr& goalPtr,
            ompl::base::GoalSampleableRegion* goalSampleableRegion,
            const FileMap& params)
            : startRegionId{0},
              goalRegionId{1},
              stateRadius{params.doubleVal("StateRadius")},
              abstractStateRadius{params.doubleVal("AbstractStateRadius")},
              initialRegionCount{static_cast<const unsigned int>(
                      params.integerVal("RegionCount"))},
              neighborEdgeCount{static_cast<const unsigned int>(
                      params.integerVal("NumEdges"))},
              resizeFactor{params.doubleVal("PRMResizeFactor")},
              initialAlpha{static_cast<const unsigned int>(
                      params.integerVal("InitialAlpha"))},
              initialBeta{static_cast<const unsigned int>(
                      params.integerVal("InitialBeta"))},
              bonusType{static_cast<const unsigned int>(
                      params.integerVal("BonusType"))},
              useSplit{params.boolVal("UseSplit")},
              fullStartState{nullptr},
              fullGoalState{nullptr},
              abstractStartState{
                      globalParameters.globalAbstractAppBaseGeometric
                              ->getProblemDefinition()
                              ->getStartState(0)},
              abstractGoalState{globalParameters.globalAbstractAppBaseGeometric
                                        ->getProblemDefinition()
                                        ->getGoal()
                                        ->as<ompl::base::GoalState>()
                                        ->getState()},
              regions{},
              edges{},
              nearestRegions{},
              spaceInformation{spaceInformation},
              fullStateSampler{spaceInformation->allocStateSampler()},
              goalRegionSampler{goalSampleableRegion},
              haltonSampler{globalParameters.abstractBounds} {
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

        fullStartState = spaceInformation->allocState();
        spaceInformation->copyState(fullStartState, start);

        fullGoalState = spaceInformation->allocState();
        spaceInformation->copyState(fullGoalState,
                goalPtr.get()->as<ompl::base::GoalState>()->getState());
    }

    IntegratedBeast(const IntegratedBeast&) = delete;
    IntegratedBeast(IntegratedBeast&&) = delete;

    ~IntegratedBeast() {
        for (auto region : regions) {
            delete region;
        }

        clearAllEdges();
    }

    void initialize() {
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

    void initializeRegions(const size_t regionCount) {
        if (regionCount <= 2) {
            throw ompl::Exception("IntegratedBeast::initializeRegions",
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

    bool sample(ompl::base::State* from, ompl::base::State* to) {
        if (lastSelectedEdge != nullptr) {
            if (targetSuccess) {
                if (!addedGoalEdge &&
                        lastSelectedEdge->targetRegion->id == goalRegionId) {
                    addGoalEdge();
                }

                lastSelectedEdge->interior = true;
                lastSelectedEdge->alpha++;
            } else {
                lastSelectedEdge->beta++;
                if (useSplit && (lastSelectedEdge->beta % 10 == 0)) {
                    splitRegion(lastSelectedEdge->targetRegion);
                }
            }

            lastSelectedEdge->totalEffort = lastSelectedEdge->interior ?
                    getInteriorEdgeEffort(lastSelectedEdge) :
                    lastSelectedEdge->targetRegion->g +
                            lastSelectedEdge->getEffort();
            insertOrUpdateOpen(lastSelectedEdge);

            updateRegion(lastSelectedEdge->sourceRegion);
            computeShortestPath();

            if (targetSuccess) {
                addOutgoingEdgesToOpen(lastSelectedEdge->targetRegion);
            }
        }

        lastSelectedEdge = open.peek();
        targetSuccess = false;

//        std::cout << "Edge: " << lastSelectedEdge->sourceRegion->id << "->"
//                  << lastSelectedEdge->targetRegion->id
//                  << " Teffort:" << lastSelectedEdge->totalEffort
//                  << " Second best: " << open.peek()->totalEffort << "\n";

        // std::cout << "top total effort " << lastSelectedEdge->totalEffort
        //          << std::endl;
        // std::cout << "top edge: " << lastSelectedEdge->sourceRegionId << ","
        //          << lastSelectedEdge->targetRegionId << std::endl;
        // std::cout << "top target g "
        //          << regions[lastSelectedEdge->targetRegionId]->g <<
        //          std::endl;
        // std::cout << "top a b " << lastSelectedEdge->alpha << " "
        //          << lastSelectedEdge->beta << std::endl;

        const RegionId sourceRegionId = lastSelectedEdge->sourceRegion->id;
        const RegionId targetRegionId = lastSelectedEdge->targetRegion->id;

        auto sourceRegionSample = regions[sourceRegionId]->sampleState();

        spaceInformation->copyState(from, sourceRegionSample);

        if (sourceRegionId == targetRegionId &&
                sourceRegionId == goalRegionId) {
            goalRegionSampler->sampleGoal(to);
        } else {
            ompl::base::ScopedState<> targetRegionCenter(
                    globalParameters.globalAppBaseControl
                            ->getGeometricComponentStateSpace());

            targetRegionCenter = regions[targetRegionId]->state;
            ompl::base::ScopedState<> fullState =
                    globalParameters.globalAppBaseControl
                            ->getFullStateFromGeometricComponent(
                                    targetRegionCenter);

            fullStateSampler->sampleUniformNear(
                    to, fullState.get(), stateRadius);
            // alternative: rejection sampling
            // sampleFullState(regions[targetRegionId], to);
        }

#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
        return true;
    }

    void sampleFullState(const Region* samplingRegion, State* to) {
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

    bool isGoalEdge(Edge* edge) {
        const RegionId sourceRegionId = edge->sourceRegion->id;
        const RegionId targetRegionId = edge->targetRegion->id;
        return sourceRegionId == targetRegionId &&
                sourceRegionId == goalRegionId;
    }

    void addGoalEdge() {
        // We might want to tune the initial goal edge beta distribution.
        // to bias towards the goal edge
        auto goalEdge = new Edge(static_cast<const EdgeId>(edges.size()),
                regions[goalRegionId],
                regions[goalRegionId],
                1,
                1);

        edges.push_back(goalEdge);

        goalEdge->totalEffort = 1;

        open.push(goalEdge);
        // TODO add to inconsistentRegions
        addedGoalEdge = true;
    }

    void reached(ompl::base::State* state) {
        auto region = addStateToClosestRegion(state);

        if (lastSelectedEdge != nullptr &&
                region->id == lastSelectedEdge->targetRegion->id) {
            targetSuccess = true;
        } else {
            addOutgoingEdgesToOpen(region);
        }
    }

    Region* addStateToClosestRegion(State* state) {
        ompl::base::ScopedState<> incomingState(
                spaceInformation->getStateSpace());
        incomingState = state;
        auto region = findRegion(incomingState);

        region->addState(state);

        return region;
    }

    Region* allocateRegion(State* state) {
        return allocateRegion(static_cast<RegionId>(regions.size()), state);
    }

    Region* allocateRegion(const RegionId id, State* state) {
        auto region = new Region(id, state);
        regions.push_back(region);
        return region;
    }

    virtual void generateRegions(const size_t newRegionCount) {
        auto currentRegionCount = static_cast<unsigned int>(regions.size());

        if (newRegionCount <= currentRegionCount) {
            throw ompl::Exception("IntegratedBeast::generateRegions",
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

    void connectAllRegions() {
        clearAllEdges();

        for (auto& region : regions) {
            region->outEdges.clear();
            region->inEdges.clear();
        }

        for (auto& region : regions) {
            addKNeighbors(region, neighborEdgeCount);
        }
    }

    void clearAllEdges() {
        for (auto edge : edges) {
            removeEdge(edge);
        }

        edges.clear();
    }

    bool isConnected(IntegratedBeast::Region* sourceRegion,
            IntegratedBeast::Region* targetRegion) {
        for (auto outEdge : sourceRegion->outEdges) {
            if (outEdge->targetRegion->id == targetRegion->id)
                return true;
        }

        return false;
    }

    void connectRegions(Region* sourceRegion, Region* targetRegion) {
        if (isConnected(sourceRegion, targetRegion)) {
            return; // Already connected
        }

        const auto edgeId = static_cast<const EdgeId>(edges.size());

        auto edge = new Edge(static_cast<const EdgeId>(edges.size()),
                sourceRegion,
                targetRegion,
                initialAlpha,
                initialBeta);

        edges.push_back(edge);

        sourceRegion->outEdges.push_back(edge);
        targetRegion->inEdges.push_back(edge);
    }

    void disconnectRegion(Region* region) {
        std::vector<Edge*> inEdges = region->inEdges;
        std::vector<Edge*> outEdges = region->outEdges;

        for (auto inEdge : inEdges) {
            disconnectRegions(inEdge->sourceRegion, region);
            assert(inEdge->targetRegion == region);
        }

        for (auto outEdge : outEdges) {
            disconnectRegions(region, outEdge->targetRegion);
            assert(outEdge->sourceRegion == region);
        }
    }

    void disconnectRegions(Region* sourceRegion, Region* targetRegion) {
        Edge* connectingEdge{nullptr};

        auto& sourceOutEdges = sourceRegion->outEdges;

        for (auto outEdge : sourceOutEdges) {
            if (outEdge->targetRegion->id == targetRegion->id) {
                connectingEdge = outEdge;
                break;
            }
        }

        if (connectingEdge == nullptr) {
            // Regions were not connected
            return;
        }

        // Remove edge from source and target
        std::remove(
                sourceOutEdges.begin(), sourceOutEdges.end(), connectingEdge);

        auto& targetInEdges = connectingEdge->targetRegion->inEdges;
        std::remove(targetInEdges.begin(), targetInEdges.end(), connectingEdge);
    }

    void removeEdge(Edge* edge) {
        if (edge != nullptr) {
            auto edgeId = edge->edgeId;
            edges[edgeId] = nullptr;
            delete edge;
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
    void splitRegion(Region* originalRegion) {
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

    double getInteriorEdgeEffort(Edge* edge) {
        const auto numberOfStates = edge->targetRegion->states.size();

        double bestValue = std::numeric_limits<double>::infinity();

        for (auto outEdge : edge->targetRegion->outEdges) {
            double bonusValue = 0;
            switch (bonusType) {
            case 0:
                bonusValue = outEdge->getScottBonusEffort(numberOfStates);
                break;
            case 1:
                bonusValue = outEdge->getBonusEffort(numberOfStates);
                break;
            case 2:
                bonusValue = outEdge->getBonusEffort2(numberOfStates);
                break;
            case 3:
                bonusValue = outEdge->getBonusEffort3(numberOfStates);
                break;
            default:
                break;
            }

            double value = bonusValue + outEdge->targetRegion->g;

            if (value < bestValue) {
                bestValue = value;
            }
        }

        return bestValue + edge->getEffort();
    }

    void addOutgoingEdgesToOpen(const Region* region) {
        for (auto edge : region->outEdges) {
            const Region* targetRegion = edge->targetRegion;

            //
            if (std::isinf(targetRegion->g)) {
                computeShortestPath();
            }

            edge->totalEffort = targetRegion->g + edge->getEffort();

            insertOrUpdateOpen(edge);
        }
    }

    void insertOrUpdateOpen(Edge* edge, bool addToOpen = true) {
        if (!open.inHeap(edge) && addToOpen) {
            open.push(edge);
        } else {
            open.siftFromItem(edge);
        }
    }

    void computeShortestPath() {
        // if (inconsistentRegions.isEmpty()) {
        //    std::cout << "nothing in u" << std::endl;
        //}
        while (!inconsistentRegions.isEmpty()) {
            Region* u = regions[inconsistentRegions.pop()->id];
            auto oldKey = u->key;
            auto newKey = u->calculateKey();

            if (oldKey < newKey) {
                u->key = newKey; // update key
                inconsistentRegions.push(u);
            } else if (u->g > u->rhs) {
                u->g = u->rhs;

                for (auto edge : u->inEdges) {
                    double totalEffort = edge->interior ?
                            getInteriorEdgeEffort(edge) :
                            u->g + edge->getEffort();

                    edge->totalEffort = totalEffort;
                    insertOrUpdateOpen(edge, false);
                    updateRegion(edge->getInEdgeTargetRegion());
                }
            } else {
                u->g = std::numeric_limits<double>::infinity();
                for (auto edge : u->inEdges) {
                    // in the paper, the g here is the old g?
                    double totalEffort = edge->interior ?
                            getInteriorEdgeEffort(edge) :
                            u->g + edge->getEffort();

                    edge->totalEffort = totalEffort;
                    insertOrUpdateOpen(edge, false);
                    updateRegion(edge->getInEdgeTargetRegion());
                }

                // Update this region
                updateRegion(u);
            }
        }
#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
    }

public:
#ifdef STREAM_GRAPH
    void publishAbstractGraph() {
        //        return;
        //        static int counter = -1;
        //        ++counter;
        //        counter %= 1000;
        //        if (counter > 0) {
        //            return;
        //        }

        //        std::cout << "Graph test" << std::endl;
        httplib::Client cli("localhost", 8080, 300, httplib::HttpVersion::v1_1);

        std::ostringstream commandBuilder;

        const unsigned int dimension =
                globalParameters.globalAbstractAppBaseGeometric
                        ->getSpaceInformation()
                        .get()
                        ->getStateDimension();

        for (auto region : regions) {
            //            if (region->getTotalTries() == 0 && region->id > 1) {
            //                continue; // Skip untouched regions
            //            }

            auto vectorState =
                    region->state
                            ->as<ompl::base::CompoundStateSpace::StateType>()
                            ->as<ompl::base::RealVectorStateSpace::StateType>(
                                    0);

            double r = 0;
            double g = 0;
            double b = 0;
            double size = std::max(log(region->getTotalTries()), 2.);

            if (region->id == 0) {
                r = 1;
                size = 50;
            } else if (region->id == 1) {
                g = 1;
                size = 50;
            }

            commandBuilder << "{\"" << (region->alreadyVisualized ? "cn" : "an")
                           << "\":{\"" << region->id << "\":{"
                           << R"("label":"g: )" << region->g << "\""
                           << R"(,"size":)" << size << R"(,"r":)" << r
                           << R"(,"g":)" << g << R"(,"b":)" << b << R"(,"x":)"
                           << vectorState->values[0] * 100 << R"(,"y":)"
                           << vectorState->values[1] * 100 << R"(,"z":)"
                           << (dimension == 3 ? vectorState->values[2] * 100 :
                                                0)
                           << "}}}\r\n";
            region->alreadyVisualized = true;
        }

        for (auto edge : edges) {
            //            if (edge->alpha + edge->beta == 2) {
            //                continue; // Skip untouched edges
            //            }

            commandBuilder << "{\"" << (edge->alreadyVisualized ? "ce" : "ae")
                           << "\":{\"" << edge << "\":{"
                           << R"("source":")" << edge->sourceRegion->id << "\","
                           << R"("target":")" << edge->targetRegion->id << "\","
                           << "\"directed\":true,"
                           << R"("label":"Te: )" << edge->totalEffort << "\","
                           << R"("weight":")" << edge->alpha + edge->beta
                           << "\"}}}\r\n";

            edge->alreadyVisualized = true;
        }
        commandBuilder << std::endl;

        std::string commandString = commandBuilder.str();

        //        std::cout << commandString << std::endl;
        auto res = cli.post("/workspace1?operation=updateGraph",
                commandString,
                "plain/text");

        //        std::cout << res->status << std::endl;
        //        std::cout << res->body << std::endl;
        //        std::cout << "Graph test end" << std::endl;
    }
#endif

    void updateRegion(Region* region) {
        if (region->id != goalRegionId) {
            double minValue = std::numeric_limits<double>::infinity();

            for (auto e : region->outEdges) {
                double value = e->targetRegion->g + e->getEffort();
                if (value < minValue) {
                    minValue = value;
                }
            }

            region->rhs = minValue;
        }

        if (inconsistentRegions.inHeap(region)) {
            inconsistentRegions.remove(region);
        }

        if (region->g != region->rhs) {
            region->key = region->calculateKey();
            inconsistentRegions.push(region);
        }
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

    const double stateRadius;
    const double abstractStateRadius;
    const unsigned int initialRegionCount;
    const double resizeFactor;
    const unsigned int neighborEdgeCount;
    const unsigned int initialAlpha;
    const unsigned int initialBeta;
    const unsigned int bonusType;
    const bool useSplit;
    const bool haltonSampling;

    ompl::base::State* fullStartState;
    ompl::base::State* fullGoalState;
    const ompl::base::State* abstractStartState;
    const ompl::base::State* abstractGoalState;
    std::vector<Region*> regions;
    std::vector<Edge*> edges;
    std::unique_ptr<ompl::NearestNeighbors<Region*>> nearestRegions;

    const ompl::base::SpaceInformation* spaceInformation;
    ompl::base::StateSamplerPtr fullStateSampler;

    ompl::base::StateSpacePtr abstractSpace;
    ompl::base::ValidStateSamplerPtr abstractSampler;

    const ompl::base::GoalSampleableRegion* goalRegionSampler;

    const HaltonSampler haltonSampler;

    InPlaceBinaryHeap<Region, Region> inconsistentRegions;
    InPlaceBinaryHeap<Edge, Edge> open;
    Edge* lastSelectedEdge = nullptr;
    bool targetSuccess = false;
    bool addedGoalEdge = false;

    Timer* goalEdgeTimer;
};
