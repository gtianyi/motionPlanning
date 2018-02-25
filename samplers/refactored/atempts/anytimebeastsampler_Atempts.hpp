/**
 * \file anytimebeastsampler_Atemtps.hpp
 *
 * ATEMPTS sampler
 *
 * \author Tianyi Gu
 * \date   11 / 26 / 2017
 */ 
#pragma once

#include <set>

#include "../../abstractions/prmlite.hpp"

#include "abstractvertex_atempts.hpp"
#include "abstractedge_atempts.hpp"
#include "atemptsdijkstra.hpp"
using namespace std;
namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

class AnytimeBeastSampler_Atempts : public ompl::base::UniformValidStateSampler {
    typedef AbstractVertex Vertex;
    typedef AbstractEdge Edge;
  public:
    AnytimeBeastSampler_Atempts(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
                                base::GoalSampleableRegion *gsr, const ompl::base::OptimizationObjectivePtr &optimizationObjective, const FileMap &params) :
            UniformValidStateSampler(base), fullStateSampler(base->allocStateSampler()), optimizationObjective(optimizationObjective), stateRadius(params.doubleVal("StateRadius")), goalSampler(gsr) {

        startState = base->allocState();
        base->copyState(startState, start);

        auto abstractStart = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getStartState(0);
        auto abstractGoal = globalParameters.globalAbstractAppBaseGeometric->getProblemDefinition()->getGoal()->as<ompl::base::GoalState>()->getState();

        abstraction = new PRMLite(base, abstractStart, abstractGoal, params);

        Edge::validEdgeDistributionAlpha = params.doubleVal("ValidEdgeDistributionAlpha");
        Edge::validEdgeDistributionBeta = params.doubleVal("ValidEdgeDistributionBeta");

        Edge::invalidEdgeDistributionAlpha = params.doubleVal("InvalidEdgeDistributionAlpha");
        Edge::invalidEdgeDistributionBeta = params.doubleVal("InvalidEdgeDistributionBeta");
    }

    virtual ~AnytimeBeastSampler_Atempts() {
        delete abstraction;
    }

    void initialize() {
        abstraction->initialize();

        startID = abstraction->getStartIndex();
        goalID = abstraction->getGoalIndex();

        unsigned int abstractionSize = abstraction->getAbstractionSize();

        int totalEdgeCount = 0;
        for(unsigned int i = 0; i < abstractionSize; ++i) {
            shared_ptr<Vertex> v = make_shared<Vertex>(i);
            vertices.push_back(v);
            auto neighbors = abstraction->getNeighboringCells(i);
            for(auto n : neighbors) {
                getEdge(i, n);
                getEdge(n, i);
                totalEdgeCount += 2;
            }
        }
        cout << "Edge count: " << totalEdgeCount << endl;

        atemptsDijkstra = make_shared<AtemptsDijkstra<Vertex, Edge>>(vertices, startID, goalID, abstraction,
                                            [&](unsigned int a, unsigned int b){ return getEdge(a, b); });
        
	atemptsDijkstra->updateCostGByEdge();
        atemptsDijkstra->updatePareto();

        vertices[startID]->addState(startState, 0);
        pushOpen(vertices[startID]);
    }

    bool sample(ompl::base::State *from, ompl::base::State *to, const base::PlannerTerminationCondition &ptc) {
        // cout << "opensize: "<< open.size() << endl;
        cout << "opentop: "<< open.top()->id << endl;
        cout << "opentopPareto: "<< open.top()->undominatePathCount << endl;
        // cout << "epsilonBar: "<< epsilonBar << endl;
        // cout << "topeffort: "<< open.top()->currentEffortToGoal << endl;
        if(targetEdge != nullptr) {
            if (firstTargetSuccessState != nullptr){
                targetSuccess();
            }else{
                targetFailure();
            }
            atemptsDijkstra->updateCostGByEdge();
            atemptsDijkstra->updatePareto();
            resortOpen();
        }

        targetEdge = selectTargetEdge(ptc);
        if(targetEdge == nullptr) {
            return false;
        }

        firstTargetSuccessState = nullptr;

        if(targetEdge->startID == targetEdge->endID &&
           targetEdge->startID == goalID) {
            goalSampler->sampleGoal(to);
            si_->copyState(from, vertices[targetEdge->startID]->
                           sampleStateByDis(si_, to));
        } else {
            ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
            vertexState = abstraction->getState(targetEdge->endID);
            ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
            fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
            si_->copyState(from, vertices[targetEdge->startID]->sampleStateByDis(si_, to));
        }
        return true;
    }

    void reached(ompl::base::State *start, double startG, ompl::base::State *end, double endG) {
        ompl::base::ScopedState<> incomingState(si_->getStateSpace());
        incomingState = start;
        unsigned int startCellId = abstraction->mapToAbstractRegion(incomingState);

        incomingState = end;
        unsigned int endCellId = abstraction->mapToAbstractRegion(incomingState);

        auto endVertex = vertices[endCellId];

        if(endVertex->states.find(end) != endVertex->states.end()) {
            si_->getStateSpace()->printState(end, std::cerr);
            si_->getStateSpace()->printState(*(endVertex->states.find(end)), std::cerr);
        }

        endVertex->addState(end, endG);
        if(endCellId != startCellId) {
            shared_ptr<Edge> e = getEdge(startCellId, endCellId);
            // not sure if it would be negative
            e->updateEdgeCost(endG - startG);
            updateEpsilonBar(e);
        }

        //if the planner chose the goal region first be careful not to dereference a null pointer
        if(targetEdge != NULL && endCellId == targetEdge->endID) {
            //this region will be added to open when sample is called again
            if(firstTargetSuccessState == nullptr) {
                firstTargetSuccessState = end;
            }
        }
        pushOpen(vertices[endCellId]);
    }

    void remove(ompl::base::State *state, double g) {
        ompl::base::ScopedState<> incomingState(si_->getStateSpace());
        incomingState = state;
        unsigned int cellId = abstraction->mapToAbstractRegion(incomingState);

        auto vertex = vertices[cellId];

        ompl::base::State *copy = si_->allocState();
        si_->copyState(copy, state);
        for(auto v : vertex->removedStates) {
            if(si_->equalStates(v, state)) {
                fprintf(stderr, "ALREADY REMOVED THIS STATE!!\n");
                assert(false);
            }
        }

        vertex->removedStates.emplace_back(copy);
        vertex->removeState(state, g);

        if(vertex->states.size() == 0) {
            for(auto e : reverseEdges[cellId]) {
                e.second->interior = false;
            }
        }
    }

    void foundSolution(const ompl::base::Cost &incumbent) {
        incumbentCost = incumbent.value();
        targetEdge = NULL;
        // addedGoalEdge = false;
    }

protected:
    shared_ptr<Edge> getEdge(unsigned int a, unsigned int b) {
        shared_ptr<Edge> e = edges[a][b];
        if(e == NULL) {
            e = make_shared<Edge>(a, b);
            if(a == goalID && b == goalID){
                e->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
                e->effort = 1;
            }
            else{
                abstraction->isValidEdgeUnchecked(a, b);
                e->updateEdgeStatusKnowledge(
                    abstraction->getCollisionCheckStatusUnchecked(a, b));
            }
            e->setAbstractCost(abstraction->abstractDistanceFunctionByIndex(a, b));
            edges[a][b] = e;
            reverseEdges[b][a] = e;
        }
        return e;
    }

    void pushOpen(shared_ptr<Vertex> &v){
        if(openSetForCheckDuplicate.find(v->id) ==
           openSetForCheckDuplicate.end()){
            open.push(v);
            openSetForCheckDuplicate.insert(v->id);
        }
    }    

    void resortOpen(){
        std::make_heap(const_cast<shared_ptr<Vertex>*>( &open.top()),
                       const_cast<shared_ptr<Vertex>*>( &open.top() + open.size()),
                       Vertex::ComparatorByEffortToGoal());
    }

    shared_ptr<Edge> selectTargetEdge(const base::PlannerTerminationCondition &ptc) {
        if(ptc != false) {
            return nullptr;
        }
        shared_ptr<Vertex> curVertex = open.top();
        if(curVertex->id == goalID){
            shared_ptr<Edge> e = getEdge(goalID, goalID);
            // shared_ptr<Edge> e = edges[goalID][goalID];
            // if( e == NULL){
            //     e = make_shared<Edge>(goalID, goalID);
            //     e->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);

            //     e->effort = 1;
            //     // addedGoalEdge = true;
            //     edges[goalID][goalID] = e;
            //     reverseEdges[goalID][goalID] = e;
            // }
            return e;
        }
        int edgeStart = curVertex->bestPath->id;
        int edgeEnd  =  curVertex->bestPath->parent->id;
        shared_ptr<Edge> e = edges[edgeStart][edgeEnd];
        return e;
    }

    void targetSuccess() {
        // if(targetEdge->interior) {
        //     updateSuccesfulInteriorEdgePropagation(targetEdge);
        //     updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
        // } else {
        //     //edge has become interior
        //     targetEdge->interior = true;
        //     targetEdge->succesfulPropagation();

        //     updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
        // }
        targetEdge->succesfulPropagation();
    }

    // void updateSuccesfulInteriorEdgePropagation(shared_ptr<Edge> edge) {
    //     double numberOfStates = vertices[edge->endID]->states.size();
    //     shared_ptr<Edge> e = getEdge(edge->endID, edge->interiorToNextEdgeID);
    //     e->rewardHypotheticalSamplesAfterPositivePropagation(numberOfStates);
    // }

    // double getInteriorEdgeEffort(shared_ptr<Edge> edge) {
    //     double mySamples = edge->getEstimatedRequiredSamples();
    //     double numberOfStates = vertices[edge->endID]->states.size();

    //     double bestValue = std::numeric_limits<double>::infinity();
    //     auto neighbors = abstraction->getNeighboringCells(edge->endID);
    //     for(auto n : neighbors) {
    //         if(n == edge->startID)
    //             continue;
    //         shared_ptr<Edge> e = getEdge(edge->endID, n);
    //         double value = mySamples +
    //                 e->getHypotheticalEstimatedSamplesAfterPositivePropagation(
    //                     numberOfStates);

    //         if(value < bestValue) {
    //             bestValue = value;
    //             edge->interiorToNextEdgeID = n;
    //         }
    //     }
    //     return bestValue;
    // }

    // void updateEdgeEffort(shared_ptr<Edge> e, double effort) {
    //     assert(effort >= 0);
    //     e->effort = effort;
    // }

    void targetFailure() {
        targetEdge->failurePropagation();
        // here we just update effort without interior thing,  could be a problem
    }

    void updateEpsilonBar(shared_ptr<Edge> e){
        int startCellId = e->startID;
        int endCellId = e->endID;
        if(edgeEpsilonPositions.find(startCellId) == edgeEpsilonPositions.end() ||
           edgeEpsilonPositions[startCellId].find(endCellId) == edgeEpsilonPositions[startCellId].end()){
            edgeEpsilonPositions[startCellId][endCellId] = edgeEpsilons.size();
            epsilonBar =( epsilonBar * edgeEpsilons.size() + e->epsilon ) / (double)(edgeEpsilons.size() + 1);
            edgeEpsilons.push_back(e->epsilon);
        }
        else{
            epsilonBar =  ( epsilonBar * edgeEpsilons.size() - edgeEpsilons[edgeEpsilonPositions[startCellId][endCellId]]
                            + e->epsilon ) / (double) edgeEpsilons.size();
            edgeEpsilons[edgeEpsilonPositions[startCellId][endCellId]] = e->epsilon;
        }
        atemptsDijkstra->updateEpsilonBar(epsilonBar);
    }
    
    bool refineAbstract() {
        // through abstract graph and double the sample,
        // rebuild the abstract,  map the state to new vetices
        return true;
    }
    
    Abstraction *abstraction = nullptr;
    unsigned int startID, goalID;
    StateSamplerPtr fullStateSampler;
    base::GoalSampleableRegion *goalSampler;

    ompl::base::State *startState = nullptr;

    ompl::RNG randomNumbers;

    shared_ptr<Edge> targetEdge = nullptr;
    ompl::base::State *firstTargetSuccessState = nullptr;
    bool addedGoalEdge = false;

    double stateRadius;

    double incumbentCost = std::numeric_limits<double>::infinity();
    const ompl::base::OptimizationObjectivePtr &optimizationObjective;

    std::vector<shared_ptr<Vertex>> vertices;
    std::unordered_map<unsigned int, std::unordered_map<unsigned int, shared_ptr<Edge>>> edges;
    std::unordered_map<unsigned int, std::unordered_map<unsigned int, shared_ptr<Edge>>> reverseEdges;

    // open list contains all verices touched by the motion tree
    // Sorted by EffortToGoal
    std::priority_queue<shared_ptr<Vertex>, vector<shared_ptr<Vertex>>, Vertex::ComparatorByEffortToGoal> open;
    std::unordered_set<int> openSetForCheckDuplicate;
   
    shared_ptr<AtemptsDijkstra<Vertex, Edge>> atemptsDijkstra;

    std::unordered_map<unsigned int, std::unordered_map<unsigned int, int>> edgeEpsilonPositions;
    vector<double> edgeEpsilons;
    double epsilonBar = 1;
};

}

}

}

}
