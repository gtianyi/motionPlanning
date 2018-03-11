#pragma once

#include "beastsamplerbase.hpp"
#include <vector>

namespace ompl {

namespace base {

class BeastSampler_dstarUpdateRightEdge : public ompl::base::BeastSamplerBase {
  public:
    BeastSampler_dstarUpdateRightEdge(ompl::base::SpaceInformation *base,
                                      ompl::base::State *start,
                                      const ompl::base::GoalPtr &goal,
                                      ompl::base::GoalSampleableRegion *gsr,
                                      const FileMap &params) :
            BeastSamplerBase(base, start, goal, gsr, params),
            addedGoalEdge(false) {}

    ~BeastSampler_dstarUpdateRightEdge() {}

    virtual void initialize() {
        BeastSamplerBase::initialize();

        unsigned int abstractionSize = abstraction->getAbstractionSize();
        vertices.reserve(abstractionSize);

        for(unsigned int i = 0; i < abstractionSize; ++i) {
            vertices.emplace_back(i);
            auto neighbors = abstraction->getNeighboringCells(i);
            for(auto n : neighbors) {
                getEdge(i, n);
                getEdge(n, i);
            }
        }

        vertices[goalID].rhs = 0;
        vertices[goalID].key = calculateKey(goalID);
        U.push(&vertices[goalID]);

        {
            Timer t("D* lite");
            computeShortestPath();
        }

        for(auto eset : edges) {
            for(auto e : eset.second) {
                e.second->initialEffort = e.second->effort;
            }
        }

        vertices[startID].addState(startState);
        addOutgoingEdgesToOpen(startID);
    }

    virtual bool sample(ompl::base::State *from, ompl::base::State *to) {
#ifdef STREAM_GRAPHICS
        static unsigned int sampleCount = 0;
        if(sampleCount++ % 100 == 0) {
            // fprintf(stderr, "open: %u\n", open.getFill());
            // writeVertexFile(sampleCount / 1000);
            // writeOpenEdgeFile(sampleCount / 1000);
            // writeUpdatedEdgeFile(sampleCount / 10);
            writeEdgeFile(sampleCount / 100, 1);
        }
#endif

        if(targetEdge != NULL) { //only will fail the first time through

           for (const auto &e : edgesNeedUpdate){
                if(!addedGoalEdge && e->endID == goalID) {
                    Edge *goalEdge = new Edge(goalID, goalID);
                    goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
                    goalEdge->effort = 1;
                    open.push(goalEdge);
                    addedGoalEdge = true;
                }

                if(e->interior) {
                    updateSuccesfulInteriorEdgePropagation(e);
                    updateEdgeEffort(e, getInteriorEdgeEffort(e));
                } else {
                    //edge has become interior
                    e->interior = true;
                    e->succesfulPropagation();
                    updateEdgeEffort(e, getInteriorEdgeEffort(e));
                }
                // might be problems here
                // because these vertices might need to be updated in order ? 
                updateVertex(e->startID); 
           }
               
           if( !targetSuccess ){
               realTargetEdge->failurePropagation();
               updateEdgeEffort(realTargetEdge,
                                realTargetEdge->getEstimatedRequiredSamples()      
                                + vertices[realTargetEdge->endID].g);
           }
           
           computeShortestPath();

           for (const auto &e : edgesNeedUpdate){
                addOutgoingEdgesToOpen(e->endID);
           }
        }

        bool getNextEdge = true;
        while(getNextEdge) {
            assert(!open.isEmpty());

            targetEdge = open.peek();

            if(targetEdge->status == Abstraction::Edge::UNKNOWN) {
                Abstraction::Edge::CollisionCheckingStatus status = abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
                        Abstraction::Edge::INVALID;
                targetEdge->updateEdgeStatusKnowledge(status);
				
                //yes this looks weird but we need it for right now to do some debugging
                updateEdgeEffort(targetEdge, targetEdge->effort);

                updateVertex(targetEdge->startID);
                computeShortestPath();
            } else {
                getNextEdge = false;
            }
        }

        targetSuccess = false;

        if(targetEdge->startID == targetEdge->endID && targetEdge->startID == goalID) {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            goalSampler->sampleGoal(to);
        } else {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
            // guty: need to use globalAppBaseGeometric for linkage
            if(abstraction->supportsSampling()) {
                vertexState = abstraction->sampleAbstractState(targetEdge->endID);
            } else {
                vertexState = abstraction->getState(targetEdge->endID);
                ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
                fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
                // guty: add a sampler for linkage here
            }
        }

        realTargetEdge = NULL;
        prevRegion = -1;
        currRegion = -1;
        edgesNeedUpdate.clear();
        return true;
    }

    virtual bool sample(ompl::base::State *) {
        throw ompl::Exception("NewSampler::sample", "not implemented");
        return false;
    }

    virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
        throw ompl::Exception("NewSampler::sampleNear", "not implemented");
        return false;
    }

    void reached(ompl::base::State *state){
        return;
    }

    void reachedFromTree(ompl::base::State *state_from,
                         ompl::base::State *state_to) {
        ompl::base::ScopedState<> incomingState_from(si_->getStateSpace());
        ompl::base::ScopedState<> incomingState_to(si_->getStateSpace());
        incomingState_from = state_from;
        incomingState_to = state_to;
        unsigned int CellId_from = abstraction->
                mapToAbstractRegion(incomingState_from);
        unsigned int CellId_to = abstraction->mapToAbstractRegion(incomingState_to);
        
        vertices[CellId_to].addState(state_to);
        if(currRegion != CellId_to){
            prevRegion = currRegion == -1 ? CellId_from : currRegion;
            currRegion = CellId_to;
            Edge *e = getEdge(prevRegion, currRegion);
            if(realTargetEdge != NULL && CellId_to == realTargetEdge->endID){
                targetSuccess = true;
            }
            else{
                edgesNeedUpdate.push_back(e);
                addOutgoingEdgesToOpen(CellId_to);
            }
        }
    }

    void updateRealTargeEdge(ompl::base::State *state_from,
                             ompl::base::State *state_to){
        ompl::base::ScopedState<> incomingState_from(si_->getStateSpace());
        ompl::base::ScopedState<> incomingState_to(si_->getStateSpace());
        incomingState_from = state_from;
        incomingState_to = state_to;
        unsigned int CellId_from = abstraction->
                mapToAbstractRegion(incomingState_from);
        unsigned int CellId_to = abstraction->mapToAbstractRegion(incomingState_to);
        realTargetEdge = getEdge(CellId_from, CellId_to);
    }


  protected:
    void vertexMayBeInconsistent(unsigned int id) {
        updateVertex(id);
    }

    void vertexHasInfiniteValue(unsigned int id) {
        computeShortestPath();
    }


    Key calculateKey(unsigned int id) {
        Vertex &s = vertices[id];
        Key key;
        key.first = std::min(s.g, s.rhs);
        key.second = std::min(s.g, s.rhs);
        return key;
    }

    void updateVertex(unsigned int id) {
        Vertex &s = vertices[id];
        if(s.id != goalID) {
            double minValue = std::numeric_limits<double>::infinity();
            auto neighbors = abstraction->getNeighboringCells(id);
            for(auto n : neighbors) {
                Edge *e = getEdge(id, n);
                double value = vertices[n].g + e->getEstimatedRequiredSamples();
                if(value < minValue) {
                    minValue = value;
                }
            }
            s.rhs = minValue;
        }

        if(U.inHeap(&vertices[id])) {
            U.remove(&vertices[id]);
        }

        if(s.g != s.rhs) {
            s.key = calculateKey(id);
            U.push(&vertices[id]);
        }
    }

    void computeShortestPath() {
        while(!U.isEmpty()) {
            Vertex &u = vertices[U.pop()->id];
            Key k_old = u.key;
            Key k_new = calculateKey(u.id);

            if(k_old < k_new) {
                u.key = k_new;
                U.push(&vertices[u.id]);
            }
            else if(u.g > u.rhs) {
                u.g = u.rhs;
                for(auto e : reverseEdges[u.id]) {
                    if(e.second->interior) {
                        updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
                    }
                    else {
                        updateEdgeEffort(e.second, u.g + e.second->getEstimatedRequiredSamples(), false);
                    }
                }

                auto neighbors = getNeighboringCells(u.id);
                for(auto n : neighbors) {
                    updateVertex(n);
                }
            } else {
                u.g = std::numeric_limits<double>::infinity();

                for(auto e : reverseEdges[u.id]) {
                    if(e.second->interior) {
                        updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
                    }
                    else {
                        updateEdgeEffort(e.second, u.g, false);
                    }
                }

                updateVertex(u.id);
                auto neighbors = abstraction->getNeighboringCells(u.id);
                for(auto n : neighbors) {
                    updateVertex(n);
                }
            }
        }
    }

    bool addedGoalEdge;
    int currRegion = -1;
    int prevRegion = -1;
    std::vector<Edge*> edgesNeedUpdate;
    Edge *realTargetEdge = NULL;
};

}

}
