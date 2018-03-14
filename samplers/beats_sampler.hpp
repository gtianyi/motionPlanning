#pragma once

#include "beastsamplerbase.hpp"
#include <boost/math/distributions.hpp>

namespace ompl {

namespace base {

class BeatsSampler: public ompl::base::BeastSamplerBase {
    struct VertexWrapper {
        VertexWrapper(Vertex* vertex)
                : vertex(vertex),
                  currentParent(NULL) {}

        virtual ~VertexWrapper() {
            for (auto p : parents) {
                delete p;
            }
        }

        inline unsigned int getId() const {
            return vertex->id;
        }

        virtual double getVal() const {
            return vertex->g;
        }
        virtual void setVal(double val) {
            vertex->g = val;
        }
        static bool pred(const VertexWrapper* a, const VertexWrapper* b) {
            return a->vertex->g < b->vertex->g;
        }
        static unsigned int getHeapIndex(const VertexWrapper* r) {
            return r->heapIndex;
        }
        static void setHeapIndex(VertexWrapper* r, unsigned int i) {
            r->heapIndex = i;
        }

        Vertex* vertex;
        unsigned int heapIndex;

        struct Parent {
            Parent(unsigned int parent, double cost)
                    : parent(parent),
                      cost(cost) {}
            static bool HeapCompare(const Parent* r1, const Parent* r2) {
                return r1->cost < r2->cost;
            }
            unsigned int parent;
            double cost;
        };

        virtual bool addParent(unsigned int parent, double cost) {
            if (currentParent == NULL) {
                currentParent = new Parent(parent, cost);
                setVal(currentParent->cost);
                return true;
            } else {
                if (cost < currentParent->cost) {
                    parents.push_back(currentParent);
                    std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
                    currentParent = new Parent(parent, cost);
                    setVal(currentParent->cost);
                    return true;
                } else {
                    parents.push_back(new Parent(parent, cost));
                    std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
                    return false;
                }
            }
        }

        bool hasMoreParents() const {
            return !parents.empty() || currentParent != NULL;
        }

        unsigned int getBestParentIndex() const {
            assert(currentParent != NULL);
            return currentParent->parent;
        }

        void popBestParent() {
            assert(currentParent != NULL);

            delete currentParent;

            if (parents.size() == 0) {
                currentParent = NULL;
                setVal(std::numeric_limits<double>::infinity());
            } else {
                currentParent = parents.front();
                std::pop_heap(parents.begin(), parents.end(), Parent::HeapCompare);
                parents.pop_back();
                setVal(currentParent->cost);
            }
        }

        std::vector<Parent*> parents;
        Parent* currentParent;
    };
  public:
    BeatsSampler(ompl::base::SpaceInformation* base, ompl::base::State* start, const ompl::base::GoalPtr& goal,
                 base::GoalSampleableRegion* gsr, const FileMap& params) : BeastSamplerBase(base,
                                                                                            start,
                                                                                            goal,
                                                                                            gsr,
                                                                                            params),
                                                                           addedGoalEdge(false) {}

    ~BeatsSampler() {}

    virtual void initialize() {
        BeastSamplerBase::initialize();

        unsigned int abstractionSize = abstraction->getAbstractionSize();
        vertices.reserve(abstractionSize);

        for (unsigned int i = 0; i < abstractionSize; ++i) {
            vertices.emplace_back(i);
            auto neighbors = abstraction->getNeighboringCells(i);
            for (auto n : neighbors) {
                getEdge(i, n);
                getEdge(n, i);
            }
        }

        {
            Timer t("ThompsonSampling - beast");
            computeShortestPathWithThompsonSampling();
        }

        for (auto eset : edges) {
            for (auto e : eset.second) {
                e.second->initialEffort = e.second->effort;
            }
        }

        vertices[startID].addState(startState);
        addOutgoingEdgesToOpenTS(startID);
    }

    virtual bool sample(ompl::base::State* from, ompl::base::State* to) {
#ifdef STREAM_GRAPHICS
        static unsigned int sampleCount = 0;
        if(sampleCount++ % 100 == 0) {
            writeEdgeFile(sampleCount / 100, 1);
        }
#endif
        if (targetEdge != NULL) {
            if (targetSuccess) {
                if (!addedGoalEdge && targetEdge->endID == goalID) {
                    Edge* goalEdge = new Edge(goalID, goalID);
                    goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
                    goalEdge->effort = 1;
                    open.push(goalEdge);
                    addedGoalEdge = true;
                }
                targetEdge->succesfulPropagation();
                // if (targetEdge->interior) {
                //     updateSuccesfulInteriorEdgePropagation(targetEdge);
                //     updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
                // } else {
                //     //edge has become interior
                //     targetEdge->interior = true;
                //     targetEdge->succesfulPropagation();
                //     updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
                // }
            }
            else {
                targetEdge->failurePropagation();
                // updateEdgeEffort(targetEdge,
                //                  targetEdge->getEstimatedRequiredSamples()
                //                  + vertices[targetEdge->endID].g);
            }

            computeShortestPathWithThompsonSampling();

            if (targetSuccess) {
                addOutgoingEdgesToOpenTS(targetEdge->endID);
            }
        }
        
        bool getNextEdge = true;
        while (getNextEdge) {
            assert(!open.isEmpty());

            targetEdge = open.peek();

            if (targetEdge->status == Abstraction::Edge::UNKNOWN) {
                Abstraction::Edge::CollisionCheckingStatus status =
                        abstraction->isValidEdge(targetEdge->startID,
                                                 targetEdge->endID) ?
                        Abstraction::Edge::VALID :
                        Abstraction::Edge::INVALID;
                targetEdge->updateEdgeStatusKnowledge(status);
                computeShortestPathWithThompsonSampling();
            } else {
                getNextEdge = false;
            }
        }

        targetSuccess = false;
        // guty: we are here trying to see stuck where,  need better visualization
        // std::cout << "edge:" <<targetEdge->effort << std::endl;
        if (targetEdge->startID == targetEdge->endID &&
            targetEdge->startID == goalID) {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            goalSampler->sampleGoal(to);
        }
        else {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            ompl::base::ScopedState<>vertexState(
                globalParameters.globalAppBaseControl->
                getGeometricComponentStateSpace());
           
            vertexState = abstraction->getState(targetEdge->endID);
            ompl::base::ScopedState<> fullState =
                    globalParameters.globalAppBaseControl->
                    getFullStateFromGeometricComponent(vertexState);
            fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
        }
        return true;
    }

    virtual bool sample(ompl::base::State*) {
        throw ompl::Exception("NewSampler::sample", "not implemented");
    }

    virtual bool sampleNear(ompl::base::State*,
                            const ompl::base::State*,
                            const double) {
        throw ompl::Exception("NewSampler::sampleNear", "not implemented");
    }

    void reached(ompl::base::State* state) {
        ompl::base::ScopedState<> incomingState(si_->getStateSpace());
        incomingState = state;
        unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);

        vertices[newCellId].addState(state);

        //if the planner chose the goal region first be careful not to dereference a null pointer
        if (targetEdge != NULL && newCellId == targetEdge->endID) {
            //this region will be added to open when sample is called again
            targetSuccess = true;
        }
        // else {
        //     addOutgoingEdgesToOpen(newCellId);
        // }
    }

    void reachedFromState(ompl::base::State* from,
                          ompl::base::State* to){
        return;
    }


  protected:
    double sampleEffort(const Edge* edge) {
        double a = std::gamma_distribution<double>(edge->alpha, 1.0)(randomNumberGenerator);
        double b = std::gamma_distribution<double>(edge->beta, 1.0)(randomNumberGenerator);
        assert(a != 0.0);
        assert(b != 0.0); 
        return (a + b) / a;
    }

    void computeShortestPathWithThompsonSampling() {
        std::vector<VertexWrapper *>wrappers;
        wrappers.reserve(vertices.size());
        for (auto &v : vertices){
            wrappers.emplace_back(new VertexWrapper(&v));
        }
        InPlaceBinaryHeap<VertexWrapper, VertexWrapper> openList;
        int closed[vertices.size()] = {0};
        wrappers[goalID]->setVal(0);
        openList.push(wrappers[goalID]);
        int count = 0;
        while( !openList.isEmpty()){
            VertexWrapper * current = openList.pop();
            closed[current->getId()] = 1;
            std::vector<unsigned int> kids =
                    abstraction->getNeighboringCells(current->getId());
            for (auto & kid: kids){
                Edge *e = getEdge(kid, current->getId());
                double effort = current->getVal() + sampleEffort(e);
                if(closed[kid] == 0){
                    wrappers[kid]->setVal(effort);
                    openList.push(wrappers[kid]);
                    e->effort = effort;
                }
            }
        }
        for (auto v : wrappers){
            delete v;
        }
    }

    void vertexMayBeInconsistent(unsigned int id){
        computeShortestPathWithThompsonSampling();
    }

    void vertexHasInfiniteValue(unsigned int id){
        computeShortestPathWithThompsonSampling();
    }

    bool addedGoalEdge;
    std::mt19937 randomNumberGenerator{1};
};

}

}
