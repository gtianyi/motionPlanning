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

        vertices[goalID].rhs = 0;
        vertices[goalID].key = calculateKey(goalID);
        U.push(&vertices[goalID]);

        {
            Timer t("D* lite");
            computeShortestPath();
        }

        for (auto eset : edges) {
            for (auto e : eset.second) {
                e.second->initialEffort = e.second->effort;
            }
        }

        vertices[startID].addState(startState);
        addOutgoingEdgesToOpen(startID);
    }

    virtual bool sample(ompl::base::State* from, ompl::base::State* to) {
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
        thompsonSampleInitialization();
        computeShortestPath();

        if (targetEdge != NULL) { //only will fail the first time through

            if (targetSuccess) {
                if (!addedGoalEdge && targetEdge->endID == goalID) {
                    Edge* goalEdge = new Edge(goalID, goalID);
                    goalEdge->updateEdgeStatusKnowledge(Abstraction::Edge::VALID);
                    goalEdge->effort = 1;
                    open.push(goalEdge);
                    addedGoalEdge = true;
                }

                if (targetEdge->interior) {
                    updateSuccesfulInteriorEdgePropagation(targetEdge);
                    updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
                } else {
                    //edge has become interior
                    targetEdge->interior = true;
                    targetEdge->succesfulPropagation();
                    updateEdgeEffort(targetEdge, getInteriorEdgeEffort(targetEdge));
                }
            } else {
                targetEdge->failurePropagation();
                updateEdgeEffort(targetEdge, targetEdge->getEstimatedRequiredSamples() + vertices[targetEdge->endID].g);
            }

            updateVertex(targetEdge->startID);
            computeShortestPath();

            if (targetSuccess) {
                addOutgoingEdgesToOpen(targetEdge->endID);
            }
        }

        bool getNextEdge = true;
        while (getNextEdge) {
            assert(!open.isEmpty());

            targetEdge = open.peek();

            if (targetEdge->status == Abstraction::Edge::UNKNOWN) {
                Abstraction::Edge::CollisionCheckingStatus status =
                    abstraction->isValidEdge(targetEdge->startID, targetEdge->endID) ? Abstraction::Edge::VALID :
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

        if (targetEdge->startID == targetEdge->endID && targetEdge->startID == goalID) {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            goalSampler->sampleGoal(to);
        } else {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            ompl::base::ScopedState<>
                vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
            // guty: need to use globalAppBaseGeometric for linkage
            if (abstraction->supportsSampling()) {
                vertexState = abstraction->sampleAbstractState(targetEdge->endID);
            } else {
                vertexState = abstraction->getState(targetEdge->endID);
                ompl::base::ScopedState<>
                    fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
                fullStateSampler->sampleUniformNear(to, fullState.get(), stateRadius);
                // guty: add a sampler for linkage here
            }
        }
        return true;
    }

    virtual bool sample(ompl::base::State*) {
        throw ompl::Exception("NewSampler::sample", "not implemented");
    }

    virtual bool sampleNear(ompl::base::State*, const ompl::base::State*, const double) {
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
        } else {
            addOutgoingEdgesToOpen(newCellId);
        }
    }


protected:
    void vertexMayBeInconsistent(unsigned int id) {
        updateVertex(id);
    }

    void vertexHasInfiniteValue(unsigned int id) {
        computeShortestPath();
    }


    Key calculateKey(unsigned int id) {
        Vertex& s = vertices[id];
        Key key;
        key.first = std::min(s.g, s.rhs);
        key.second = std::min(s.g, s.rhs);
        return key;
    }

    void updateVertex(unsigned int id) {
        Vertex& s = vertices[id];
        if (s.id != goalID) {
            double minValue = std::numeric_limits<double>::infinity();
            auto neighbors = abstraction->getNeighboringCells(id);
            for (auto n : neighbors) {
                Edge* e = getEdge(id, n);
                double value = vertices[n].g + e->getEstimatedRequiredSamples();
                if (value < minValue) {
                    minValue = value;
                }
            }
            s.rhs = minValue;
        }

        if (U.inHeap(&vertices[id])) {
            U.remove(&vertices[id]);
        }

        if (s.g != s.rhs) {
            s.key = calculateKey(id);
            U.push(&vertices[id]);
        }
    }

    double sampleEffort(const Edge* edge) {
        double a = std::gamma_distribution<double>(edge->alpha, 1.0)(randomNumberGenerator);
        double b = std::gamma_distribution<double>(edge->beta, 1.0)(randomNumberGenerator);
        assert(a != 0.0);
        assert(b != 0.0);
        return (a + b) / a;
    }

    void thompsonSampleInitialization() {
        // Sample effort for all edges 
        for (auto eset : edges) {
            for (auto edgePair : eset.second) {
                Edge* edge = edgePair.second;
                updateEdgeEffort(edge, sampleEffort(edge), false);
            }
        }
    }

    void computeShortestPath() {
        while (!U.isEmpty()) {
            Vertex& currentVertex = vertices[U.pop()->id];
            Key k_old = currentVertex.key;
            Key k_new = calculateKey(currentVertex.id);

            if (k_old < k_new) {
                currentVertex.key = k_new;
                U.push(&vertices[currentVertex.id]);
            } else if (currentVertex.g > currentVertex.rhs) {
                currentVertex.g = currentVertex.rhs;
                for (auto e : reverseEdges[currentVertex.id]) {
                    if (e.second->interior) {
                        updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
                    } else {
                        updateEdgeEffort(e.second, currentVertex.g + e.second->getEstimatedRequiredSamples(), false);
                    }
                }

                auto neighbors = getNeighboringCells(currentVertex.id);
                for (auto n : neighbors) {
                    updateVertex(n);
                }
            } else {
                currentVertex.g = std::numeric_limits<double>::infinity();

                for (auto e : reverseEdges[currentVertex.id]) {
                    if (e.second->interior) {
                        updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
                    } else {
                        updateEdgeEffort(e.second, currentVertex.g, false);
                    }
                }

                updateVertex(currentVertex.id);
                auto neighbors = abstraction->getNeighboringCells(currentVertex.id);
                for (auto n : neighbors) {
                    updateVertex(n);
                }
            }
        }
    }

    bool addedGoalEdge;
    std::mt19937 randomNumberGenerator{1};
};

}

}
