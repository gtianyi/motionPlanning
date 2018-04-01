#pragma once
//#define STREAM_GRAPHICS

#include <unordered_set>
#include <unordered_map>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/GenericParam.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include "../domains/geometry/detail/FCLStateValidityChecker.hpp"

#include "../structs/probabilitydensityfunction.hpp"
#include "../structs/inplacebinaryheap.hpp"

namespace ompl {
namespace base {
class guststatesampler : public AbstractionBasedSampler {

  protected:
    struct Vertex {
        Vertex(unsigned int id) : heapIndex(std::numeric_limits<unsigned int>::max()), id(id),num_spliting(0),
                                  numSelections(0), heuristic(std::numeric_limits<double>::infinity()), weight(0), onOpen(false),
                                  currentParent(NULL) {}

        ~Vertex() {
            for(auto p : parents) {
                delete p;
            }
        }

        void selected(double alpha,double delta , double beta, double hmax) {

            /* New Heuristic calculation */
            numSelections++;
            double hnorm;
            hnorm = delta+(1-delta)*(1- heuristic / hmax);
            weight = pow(1, numSelections) * pow (hnorm , alpha);
        }

        int64_t getRandomRegionAlongPathToGoal(ompl::RNG &randomNumbers) const {
            unsigned int randomIndex = (unsigned int)(randomNumbers.uniform01() * regionPath.size());
            //if(regionPath.size() == 0) {
                //fprintf(stderr, "about to fail\n");
                // return 0;
            //}
            return regionPath[randomIndex];
        }

        unsigned int heapIndex;


        static bool pred(const Vertex *a, const Vertex *b) {
            return a->heuristic < b->heuristic;
        }
        static unsigned int getHeapIndex(const Vertex *r) {
            return r->heapIndex;
        }
        static void setHeapIndex(Vertex *r, unsigned int i) {
            r->heapIndex = i;
        }
        static bool HeapCompare(const Vertex *r1, const Vertex *r2) {
            return r1->weight < r2->weight;
        }


        unsigned int id, numSelections;
        int num_spliting;
        double heuristic, weight;
        std::vector<unsigned int> regionPath;
        ompl::base::State *state;
        bool onOpen;


        struct Parent {
            Parent(unsigned int parent, double cost, const std::vector<unsigned int> &path) : parent(parent), cost(cost), path(path) {}
            static bool HeapCompare(const Parent *r1, const Parent *r2) {
                return r1->cost < r2->cost;   
            }
            unsigned int parent;
            double cost;
            std::vector<unsigned int> path;
        };

        bool addParent(unsigned int parent, double cost, const std::vector<unsigned int> &path) {
            Parent *newParent = new Parent(parent, cost, path);
            newParent->path.emplace_back(id);

            if(currentParent == NULL) {
                currentParent = newParent;
                regionPath = currentParent->path;
                heuristic = currentParent->cost;
                return true;
            } else {
                if(cost < currentParent->cost) {
                    parents.push_back(currentParent);
                    std::push_heap(parents.begin(), parents.end(), Parent::HeapCompare);
                    currentParent = newParent;
                    heuristic = currentParent->cost;
                    regionPath = currentParent->path;
                    return true;
                } else {
                    parents.push_back(newParent);
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

            if(parents.size() == 0) {
                currentParent = NULL;
                heuristic = std::numeric_limits<double>::infinity();
                regionPath.clear();
            } else {
                currentParent = parents.front();
                std::pop_heap(parents.begin(), parents.end(), Parent::HeapCompare);
                parents.pop_back();
                heuristic = currentParent->cost;  
                regionPath = currentParent->path;
            }
        }

        std::vector<Parent *> parents;
        Parent *currentParent;
    };
    void dijkstra(Vertex *start) {
        Timer t("dijkstra");
        InPlaceBinaryHeap<Vertex, Vertex> open;
        std::unordered_set<unsigned int> closed;
        start->heuristic = 0;
        start->regionPath.emplace_back(start->id);
        open.push(start);

        closed.insert(start->id);

        while(!open.isEmpty()) {
            Vertex *current = open.pop();

            if(current->id != start->id) {
                unsigned int parentIndex = current->getBestParentIndex();
                if(!abstraction->isValidEdge(parentIndex, current->id)) {
                    //this will update the value of the vertex if needed
                    current->popBestParent();
                    if(current->hasMoreParents()) {
                        open.push(current);
                    }
                    continue;
                }
            }

            closed.insert(current->id);

            if(closed.size() == vertices.size()) break;

            std::vector<unsigned int> kids = getNeighboringCells(current->id);
            for(unsigned int kidIndex : kids) {
                if(closed.find(kidIndex) != closed.end()) continue;

                double newValue = current->heuristic + abstraction->abstractDistanceFunctionByIndex(current->id, kidIndex);

                Vertex *kid = vertices[kidIndex];

                //this will update the value of the vertex if needed
                bool addedBetterParent = kid->addParent(current->id, newValue, current->regionPath);

                if(open.inHeap(kid)) {
                    if(addedBetterParent) {
                        open.siftFromItem(kid);
                    }
                } else {
                    open.push(kid);
                }
            }
        }
    }

    std::vector<Vertex*> vertices;

    std::vector<Vertex*> regionHeap;
    ompl::RNG randomNumbers;

    ompl::base::State* start;
    unsigned int startRegionId, goalRegionId,prmSize;
    Vertex* activeRegion;
    Vertex* selectedRegion;


    double alpha,b,delta,beta;
    int max_no_reg;
    unsigned int usesplit;

  public:
    unsigned int factor;
    double hmax=0;
    int current_region_id=-1;
    guststatesampler(ompl::base::SpaceInformation *base, ompl::base::State *start_, const ompl::base::GoalPtr &goal, const FileMap &params)
            : AbstractionBasedSampler(base, start_, goal, params), alpha(params.doubleVal("Alpha")), b(params.doubleVal("B")),
              beta(params.doubleVal("Beta")), delta(params.doubleVal("Delta")),prmSize(params.integerVal("PRMSize")),
              usesplit(params.integerVal("UseSplit")), activeRegion(NULL), start(base->getStateSpace()->allocState())
    {

        max_no_reg = params.exists("MaxRegion") ? (params.integerVal("MaxRegion")) : vertices.max_size() ;

        base->getStateSpace()->copyState(start, start_);
    }

    virtual ~guststatesampler() {}

    virtual int get_region_size ()
    {
        return vertices.size();
    }

    virtual void initialize()
    {
        Timer timer("Abstraction Computation");
        abstraction->initialize();
        unsigned int abstractionSize = abstraction->getAbstractionSize();
        vertices.clear();

        //factor=1000000;
		//vertices.reserve(abstractionSize);

        for(unsigned int i = 0; i < abstractionSize; ++i) {
				Vertex* v= new Vertex(i); 
            vertices.push_back(v);
        }

        startRegionId = abstraction->getStartIndex();
        goalRegionId = abstraction->getGoalIndex();
        dijkstra(vertices[goalRegionId]);

        //the connectivity check being done on abstraction initialization should assure this
        assert(!std::isinf(vertices[startRegionId]->heuristic));

        for (int j = 0; j < abstractionSize; ++j)
        {
            if (vertices[j]->heuristic==INFINITY)
                continue;
            if (vertices[j]->heuristic>hmax)
                hmax =vertices[j]->heuristic;
        }


#ifdef STREAM_GRAPHICS
        generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].heuristic; }, "gustmap.py");
        ompl::base::ScopedState<> incomingState(si_->getStateSpace());
        incomingState = start;
        unsigned int startID = abstraction->mapToAbstractRegion(incomingState);
        generatePythonPlotting([&](unsigned int vertex) { return vertices[vertex].heuristic; }, vertices[startID].regionPath, "gustpath.py");
#endif

        reached(start);
    }
    virtual bool sample(ompl::base::State *state) {
        if(activeRegion != NULL) {
            if(!activeRegion->onOpen) {
                regionHeap.push_back(activeRegion);
                std::push_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);
                activeRegion->onOpen = true;
            }
        }

        if(randomNumbers.uniform01() < b) {

            assert(!regionHeap.empty());


            activeRegion = regionHeap.front();
            std::pop_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);
            regionHeap.pop_back();
            activeRegion->selected(alpha,delta ,beta,hmax);
            activeRegion->onOpen = false;
            current_region_id=activeRegion->id;


            ompl::base::ScopedState<> vertexState(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
            int64_t regionAlongPath = activeRegion->getRandomRegionAlongPathToGoal(randomNumbers);

            vertexState = abstraction->getState(regionAlongPath);

            ompl::base::ScopedState<> fullState = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState);
            fullStateSampler->sampleUniformNear(state, fullState.get(), stateRadius);

            return true;
        }
        else
        {
            current_region_id=-1;
            fullStateSampler->sampleUniform(state);
            return true;
        }
    }


    virtual void split_regions(int original_region_id ) {
        // add by tianyi
        PRMGust* abstractionGust = static_cast<PRMGust*>(abstraction); 
        // check for splitting or not
        if(!usesplit)
            return;
        // check for the max number of splitting per region
        if ( vertices[original_region_id]->num_spliting >=max_no_reg)
            return;
        // check for max memory size
        if (vertices.size()==prmSize*factor)
            return;

        vertices[original_region_id]->num_spliting++;

        // old abstract size before splitting
        unsigned int oldabstractionSize = abstractionGust->getAbstractionSize();

        // get the new region sample state
        abstractionGust->split_regions(original_region_id);

        // new abstract size after  splitting
        unsigned int abstractionSize = abstractionGust->getAbstractionSize();

		//vertices.resize(abstractionSize);

        // adding new region to the vector
        for(unsigned int i = oldabstractionSize; i < abstractionSize; ++i) {
            Vertex* v = new Vertex(i);
            vertices.push_back(v);
        }

        // copying all the region parameter from the old one to the new

        vertices[oldabstractionSize]->weight=vertices[original_region_id]->weight;

        vertices[oldabstractionSize]->heuristic=vertices[original_region_id]->heuristic;

        for (int j = 0; j < vertices[original_region_id]->regionPath.size(); ++j) {
            vertices[oldabstractionSize]->regionPath.emplace_back(
                    vertices[original_region_id]->regionPath[j]);
        }

    }

    virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
        throw ompl::Exception("guststatesampler::sampleNear", "not implemented");
        return false;
    }

    void reached(ompl::base::State *state) {
        ompl::base::ScopedState<> incomingState(si_->getStateSpace());
        incomingState = state;

        unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);
        if(!vertices[newCellId]->onOpen) {
            vertices[newCellId]->selected(alpha,delta ,beta,hmax);

            regionHeap.push_back(vertices[newCellId]);

            std::push_heap(regionHeap.begin(), regionHeap.end(), Vertex::HeapCompare);

            vertices[newCellId]->onOpen = true;
        }
    }

};

}

}
