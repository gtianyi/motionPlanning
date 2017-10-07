/**
 * \file abstractvertex_atemtps.hpp
 *
 * vertex object
 *
 * \author Tianyi Gu
 * \date   09 / 12 / 2017
 */ 
#pragma once

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

class AbstractVertex {
  public:

    struct AbstractVetexComparator {
        bool operator()(const AbstractVertex *lhs, const AbstractVertex *rhs) const {
            if(fabs(lhs->effort - rhs->effort) <= 0.00001) {
                return lhs->startID < rhs->startID;
            }
            return lhs->effort < rhs->effort;
        }
    };
    
    AbstractVertex(unsigned int id) : id(id) {}
	
    virtual ~AbstractVertex() {}

    void addState(ompl::base::State *s, double g) {
        assert(states.find(s) == states.end());
        costG = (costG * states.size() + g) / double(states.size() + 1);
        states.insert(s);
    }

    void removeState(ompl::base::State *s, double g) {
        auto iter = states.find(s);
        assert(states.size() > 0);
        assert(iter != states.end());
        states.erase(iter);
        if(states.size() == 0) costG = std::numeric_limits<double>::infinity();
        else costG = (costG * (states.size() + 1)- g) / double(states.size())
    }

   

    // add by tianyi, Aug / 8 / 2017
    ompl::base::State* sampleStateByDis(const ompl::base::SpaceInformation *si_,
                                        ompl::base::State* targetState) {
        double bestDis = std::numeric_limits<double>::infinity();
        ompl::base::State* ret;
        if(states.size() == 0){
            // if all state is removed by sstprunning or cost prunning
            // the we will select the closest state on the motion tree as
            // source state
            return targetState;
        }
        for(auto s: states){
            // we don't have the counter here, just pick the closest one
            double curDis = si_->distance(s, targetState);
            if(bestDis > curDis){
                bestDis = curDis;
                ret = s;
            }
        }
        return ret;
    }

    static bool pred(const AbstractVertex *a, const AbstractVertex *b) {
        return a->currentEffortToGoal < b->currentEffortToGoal;
    }
    static unsigned int getHeapIndex(const AbstractVertex *r) {
        return r->heapIndex;
    }
    static void setHeapIndex(AbstractVertex *r, unsigned int i) {
        r->heapIndex = i;
    }

    bool operator<(const AtemptsVertexWrapper& k) const {
        if(currentEffortToGoal == k.currentEffortToGoal)
            return currentCostToGoal < k.currentCostToGoal;
        return currentEffortToGoal < k.currentEffortToGoal;
    }

    bool insertPath(Path * p, double incumbentCost){
        Path *cur = paretoFrontier->next;
        Path *prev = paretoFrontier;
        while(cur != nullptr){
            if(cur->costToGoal + costG > incumbent){
                // prune all paths whose total cost is greater than incumbent
                prev->next = cur->next;
                delete cur;
                cur = prev->next;
            }
            else if (p->costToGoal < cur->costToGoal){
                if(p->effortToGoal < cur->effortToGoal){
                    // p dominate cur,  delete cur
                    prev->next = cur->next;
                    delete cur;
                    cur = prev->next;
                }
                else{
                    prev = cur;
                    cur = cur->next;   
                }               
            }
            // insert follow by prev
            else if(p->effortToGoal < cur->effortToGoal) break;
            // dominate by cur;
            else return false;
        }
        
        p->next = prev->next;
        prev->next = p;
       
        return true;
    }
  
    unsigned int id;    

    std::unordered_set<ompl::base::State*> states;
    std::vector<ompl::base::State*> removedStates;
    unsigned int heapIndex = std::numeric_limits<unsigned int>::max();

    // costG is average of all motions begin at the start state and end in here
    double costG = std::numeric_limits<double>::infinity();
    double costGByEdge = std::numeric_limits<double>::infinity(); 
    double currentEffortToGoal = std::numeric_limits<double>::infinity();
    double currentCostF = std::numeric_limits<double>::infinity();
    Path * bestPath; // current best path that better than incumbent
    // header sentiel node of trade-off curve link list 
    Path * paretoFrontier = new Path(0, nullptr); 
};

}

}

}

}
