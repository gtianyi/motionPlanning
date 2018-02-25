/**
 * \file abstractvertex_atemtps.hpp
 *
 * vertex object
 *
 * \author Tianyi Gu
 * \date   10 / 31 / 2017
 */ 
#pragma once
#include "atemptspath.hpp"
using namespace std;

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

class AbstractVertex {
  public:

    struct ComparatorByEffortToGoal {
        bool operator()(const shared_ptr<AbstractVertex> lhs,
                        const shared_ptr<AbstractVertex> rhs) const {
            return lhs->currentEffortToGoal > rhs->currentEffortToGoal;
        }
    };

    struct ComparatorByCostGByEdge {
        bool operator()(const shared_ptr<AbstractVertex> lhs,
                        const shared_ptr<AbstractVertex> rhs) const {
            return lhs->costGByEdge < rhs->costGByEdge;
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
        if(states.size() == 0)
            costG = std::numeric_limits<double>::infinity();
        else
            costG = (costG * (states.size() + 1)- g) / double(states.size());
    }
    
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

    bool insertPath(shared_ptr<AtemptsPath> p, double incumbentCost){
        shared_ptr<AtemptsPath> cur = paretoFrontier->next;
        shared_ptr<AtemptsPath> prev = paretoFrontier;
        updateCurrentCostG();
        // prune all paths whose total cost is greater than incumbent
        while(cur != nullptr){
            if(cur->costToGoal + currentCostG > incumbentCost){
                prev->next = cur->next;
                cur = prev->next;
                undominatePathCount--;
            }
            else break;
        }
        updateBest();
        if(p->costToGoal + currentCostG > incumbentCost) {
            return false;
        }
        while(cur !=  nullptr){
             if (p->costToGoal < cur->costToGoal){
                if(p->effortToGoal < cur->effortToGoal){
                    // p dominate cur,  delete cur
                    prev->next = cur->next;
                    cur = prev->next;
                    undominatePathCount--;
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
        updateBest();
        undominatePathCount++;
        return true;
    }

    void updateCurrentCostG(){
        currentCostG = states.size() == 0 ? costGByEdge : costG;
        // if no state in region, another way would be using
        // edge cost toward the best frontier region + frontier costG
    }
    void updateBest(){
        if(paretoFrontier->next == nullptr){
            currentEffortToGoal = std::numeric_limits<double>::infinity();
            currentCostF = std::numeric_limits<double>::infinity();
            bestPath = nullptr;
        }
        else{
            currentEffortToGoal =  paretoFrontier->next->effortToGoal;
            currentCostF = paretoFrontier->next->costToGoal + currentCostG;
            bestPath = paretoFrontier->next;
        }
    }

    void clearPareto(){
        paretoFrontier->next = nullptr;
        currentCostG = std::numeric_limits<double>::infinity();
        currentEffortToGoal = std::numeric_limits<double>::infinity();
        currentCostF = std::numeric_limits<double>::infinity();
        bestPath = nullptr;
        undominatePathCount = 0;
    }
  
    unsigned int id;    

    std::unordered_set<ompl::base::State*> states;
    std::vector<ompl::base::State*> removedStates;
    unsigned int heapIndex = std::numeric_limits<unsigned int>::max();

    // costG is average of all motions begin at the start state and end in here
    double costG = std::numeric_limits<double>::infinity();
    // costGByEdge is the cost given by the abstract graph
    double costGByEdge = std::numeric_limits<double>::infinity();
    // if there is motion, we use costG,  other wise we use costGByEdge
    double currentCostG = std::numeric_limits<double>::infinity();
    double currentEffortToGoal = std::numeric_limits<double>::infinity();
    double currentCostF = std::numeric_limits<double>::infinity();
    // current best path that better than incumbent
    shared_ptr<AtemptsPath> bestPath = nullptr;
    // header sentiel node of trade-off curve link list 
    shared_ptr<AtemptsPath> paretoFrontier = make_shared<AtemptsPath>(0, nullptr);
    int undominatePathCount = 0;
};

}

}

}

}
