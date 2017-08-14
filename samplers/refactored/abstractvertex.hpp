#pragma once

namespace ompl {

namespace base {

namespace refactored {

class AbstractVertex {
  public:
    AbstractVertex(unsigned int id) : id(id) {}
	
    virtual ~AbstractVertex() {}

    void addState(ompl::base::State *s) {
        assert(states.find(s) == states.end());
        states.insert(s);
    }

    void removeState(ompl::base::State *s) {
        auto iter = states.find(s);
        assert(states.size() > 0);
        assert(iter != states.end());
        states.erase(iter);
    }

    unsigned int id;

    double initG = std::numeric_limits<double>::infinity();
    double initH = std::numeric_limits<double>::infinity();

    std::unordered_set<ompl::base::State*> states;
    std::vector<ompl::base::State*> removedStates;

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
            // we don't have the counter here, only pick the closest one
            double curDis = si_->distance(s, targetState);
            if(bestDis > curDis){
                bestDis = curDis;
                ret = s;
            }
        }
        return ret;
    }
};

}

}

}
