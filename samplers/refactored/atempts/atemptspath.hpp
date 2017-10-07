/**
 * \file atemptspath.hpp
 *
 * path object 
 *
 * \author Tianyi Gu
 * \date   09 / 12 / 2017
 */   

#pragma once

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

class AtemptsPath {
  public:
   
    AtemptsPath(unsigned int id,  AtemptsPath * parent) : id(id), parent(parent){}

    static bool pred(const AtemptsPath *a, const AtemptsPath *b) {
        return *a < *b;
    }
    
    static unsigned int getHeapIndex(const AtemptsPath *r) {
        return r->heapIndex;
    }
    
    static void setHeapIndex(AtemptsPath *r, unsigned int i) {
        r->heapIndex = i;
    }

    bool operator<(const AtemptsPath& k) const {
        if(effortToGoal == k.effortToGoal)
            return costToGoal < k.costToGoal;
        return effortToGoal < k.effortToGoal;
    }

    unsigned int id;
    AtemptsPath * parent; // partial path that not include last vetex;
    AtemptsPath * next; // next path in pareto frontier
    unsigned int heapIndex = std::numeric_limits<unsigned int>::max();

    double effortToGoal = std::numeric_limits<double>::infinity();
    double costToGoal = std::numeric_limits<double>::infinity();
};

}

}

}

}
