/**
 * \file atemptspath.hpp
 *
 * path object 
 *
 * \author Tianyi Gu
 * \date   10 / 30 / 2017
 */   

#pragma once
using namespace std;

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

class AtemptsPath {
  public:
   
    AtemptsPath(unsigned int id,  shared_ptr<AtemptsPath> parent) : id(id), parent(parent){}

    struct AtemptsPathComparator{
        bool operator()(const shared_ptr<AtemptsPath> left,
                        const shared_ptr<AtemptsPath> right) const {
            return left->costToGoal < right->costToGoal;
        }
    };
        
    unsigned int id;
    shared_ptr<AtemptsPath> parent; // partial path that doesn't include head vetex;
    shared_ptr<AtemptsPath> next; // next path in pareto frontier
    unsigned int heapIndex = std::numeric_limits<unsigned int>::max();

    double effortToGoal = std::numeric_limits<double>::infinity();
    double costToGoal = std::numeric_limits<double>::infinity();
};

}

}

}

}
