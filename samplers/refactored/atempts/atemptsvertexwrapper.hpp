#pragma once

namespace ompl {

namespace base {

namespace refactored {

class AtemptsVertexWrapper {
  public:
    typedef AtemptsPath Path;

    AtemptsVertexWrapper(unsigned int id) : id(id) {}

    static bool pred(const DStarAbleVertexWrapper *a, const DStarAbleVertexWrapper *b) {
        return a->key < b->key;
    }
    static unsigned int getHeapIndex(const DStarAbleVertexWrapper *r) {
        return r->heapIndex;
    }
    static void setHeapIndex(DStarAbleVertexWrapper *r, unsigned int i) {
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
    unsigned int heapIndex = std::numeric_limits<unsigned int>::max();

    Key key;
    // costG is average of all motions begin at the start state and end in here
    double costG = std::numeric_limits<double>::infinity(); 
    double currentEffortToGoal = std::numeric_limits<double>::infinity();
    double currentCostF = std::numeric_limits<double>::infinity();
    Path * currentPath; // current path that better than incumbent
    // header sentiel node of trade-off curve link list 
    Path * paretoFrontier = new Path(0, nullptr); 
};

}

}

}
