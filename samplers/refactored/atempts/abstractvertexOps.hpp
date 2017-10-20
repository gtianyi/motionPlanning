/**
 * \file atemptspath.hpp
 *
 * path object 
 *
 * \author Tianyi Gu
 * \date   10 / 03 / 2017
 */   
#pragma once

namespace ompl {

namespace base {

namespace refactored {

class AbstractVertexOps: public AbstractVertex{
  public:

    static bool pred(const AbstractVertex *a, const AbstractVertex *b) {
        return a->costGByEdge < b->costGByEdge;
    }
    static unsigned int getHeapIndex(const AbstractVertex *r) {
        return r->heapIndex;
    }
    static void setHeapIndex(AbstractVertex *r, unsigned int i) {
        r->heapIndex = i;
    }
};

}

}

}
