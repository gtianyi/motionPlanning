/**
 * \file atemtpsdijkstra.hpp
 *
 * compute pareto frontier trade-off curve for every vetex
 *
 * \author Tianyi Gu
 * \date   09 / 12 / 2017
 */ 

#pragma once

#include "abstractvertexOps.hpp"

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

template <class Vertex, class Edge>
class AtemptsDijkstra {

    typedef AtemptsPath Path;
       
  public:
    AtemptsDijkstra(std::vector<Vertex> &baseVertices, unsigned int startID,unsigned int goalID, Abstraction *abstraction, 
                    std::function<Edge*(unsigned int, unsigned int)> getEdge) :
            vertices(baseVertices), startID(startID), goalID(goalID), abstraction(abstraction), getEdge(getEdge), callback(callback) {}

    void updateCostG() {
        // clean may have bug because we don't delete all pointers here.
        openForG.clean();
        vertices[startID].costGByEdge = 0;
        openforG.push( &vertices[startID]);
        while(!openForG.isEmpty()) {
            Vertex *v = openForG.pop();
            std::vector<unsigned int> kidVetices =
                    abstraction->getNeighboringCells(v->id);
            for(unsigned int kidVetexIndex:kidVetices){
                Vertex *kid = &vertices[kidVetexIndex];
                Edge *e = getEdge(v->id, kidVetexIndex);
                double newCost = v->costGByEdge + e->cost;
                if(closeForG.find(kidVetexIndex)! = closeForG.end()){
                    if(newCost < closeForG[kidVetexIndex]){
                        closeForG.erase(kidVetexIndex);
                        kid->costGByEdge = newCost;
                    }
                    else continue;
                }
                else{
                    kid->costGByEdge = newCost;
                }
                openForG.push(kid);
                closeForG[v->id] = v->costGByEdge;
            }
        }
    }

    void updatePareto() {
        // clean may have bug because we don't delete all pointers here.
        open.clean();
        Path * p = new Path(goalID, nullptr);
        p->effortToGoal = 0;
        p->costToGoal = 0;
        open.push(p);
        while(!open.isEmpty()) {
            Path *currentP = open.pop();
            std::vector<unsigned int> kidVetices =
                    abstraction->getNeighboringCells(currentP->id);
            for(unsigned int kidVetexIndex:kidVetices){
                Path *kid = new Path(kidVetexIndex, currentP);
                Edge *e = getEdge(kidVetexIndex, currentP->id);
                kid->effortToGoal = currentP->effortToGoal + e->effort;
                kid->costToGoal = currentP->costToGoal + e->cost;
                if(vertices[kidVetexIndex].costG + kid->costToGoal < incumbentCost
                   && vertices[kidVetexIndex].insertPath(kid, incumbentCost)){
                    open.push(kid);
                }
                else delete kid;
            }
        }
    }

    void updateIncumbentCost(double _incumbentCost) {
        incumbentCost = _incumbentCost;
    }
    

    double getG(unsigned int i) const {
        return vertices[i].g;
    }

  protected:

    
    unsigned int goalID;
    unsigned int startID;
    Abstraction *abstraction = nullptr;
    std::function<Edge*(unsigned int, unsigned int)> getEdge;
    double incumbentCost = std::numeric_limits<double>::infinity();
    std::vector<Vertex> &vertices;

    InPlaceBinaryHeap<Path, Path> open;
    // fix opeenForG priority by costGByEdge
    InPlaceBinaryHeap<Vertex, AbstractVertexOps> openForG;
    unordered_map<int, double> closeForG;
};

}

}

}

}
