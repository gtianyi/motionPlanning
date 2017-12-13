/**
 * \file atemptsdijkstra.hpp
 *
 * compute pareto frontier trade-off curve for every vetex
 *
 * \author Tianyi Gu
 * \date   09 / 12 / 2017
 */ 

#pragma once

#include "atemptspath.hpp"
using namespace std;

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {

template <class Vertex, class Edge>
class AtemptsDijkstra {

    typedef AtemptsPath Path;
       
  public:
    AtemptsDijkstra(std::vector<shared_ptr<Vertex>> &baseVertices, unsigned int startID,unsigned int goalID, Abstraction *abstraction, 
                    std::function<shared_ptr<Edge>(unsigned int, unsigned int)> getEdge) :
            vertices(baseVertices), startID(startID), goalID(goalID), abstraction(abstraction), getEdge(getEdge) {}

    void updateCostGByEdge() {
        openForGPtr = make_shared<
            priority_queue<shared_ptr<Vertex>,
                           vector<shared_ptr<Vertex>>,
                           typename Vertex::ComparatorByCostGByEdge>>();
        closeForG.clear();
        vertices[startID]->costGByEdge =  0;
        openForGPtr->push( vertices[startID]);
        while( !openForGPtr->empty()) {
            // cout << "open size: "<< openForGPtr->size() << endl;
            // cout << "close size: "<< closeForG.size() << endl;
            shared_ptr<Vertex> v =  openForGPtr->top();
            openForGPtr->pop();
            // cout << "top " << v->costGByEdge << endl;
            std::vector<unsigned int >  kidVertices =
                    abstraction->getNeighboringCells(v->id);
            for(unsigned int kidVertexIndex:kidVertices){
                shared_ptr<Vertex> kid = vertices[kidVertexIndex];
                shared_ptr<Edge> e =  getEdge(v->id,  kidVertexIndex);
                double newCost =  v->costGByEdge +  e->getCost(epsilonBar);
                if(closeForG.find(kidVertexIndex) !=  closeForG.end()){
                    // if(newCost < closeForG[kidVertexIndex]){
                    //     cout << "wrong...  " << newCost << " " << closeForG[kidVertexIndex] << endl;
                    //     closeForG.erase(kidVertexIndex);
                    //     kid->costGByEdge =  newCost;
                    // }
                    // else continue;
                    continue;
                }
                else{
                    kid->costGByEdge =  newCost;
                }
                openForGPtr->push(kid);
                closeForG[v->id] =  v->costGByEdge;
            }
        }
        cout << "done cost........................................  " << endl;
    }

    void updatePareto() {
        for (const auto &v : vertices){
            v->clearPareto();
        }

        openPtr = make_shared<
            priority_queue<shared_ptr<Path>,
                           vector<shared_ptr<Path>>,
                           AtemptsPath::AtemptsPathComparator>>();
        shared_ptr<Path> p = make_shared<Path>(goalID, nullptr);
        p->effortToGoal = 0;
        p->costToGoal = 0;
        openPtr->push(p);
        int totalpath = 1;
        while( !openPtr->empty()) {
            shared_ptr<Path> currentP = openPtr->top();
            openPtr->pop();
            std::vector<unsigned int> kidVertices =
                    abstraction->getNeighboringCells(currentP->id);
            // cout << "Pareto open:" << openPtr->size() << endl;
            for(unsigned int kidVertexIndex:kidVertices){
                shared_ptr<Path> kid = make_shared<Path>(kidVertexIndex, currentP);
                shared_ptr<Edge> e = getEdge(kidVertexIndex, currentP->id);
                kid->effortToGoal = currentP->effortToGoal + e->effort;
                kid->costToGoal = currentP->costToGoal + e->getCost(epsilonBar);
                if(vertices[kidVertexIndex]->insertPath(kid, incumbentCost)){
                    openPtr->push(kid);
                    totalpath++;
                }
            }
        }
        cout << "#nodes: "<< vertices.size() << endl;
        cout << "#total path " << totalpath << endl;
        cout << "#average: " << (double)totalpath / (double)vertices.size() << endl;
        cout << "done pareto........................................  " << endl;
    }

    void updateIncumbentCost(double _incumbentCost) {
        incumbentCost = _incumbentCost;
    }
    
    void updateEpsilonBar(double _epsilonBar){
        epsilonBar = _epsilonBar;
    }

    shared_ptr<priority_queue<shared_ptr<Path>,
                              vector<shared_ptr<Path>>,
                              AtemptsPath::AtemptsPathComparator>> openPtr;
    

    shared_ptr<priority_queue<shared_ptr<Vertex>,
                              vector<shared_ptr<Vertex>>,
                              typename Vertex::ComparatorByCostGByEdge>> openForGPtr;
    unordered_map<int,  double >  closeForG;

  protected:
    
    unsigned int goalID;
    unsigned int startID;
    Abstraction *abstraction = nullptr;
    std::function<shared_ptr<Edge>(unsigned int, unsigned int)> getEdge;
    double incumbentCost = std::numeric_limits<double>::infinity();
    double epsilonBar = 1;
    std::vector<shared_ptr<Vertex>> &vertices;
    
};

}

}

}

}
