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
        double closeForG[vertices.size()] = {-1.0};
        vertices[startID]->costGByEdge =  0;
        openForGPtr->push( vertices[startID]);
        while( !openForGPtr->empty()) {
            shared_ptr<Vertex> v =  openForGPtr->top();
            openForGPtr->pop();
            std::vector<unsigned int >  kidVertices =
                    abstraction->getNeighboringCells(v->id);
            for(unsigned int kidVertexIndex:kidVertices) {
                shared_ptr<Vertex> kid = vertices[kidVertexIndex];
                shared_ptr<Edge> e =  getEdge(v->id,  kidVertexIndex);
                double newCost =  v->costGByEdge +  e->getCost(epsilonBar);
                if(closeForG[kidVertexIndex] == -1.0){
                    kid->costGByEdge =  newCost;
                    openForGPtr->push(kid);
                }
            }
            closeForG[v->id] =  v->costGByEdge;
        }
        // cout << "done cost............................. " << endl;
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
        p->effortToGoalWithBonus = 0;
        p->costToGoal = 0;
        openPtr->push(p);
        int loopcount = 0;
        while( !openPtr->empty()) {
            loopcount++;
            shared_ptr<Path> currentP = openPtr->top();
            openPtr->pop();
            std::vector<unsigned int> kidVertices =
                    abstraction->getNeighboringCells(currentP->id);
            // cout << "Pareto open:" << openPtr->size() << endl;
            for(unsigned int kidVertexIndex:kidVertices){
                shared_ptr<Path> kid = make_shared<Path>(kidVertexIndex, currentP);
                shared_ptr<Edge> e = getEdge(kidVertexIndex, currentP->id);
                // kid->effortToGoal = currentP->effortToGoal + e->effort;
                kid->effortToGoal = currentP->effortToGoalWithBonus + e->effort;
                kid->effortToGoalWithBonus = currentP->effortToGoalWithBonus +
                        e->getEffortWithBonus();
                kid->costToGoal = currentP->costToGoal + e->getCost(epsilonBar);
                // if(kid->effortToGoal !=  kid->effortToGoalWithBonus){
                //     cout << "kid index: " << kidVertexIndex << endl;
                //     cout << "kid effort2G: " << kid->effortToGoal << endl;
                //     cout << "kid effort2GwithB: " << kid->effortToGoalWithBonus << endl;
                //     cout << "parent effort2GwithB: " << currentP->effortToGoalWithBonus << endl;
                //     cout << "parent effort2G: " << currentP->effortToGoal << endl;
                //     cout << "kid cost2G: " << kid->costToGoal << endl;
                // }
                if(vertices[kidVertexIndex]->insertPath(kid, incumbentCost)){
                    openPtr->push(kid);
                }
            }
        }
        
        shared_ptr<Edge> goalEdge = getEdge(goalID, goalID);
        p->effortToGoal = goalEdge->effort;
        vertices[goalID]->clearPareto();
        vertices[goalID]->insertPath(p, incumbentCost);
        // cout << "goalEdge effort : "<< goalEdge->effort << endl;
        // cout << "goalV p : "<< vertices[goalID]->undominatePathCount << endl;
        // cout << "#nodes: "<< vertices.size() << endl;
        // cout << "#total path " << totalpath << endl;
        // cout << "#average: " << (double)totalpath / (double)vertices.size() << endl;
        // cout << "start pareto: " << vertices[startID]->undominatePathCount << endl;
        cout << "pareto loop: " << loopcount << endl;
        // cout << "done pareto........................................  " << endl;
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
