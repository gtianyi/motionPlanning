/**
 * \file abstractedge_atemtps.hpp
 *
 * edge object
 *
 * \author Tianyi Gu
 * \date   09 / 12 / 2017
 */ 
#pragma once

namespace ompl {

namespace base {

namespace refactored {

namespace atempts {
class AbstractEdge {
  public:
    static double validEdgeDistributionAlpha, validEdgeDistributionBeta, invalidEdgeDistributionAlpha, invalidEdgeDistributionBeta;

    struct AbstractEdgeComparator {
        bool operator()(const AbstractEdge *lhs, const AbstractEdge *rhs) const {
            if(fabs(lhs->effort - rhs->effort) <= 0.00001) {
                return lhs->startID < rhs->startID;
            }
            return lhs->effort < rhs->effort;
        }
    };

    AbstractEdge(unsigned int startID, unsigned int endID) :
            startID(startID), endID(endID){}

    virtual ~AbstractEdge() {}

    void updateEdgeStatusKnowledge(Abstraction::Edge::CollisionCheckingStatus newStatus) {
        status = newStatus;
        switch(status) {
            case Abstraction::Edge::VALID:
                alpha = validEdgeDistributionAlpha;
                beta = validEdgeDistributionBeta;
                effort = getEstimatedRequiredSamples();
                break;
            case Abstraction::Edge::INVALID:
                alpha = invalidEdgeDistributionAlpha;
                beta = invalidEdgeDistributionBeta;
                effort = getEstimatedRequiredSamples();
                break;

            case Abstraction::Edge::UNKNOWN:
                // alpha = unknownEdgeDistributionAlpha;
                // beta = unknownEdgeDistributionBeta;
                break;
        }
    }

    void succesfulPropagation() {
        alpha++;
        effort = getEstimatedRequiredSamples();
    }

    void failurePropagation() {
        beta++;
        effort = getEstimatedRequiredSamples();
    }

    double getEstimatedRequiredSamples() const {
        double probability = alpha / (alpha + beta);
        double estimate = 1. / probability;
        return estimate;
    }

    double getHypotheticalEstimatedSamplesAfterPositivePropagation(unsigned int numberOfStates) const {
        double additive = (1. / (double)numberOfStates);
        double probability = (alpha + additive) / (alpha + additive + beta);
        double estimate = 1. / probability;
        return estimate;
    }

    void rewardHypotheticalSamplesAfterPositivePropagation(unsigned int numberOfStates) {
        double additive = (1. / ((double)numberOfStates - 1));
        alpha += additive;
    }

    
    void updateEdgeCost(double _cost){
        assert(_cost >= 0);
        cost = (cost * motionsNum + _cost) / (double)(motionsNum + 1);
        motionsNum++;
        epsilon = cost / abstractCost;
        // we do nothing when we remove a state
    }

    void setAbstractCost(double _cost){
        assert(_cost >= 0);
        abstractCost = _cost;
        cost = _cost;
    }

    double getCost(double epsilonBar){
        if(motionsNum == 1) return epsilonBar * abstractCost;
        return cost;
    }

    unsigned int startID, endID, interiorToNextEdgeID;
		
    Abstraction::Edge::CollisionCheckingStatus status = Abstraction::Edge::UNKNOWN;
	
    double alpha = validEdgeDistributionAlpha;
    double beta = validEdgeDistributionBeta;

    double effort = std::numeric_limits<double>::infinity();
    double cost = std::numeric_limits<double>::infinity();
    double abstractCost = std::numeric_limits<double>::infinity();
    int motionsNum = 1;
    double epsilon = 1;
	
    bool interior = false;
};

double AbstractEdge::validEdgeDistributionAlpha = 0;
double AbstractEdge::validEdgeDistributionBeta = 0;

double AbstractEdge::invalidEdgeDistributionAlpha = 0;
double AbstractEdge::invalidEdgeDistributionBeta = 0;



}

}

}

}
