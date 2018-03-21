#pragma once
#include "prmlite.hpp"

class PRMGust: public PRMLite {
public:
    using PRMLite::prmSize;
    using PRMLite::numEdges;
    using PRMLite::stateRadius;
    using PRMLite::resizeFactor;
    
    PRMGust(const ompl::base::SpaceInformation* si,
                  const ompl::base::State* start,
                  const ompl::base::State* goal,
                  const FileMap& params) : PRMLite(si, start, goal, params) {}

    virtual ~PRMGust() = default;

    void split_regions(int regIon)
    {
        unsigned int oldPRMSize =  prmSize;
        prmSize +=  1;
        vertices.resize(prmSize);
        ompl::base::StateSpacePtr abstractSpace =
                globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
        ompl::base::ValidStateSamplerPtr abstractSampler =
                globalParameters.globalAbstractAppBaseGeometric->
                getSpaceInformation()->allocValidStateSampler();

        for(unsigned int i =  oldPRMSize; i <  prmSize; ++i) {
            vertices[i] =  new Vertex(i);
            vertices[i]->state =  abstractSpace->allocState();
            abstractSampler->sampleNear(vertices[i]->state,
                                        vertices[regIon]->state, 1);
        }

        Vertex v(0);
        v.state =  vertices[oldPRMSize]->state;
        //  get the nearest sample
        int nearid = nn->nearest( &  v)->id;

        //  adding new sample near the old one
        vertices[oldPRMSize]->neighbors = vertices[nearid]->neighbors;
        vertices[oldPRMSize]->populatedNeighors = vertices[nearid]->populatedNeighors;
        nn->add(vertices[oldPRMSize]);
    }
};
