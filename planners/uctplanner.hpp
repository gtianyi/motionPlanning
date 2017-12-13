#pragma once

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "../structs/filemap.hpp"

#include "../samplers/beastsampler_dstar.hpp"
#include "../samplers/beastsampler_dijkstra.hpp"
#include "../samplers/beastsampler_dstarDis.hpp"
#include "../samplers/uctsampler.hpp"
#include <limits>
#include <fstream>

namespace ompl {

namespace control {

class UCTPlanner : public ompl::control::RRT {
  public:

    /** \brief Constructor */
    UCTPlanner(const SpaceInformationPtr &si, const FileMap &params) :
            ompl::control::RRT(si), newsampler_(NULL), params(params) {

        whichSearch = params.stringVal("WhichSearch");

        std::string plannerName = "UCTPlanner_" + whichSearch;
        setName(plannerName);

        Planner::declareParam<double>("state_radius", this, &UCTPlanner::ignoreSetterDouble, &UCTPlanner::getStateRadius);
        Planner::declareParam<unsigned int>("prm_size", this, &UCTPlanner::ignoreSetterUnsigedInt, &UCTPlanner::getPRMSize);
        Planner::declareParam<unsigned int>("num_prm_edges", this, &UCTPlanner::ignoreSetterUnsigedInt, &UCTPlanner::getNumPRMEdges);

        Planner::declareParam<double>("valid_edge_distribution_alpha", this, &UCTPlanner::ignoreSetterDouble, &UCTPlanner::getValidEdgeDistributionAlpha);
        Planner::declareParam<double>("valid_edge_distribution_beta", this, &UCTPlanner::ignoreSetterDouble, &UCTPlanner::getValidEdgeDistributionBeta);
        Planner::declareParam<double>("invalid_edge_distribution_alpha", this, &UCTPlanner::ignoreSetterDouble, &UCTPlanner::getInvalidEdgeDistributionAlpha);
        Planner::declareParam<double>("invalid_edge_distribution_beta", this, &UCTPlanner::ignoreSetterDouble, &UCTPlanner::getInvalidEdgeDistributionBeta);

        //Obviously this isn't really a parameter but I have no idea how else to get it into the output file through the benchmarker
        Planner::declareParam<double>("sampler_initialization_time", this, &UCTPlanner::ignoreSetterDouble, &UCTPlanner::getSamplerInitializationTime);
    }

    virtual ~UCTPlanner() {}

    void ignoreSetterDouble(double) const {}
    void ignoreSetterUnsigedInt(unsigned int) const {}

    double getStateRadius() const {
        return params.doubleVal("StateRadius");
    }
    unsigned int getPRMSize() const {
        return params.integerVal("PRMSize");
    }
    unsigned int getNumPRMEdges() const {
        return params.integerVal("NumEdges");
    }
    double getValidEdgeDistributionAlpha() const {
        return params.doubleVal("ValidEdgeDistributionAlpha");
    }
    double getValidEdgeDistributionBeta() const {
        return params.doubleVal("ValidEdgeDistributionBeta");
    }
    double getInvalidEdgeDistributionAlpha() const {
        return params.doubleVal("InvalidEdgeDistributionAlpha");
    }
    double getInvalidEdgeDistributionBeta() const {
        return params.doubleVal("InvalidEdgeDistributionBeta");
    }
    double getSamplerInitializationTime() const {
        return samplerInitializationTime;
    }

    /** \brief Continue solving for some amount of time. Return true if solution was found. */
    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) {
        checkValidity();
        base::Goal                   *goal = pdef_->getGoal().get();
        base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

        while(const base::State *st = pis_.nextStart()) {
            Motion *motion = new Motion(siC_);
            si_->copyState(motion->state, st);
            siC_->nullControl(motion->control);
            nn_->add(motion);
        }

        if(nn_->size() == 0) {
            OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return base::PlannerStatus::INVALID_START;
        }

        if(!newsampler_) {
            auto start = clock();

            if(whichSearch.compare("UCT") == 0) {
                newsampler_ = new ompl::base::UCTSampler((ompl::base::SpaceInformation *)siC_, pdef_->getStartState(0), pdef_->getGoal(),
                                                         goal_s, params);
            }else {
                throw ompl::Exception("Unrecognized best first search type", whichSearch.c_str());
            }
		

            

            samplerInitializationTime = (double)(clock() - start) / CLOCKS_PER_SEC;
        }
        if(!controlSampler_)
            controlSampler_ = siC_->allocDirectedControlSampler();

        OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

        Motion *solution  = NULL;
        Motion *approxsol = NULL;
        double  approxdif = std::numeric_limits<double>::infinity();

        Motion      *rmotion_start = new Motion(siC_);
        base::State  *rstate_s = rmotion_start->state;
        Control       *rctrl_s = rmotion_start->control;

        Motion      *rmotion_end = new Motion(siC_);
        base::State  *rstate_e = rmotion_end->state;
        Control       *rctrl_e = rmotion_end->control;
        
        base::State  *xstate = si_->allocState();

        Motion *resusableMotion = new Motion(siC_);
        
        std::vector<std::vector<int>> edges = newsampler_->initializeForTest();
        std::ofstream outfile;
        outfile.open("test.csv");
        outfile << "start,end,estimate,alpha,beta\n";
        int edgeCount = 0;
        for (int s = 0;s < edges.size();s++){
            for (const auto &e : edges[s]){
                edgeCount++;
                Motion *nmotion = NULL;

                bool estimate_collision =  newsampler_->abstract_estimate(s, e);
                std::string esti = estimate_collision ? "TRUE" : "FALSE";

                int alpha = 0;
                int beta = 0;
                int n = 1000;
                while(n > 0){
                    n--;
                    newsampler_->sampleTest(s, e, rstate_s, rstate_e);
                    /* sample a random control that attempts to go towards
                     the random state, and also sample a control duration */
                    unsigned int cd = controlSampler_->sampleTo(rctrl_s,
                                                                rmotion_start->control,
                                                                rstate_s,
                                                                rstate_e);

                    if(addIntermediateStates_) {
                        std::vector<base::State *> pstates;
                        cd = siC_->propagateWhileValid(rstate_s,
                                                       rctrl_s, cd, pstates, true);
                        if(cd >= siC_->getMinControlDuration()) {
                            bool success = false;
                            int p = 0;
                            for(; p < pstates.size(); ++p) {
                                if(newsampler_->reachedTest(e,pstates[p])) {
                                    alpha++;
                                    success = true;
                                    break;
                                }
                            }
                            if(!success)
                                beta++;
                            //free any states after we hit the goal
                            while(++p < pstates.size())
                                si_->freeState(pstates[p]);
                        } else{
                            beta++;
                            for(size_t p = 0 ; p < pstates.size(); ++p)
                                si_->freeState(pstates[p]);
                        }
                    } else {
                        if(cd >= siC_->getMinControlDuration()) {
                            if(newsampler_->reachedTest(e,rstate_e)) {
                                alpha++;
                            }
                            else{
                                beta++;
                            }
                        }
                        else{
                            beta++;
                        }
                    }
                }
                outfile << s << "," << e << "," << esti << "," <<
                        alpha << "," << beta <<"\n";
                std::cout << edgeCount << ":"<< s << "," << e << "," << esti << ","
                          << alpha << "," << beta <<"\n";
            }
        }
        outfile.close();
        bool solved = false;
        bool approximate = false;
           
        return base::PlannerStatus(solved, approximate);
    }

    virtual void clear() {
        RRT::clear();
        // delete newsampler_;
        // newsampler_ = NULL;
    }

  protected:

    ompl::base::UCTSampler *newsampler_;
    const FileMap &params;
    std::string whichSearch;
    double samplerInitializationTime = 0;
    // motion tree,  look up motion by state id
    // unorder_map<int,  Motion *> mTree;
};

}

}
