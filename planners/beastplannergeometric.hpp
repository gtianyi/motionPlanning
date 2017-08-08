#pragma once

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "../structs/filemap.hpp"

#include "../samplers/beastsampler_dstar.hpp"
#include "../samplers/beastsampler_dijkstra.hpp"
#include <limits>

namespace ompl {

namespace geometric {

class BeastPlanner : public ompl::geometric::RRT {
 public:

  /** \brief Constructor */
  BeastPlanner (const base::SpaceInformationPtr &si, const FileMap &params) :
      ompl::geometric::RRT(si), newsampler_(NULL), params(params) {

    whichSearch = params.stringVal("WhichSearch");

    std::string plannerName = "BeastPlanner_" + whichSearch;
    setName(plannerName);

    Planner::declareParam<double>("state_radius", this, &BeastPlanner::ignoreSetterDouble, &BeastPlanner::getStateRadius);
    Planner::declareParam<unsigned int>("prm_size", this, &BeastPlanner::ignoreSetterUnsigedInt, &BeastPlanner::getPRMSize);
    Planner::declareParam<unsigned int>("num_prm_edges", this, &BeastPlanner::ignoreSetterUnsigedInt, &BeastPlanner::getNumPRMEdges);

    Planner::declareParam<double>("valid_edge_distribution_alpha", this, &BeastPlanner::ignoreSetterDouble, &BeastPlanner::getValidEdgeDistributionAlpha);
    Planner::declareParam<double>("valid_edge_distribution_beta", this, &BeastPlanner::ignoreSetterDouble, &BeastPlanner::getValidEdgeDistributionBeta);
    Planner::declareParam<double>("invalid_edge_distribution_alpha", this, &BeastPlanner::ignoreSetterDouble, &BeastPlanner::getInvalidEdgeDistributionAlpha);
    Planner::declareParam<double>("invalid_edge_distribution_beta", this, &BeastPlanner::ignoreSetterDouble, &BeastPlanner::getInvalidEdgeDistributionBeta);

    //Obviously this isn't really a parameter but I have no idea how else to get it into the output file through the benchmarker
    Planner::declareParam<double>("sampler_initialization_time", this, &BeastPlanner::ignoreSetterDouble, &BeastPlanner::getSamplerInitializationTime);
  }

  virtual ~BeastPlanner() {}

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
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, st);
      // siC_->nullControl(motion->control);
      nn_->add(motion);
    }

    if(nn_->size() == 0) {
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      return base::PlannerStatus::INVALID_START;
    }

    if(!newsampler_) {
      auto start = clock();

      if(whichSearch.compare("D*") == 0) {
        newsampler_ = new ompl::base::BeastSampler_dstar( &(*si_), pdef_->getStartState(0), pdef_->getGoal(),
                                                         goal_s, params);
      } else if(whichSearch.compare("Dijkstra") == 0) {
        // newsampler_ = new ompl::base::BeastSampler_dijkstra((ompl::base::SpaceInformation *) si_, pdef_->getStartState(0), pdef_->getGoal(),
        // goal_s, params);
      } else {
        throw ompl::Exception("Unrecognized best first search type", whichSearch.c_str());
      }
		

      newsampler_->initialize();

      samplerInitializationTime = (double)(clock() - start) / CLOCKS_PER_SEC;
    }
    // if(!controlSampler_)
    //   controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion      *rmotion = new Motion(si_);
    base::State  *rstate = rmotion->state;
    // Control       *rctrl = rmotion->control;
    base::State  *xstate = si_->allocState();

    Motion *resusableMotion = new Motion(si_);

    while(ptc == false) {
      Motion *nmotion = NULL;

      /* sample random state (with goal biasing) */
      // if(goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
      // 	goal_s->sampleGoal(rstate);

      // 	/* find closest state in the tree */
      // 	nmotion = nn_->nearest(rmotion);
      // }
      // else {
      newsampler_->sample(resusableMotion->state, rstate);
      /* find closest state in the tree */
      nmotion = nn_->nearest(rmotion);
      // }

 /* find closest state in the tree */
      // Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
  }

  virtual void clear() {
    RRT::clear();
    // delete newsampler_;
    // newsampler_ = NULL;
  }

 protected:

  ompl::base::BeastSamplerBase *newsampler_;
  const FileMap &params;
  std::string whichSearch;
  double samplerInitializationTime = 0;
};

}

}
