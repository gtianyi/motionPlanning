#pragma once

/*********************************************
 * reference:
1.  ompl                KinematicChainBenchmark.cpp
2.  moremotionplanning  acrobot.hpp
*********************************************/
/* Author: Tianyi Gu */

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include "../planners/beastplannergeometric.hpp"

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

// a 2D line segment
struct Segment
{
  Segment(double p0_x, double p0_y, double p1_x, double p1_y)
      : x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y)
  {
  }
  double x0, y0, x1, y1;
};

// the robot and environment are modeled both as a vector of segments.
using Environment = std::vector<Segment>;

// simply use a random projection
class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
 public:
  KinematicChainProjector(const ompl::base::StateSpace *space)
      : ompl::base::ProjectionEvaluator(space)
  {
    int dimension = std::max(2, (int)ceil(log((double) space->getDimension())));
    projectionMatrix_.computeRandom(space->getDimension(), dimension);
  }
  unsigned int getDimension() const override
  {
    return projectionMatrix_.mat.size1();
  }
  void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const override
  {
    std::vector<double> v(space_->getDimension());
    space_->copyToReals(v, state);
    projectionMatrix_.project(&v[0], projection);
  }
 protected:
  ompl::base::ProjectionMatrix projectionMatrix_;
};


class KinematicChainSpace : public ompl::base::CompoundStateSpace
{
 public:
  KinematicChainSpace(unsigned int numLinks, double linkLength, Environment *env = nullptr)
      : ompl::base::CompoundStateSpace(), linkLength_(linkLength), environment_(env)
  {
    for (unsigned int i = 0; i < numLinks; ++i)
      addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 1.);
    lock();
  }

  void registerProjections() override
  {
    registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
  }

  double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
  {
    const StateType *cstate1 = state1->as<StateType>();
    const StateType *cstate2 = state2->as<StateType>();
    double theta1 = 0., theta2 = 0., dx = 0., dy = 0., dist = 0.;

    for (unsigned int i = 0; i < getSubspaceCount(); ++i)
    {
      theta1 += cstate1->as<ompl::base::SO2StateSpace::StateType>(i)->value;
      theta2 += cstate2->as<ompl::base::SO2StateSpace::StateType>(i)->value;
      dx += cos(theta1) - cos(theta2);
      dy += sin(theta1) - sin(theta2);
      dist += sqrt(dx * dx + dy * dy);
    }
    return dist * linkLength_;
  }
  double linkLength() const
  {
    return linkLength_;
  }
  const Environment* environment() const
  {
    return environment_;
  }

 protected:
  double linkLength_;
  Environment* environment_;
};


class KinematicChainValidityChecker : public ompl::base::StateValidityChecker
{
 public:
  KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si)
      : ompl::base::StateValidityChecker(si)
  {
  }

  bool isValid(const ompl::base::State *state) const override
  {
    const KinematicChainSpace* space = si_->getStateSpace()->as<KinematicChainSpace>();
    const KinematicChainSpace::StateType *s = state->as<KinematicChainSpace::StateType>();
    unsigned int n = si_->getStateDimension();
    Environment segments;
    double linkLength = space->linkLength();
    double theta = 0., x = 0., y = 0., xN, yN;

    segments.reserve(n + 1);
    for(unsigned int i = 0; i < n; ++i)
    {
      theta += s->as<ompl::base::SO2StateSpace::StateType>(i)->value;
      xN = x + cos(theta) * linkLength;
      yN = y + sin(theta) * linkLength;
      segments.emplace_back(x, y, xN, yN);
      x = xN;
      y = yN;
    }
    xN = x + cos(theta) * 0.001;
    yN = y + sin(theta) * 0.001;
    segments.emplace_back(x, y, xN, yN);
    return selfIntersectionTest(segments)
        && environmentIntersectionTest(segments, *space->environment());
  }

 protected:
  // return true iff env does *not* include a pair of intersecting segments
  bool selfIntersectionTest(const Environment& env) const
  {
    for (unsigned int i = 0; i < env.size(); ++i)
      for (unsigned int j = i + 1; j < env.size(); ++j)
        if (intersectionTest(env[i], env[j]))
          return false;
    return true;
  }
  // return true iff no segment in env0 intersects any segment in env1
  bool environmentIntersectionTest(const Environment& env0, const Environment& env1) const
  {
    for (const auto & i : env0)
      for (const auto & j : env1)
        if (intersectionTest(i, j))
          return false;
    return true;
  }
  // return true iff segment s0 intersects segment s1
  bool intersectionTest(const Segment& s0, const Segment& s1) const
  {
    // adopted from:
    // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/1201356#1201356
    double s10_x = s0.x1 - s0.x0;
    double s10_y = s0.y1 - s0.y0;
    double s32_x = s1.x1 - s1.x0;
    double s32_y = s1.y1 - s1.y0;
    double denom = s10_x * s32_y - s32_x * s10_y;
    if (fabs(denom) < std::numeric_limits<double>::epsilon())
      return false; // Collinear
    bool denomPositive = denom > 0;

    double s02_x = s0.x0 - s1.x0;
    double s02_y = s0.y0 - s1.y0;
    double s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
      return false; // No collision
    double t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
      return false; // No collision
    if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive)
        || ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive))
      return false; // No collision
    return true;
  }
};


Environment createHornEnvironment(unsigned int d, double eps)
{
  std::ofstream envFile("environment.dat");
  std::vector<Segment> env;
  double w = 1. / (double)d, x = w, y = -eps, xN, yN, theta = 0.,
      scale = w * (1. + boost::math::constants::pi<double>() * eps);

  envFile << x << " " << y << std::endl;
  for(unsigned int i = 0; i < d - 1; ++i)
  {
    theta += boost::math::constants::pi<double>() / (double) d;
    xN = x + cos(theta) * scale;
    yN = y + sin(theta) * scale;
    env.emplace_back(x, y, xN, yN);
    x = xN;
    y = yN;
    envFile << x << " " << y << std::endl;
  }

  theta = 0.;
  x = w;
  y = eps;
  envFile << x << " " << y << std::endl;
  scale = w * (1.0 - boost::math::constants::pi<double>() * eps);
  for(unsigned int i = 0; i < d - 1; ++i)
  {
    theta += boost::math::constants::pi<double>() / d;
    xN = x + cos(theta) * scale;
    yN = y + sin(theta) * scale;
    env.emplace_back(x, y, xN, yN);
    x = xN;
    y = yN;
    envFile << x << " " << y << std::endl;
  }
  envFile.close();
  return env;
}

BenchmarkData linkageBenchmark(const FileMap &params) {
  unsigned int numLinks = params.integerVal("NumLinks");
  Environment env = createHornEnvironment(numLinks, log((double)numLinks) / (double)numLinks);
  auto chain(std::make_shared<KinematicChainSpace>(numLinks, 1. / (double)numLinks, &env));
  // ompl::geometric::SimpleSetup ss(chain);
  ompl::geometric::SimpleSetupPtr ssPtr(new ompl::geometric::SimpleSetup(chain));
  ssPtr->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ssPtr->getSpaceInformation()));

  ompl::base::ScopedState<> start(chain), goal(chain);
  std::vector<double> startVec(numLinks, boost::math::constants::pi<double>() / (double)numLinks);
  std::vector<double> goalVec(numLinks, 0.);

  startVec[0] = 0.;
  goalVec[0] = boost::math::constants::pi<double>() - .001;
  chain->setup();
  chain->copyFromReals(start.get(), startVec);
  chain->copyFromReals(goal.get(), goalVec);
  ssPtr->setStartAndGoalStates(start, goal);
  BenchmarkData data;
  data.benchmark = new ompl::tools::Benchmark( *ssPtr, "KinematicChain");
  // data.simplesetup = ssPtr;
  data.gsetup = ssPtr;
  // data.decomposition = linkage->allocDecomposition();

  // I can not pass benchmarkData to main.cpp.  So I run benchmark here
   auto planner = params.stringVal("Planner");

  ompl::base::PlannerPtr plannerPointer;
  if(planner.compare("RRT") == 0) {
    plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::RRT(data.gsetup->getSpaceInformation()));
  } else if(planner.compare("KPIECE") == 0) {
    plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::KPIECE1(data.gsetup->getSpaceInformation()));
  // } else if(planner.compare("EST") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::EST(data.simplesetup->getSpaceInformation()));
  // } else if(planner.compare("SyclopRRT") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::SyclopRRT(data.simplesetup->getSpaceInformation(), data.decomposition));
  // } else if(planner.compare("SyclopEST") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::SyclopEST(data.simplesetup->getSpaceInformation(), data.decomposition));
  // } else if(planner.compare("PDST") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::PDST(data.simplesetup->getSpaceInformation()));
  // } else if(planner.compare("FBiasedRRT") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::FBiasedRRT(data.simplesetup->getSpaceInformation(), params));
  // } else if(planner.compare("FBiasedShellRRT") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::FBiasedShellRRT(data.simplesetup->getSpaceInformation(), params));
  // } else if(planner.compare("PlakuRRT") == 0) {
  //   plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::PlakuRRT(data.simplesetup->getSpaceInformation(), params));
  } else if(planner.compare("BEAST") == 0) {
    plannerPointer = ompl::base::PlannerPtr(new ompl::geometric::BeastPlanner(data.gsetup->getSpaceInformation(), params));
  }
  
 plannerPointer->setProblemDefinition(data.gsetup->getProblemDefinition());
  data.benchmark->addPlanner(plannerPointer);
  
  ompl::tools::Benchmark::Request req;
  req.maxTime = params.doubleVal("Timeout");
  req.maxMem = params.doubleVal("Memory");
  req.runCount = params.doubleVal("Runs");
  req.displayProgress = true;
  req.saveConsoleOutput = false;

  data.benchmark->benchmark(req);
  data.benchmark->saveResultsToFile(params.stringVal("Output").c_str());
  
  return data;
}
