#pragma once

#include "beastsamplerbase.hpp"

namespace ompl {

namespace base {

class UCTSampler : public ompl::base::BeastSamplerBase {
public:
	UCTSampler(ompl::base::SpaceInformation *base, ompl::base::State *start, const ompl::base::GoalPtr &goal,
	            base::GoalSampleableRegion *gsr, const FileMap &params) : BeastSamplerBase(base, start, goal, gsr, params), addedGoalEdge(false) {}

	~UCTSampler() {}
    virtual void initialize() {
        BeastSamplerBase::initialize();
    }

     std::vector<std::vector<int>> initializeForTest() {
		BeastSamplerBase::initialize();

		unsigned int abstractionSize = abstraction->getAbstractionSize();
		vertices.reserve(abstractionSize);

                std::vector<std::vector<int>> ret;
		for(unsigned int i = 0; i < abstractionSize; ++i) {
			vertices.emplace_back(i);
                        std::vector<int> temp;
			auto neighbors = abstraction->getNeighboringCells(i);
			for(auto n : neighbors) {
                            temp.push_back(n);
				getEdge(i, n);
				getEdge(n, i);
			}
                        ret.push_back(temp);
		}

		vertices[goalID].rhs = 0;
		vertices[goalID].key = calculateKey(goalID);
		U.push(&vertices[goalID]);

		{
			Timer t("D* lite");
			computeShortestPath();
		}

		for(auto eset : edges) {
			for(auto e : eset.second) {
				e.second->initialEffort = e.second->effort;
			}
		}

		vertices[startID].addState(startState);
		addOutgoingEdgesToOpen(startID);

                return ret;
	}

    bool abstract_estimate(int start,  int end){
        return abstraction->isValidEdge(start,  end);
    }

    virtual bool sample(ompl::base::State *from,  ompl::base::State *to) {
        return true;
    }
    bool sampleTest(int start,  int end, ompl::base::State *from, ompl::base::State *to) {
        ompl::base::ScopedState <>  vertexState_end(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
        vertexState_end = abstraction->getState(end);
        ompl::base::ScopedState<> fullState_end = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState_end);
        fullStateSampler->sampleUniformNear(to, fullState_end.get(), stateRadius);

         ompl::base::ScopedState <>  vertexState_start(globalParameters.globalAppBaseControl->getGeometricComponentStateSpace());
        vertexState_start = abstraction->getState(start);
        ompl::base::ScopedState<> fullState_start = globalParameters.globalAppBaseControl->getFullStateFromGeometricComponent(vertexState_start);
        fullStateSampler->sampleUniformNear(from, fullState_start.get(), stateRadius);
                               
        return true;
    }

	virtual bool sample(ompl::base::State *) {
		throw ompl::Exception("NewSampler::sample", "not implemented");
		return false;
	}

	virtual bool sampleNear(ompl::base::State *, const ompl::base::State *, const double) {
		throw ompl::Exception("NewSampler::sampleNear", "not implemented");
		return false;
	}

    void reached(ompl::base::State *state) {
        return;
    }
    bool reachedTest(int end, ompl::base::State *state) {
		ompl::base::ScopedState<> incomingState(si_->getStateSpace());
		incomingState = state;
		unsigned int newCellId = abstraction->mapToAbstractRegion(incomingState);
                return newCellId == end;
	}


protected:
	void vertexMayBeInconsistent(unsigned int id) {
		updateVertex(id);
	}

	void vertexHasInfiniteValue(unsigned int id) {
		computeShortestPath();
	}


	Key calculateKey(unsigned int id) {
		Vertex &s = vertices[id];
		Key key;
		key.first = std::min(s.g, s.rhs);
		key.second = std::min(s.g, s.rhs);
		return key;
	}

	void updateVertex(unsigned int id) {
		Vertex &s = vertices[id];
		if(s.id != goalID) {
			double minValue = std::numeric_limits<double>::infinity();
			auto neighbors = abstraction->getNeighboringCells(id);
			for(auto n : neighbors) {
				Edge *e = getEdge(id, n);
				double value = vertices[n].g + e->getEstimatedRequiredSamples();
				if(value < minValue) {
					minValue = value;
				}
			}
			s.rhs = minValue;
		}

		if(U.inHeap(&vertices[id])) {
			U.remove(&vertices[id]);
		}

		if(s.g != s.rhs) {
			s.key = calculateKey(id);
			U.push(&vertices[id]);
		}
	}

	void computeShortestPath() {
		while(!U.isEmpty()) {
			Vertex &u = vertices[U.pop()->id];
			Key k_old = u.key;
			Key k_new = calculateKey(u.id);

			if(k_old < k_new) {
				u.key = k_new;
				U.push(&vertices[u.id]);
			}
			else if(u.g > u.rhs) {
				u.g = u.rhs;
				for(auto e : reverseEdges[u.id]) {
					if(e.second->interior) {
						updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
					}
					else {
						updateEdgeEffort(e.second, u.g + e.second->getEstimatedRequiredSamples(), false);
					}
				}

				auto neighbors = getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			} else {
				u.g = std::numeric_limits<double>::infinity();

				for(auto e : reverseEdges[u.id]) {
					if(e.second->interior) {
						updateEdgeEffort(e.second, getInteriorEdgeEffort(e.second), false);
					}
					else {
						updateEdgeEffort(e.second, u.g, false);
					}
				}

				updateVertex(u.id);
				auto neighbors = abstraction->getNeighboringCells(u.id);
				for(auto n : neighbors) {
					updateVertex(n);
				}
			}
		}
	}

	bool addedGoalEdge;
};

}

}
