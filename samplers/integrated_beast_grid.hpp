#pragma once

#include <math.h>
#include <ompl/base/Goal.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <vector>
#include "../structs/filemap.hpp"
#include "../structs/inplacebinaryheap.hpp"
#include "../structs/utils.hpp"
#include "integrated_beast_base.hpp"

class IntegratedBeastGrid: public IntegratedBeastBase {
public:
    IntegratedBeastGrid(const ompl::base::SpaceInformation* spaceInformation,
            const ompl::base::State* start,
            const ompl::base::GoalPtr& goalPtr,
            ompl::base::GoalSampleableRegion* goalSampleableRegion,
            const FileMap& params)
			 : IntegratedBeastBase(spaceInformation,
							  start,
							  goalPtr,
							  goalSampleableRegion,
							  params){
         }

    virtual void initialize() {
        goalEdgeTimer = new Timer("Goal edge timer");


        fullStateSampler = spaceInformation->allocStateSampler();

        abstractSpace = globalParameters.globalAbstractAppBaseGeometric
                                ->getStateSpace();
        abstractSampler = globalParameters.globalAbstractAppBaseGeometric
                                  ->getSpaceInformation()
                                  ->allocValidStateSampler();

        initializeRegions(initialRegionCount);

        regions[goalRegionId]->rhs = 0;
        regions[goalRegionId]->key = regions[goalRegionId]->calculateKey();
        inconsistentRegions.push(regions[goalRegionId]);

        computeShortestPath();
        addOutgoingEdgesToOpen(regions[startRegionId]);
    }

private:
    struct Region : IntegratedBeastBase::Region {
        Region(unsigned int id,
                ompl::base::State* state,
                std::vector<double> bounds,
                std::vector<unsigned int> gridIndices)
                : IntegratedBeastBase::Region(id, state),
                  bounds(bounds),
                  gridIndices(gridIndices) {}

        const std::vector<double> bounds;
        const std::vector<unsigned int> gridIndices;
    };

    virtual void initializeRegions(const size_t regionCount) {
        initializeGridParameters(regionCount);

        generateRegions(adjustedRegionCount,gridSize,units);

		bindStartGoalRegions();

        connectAllRegions();

#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
    }

    virtual void sampleFullState(const Region* samplingRegion, State* to) {
			//TODO fix
    }

    State* sampleAbstractState(const Region* samplingRegion,
            const double samplingRadius) {
        auto state = abstractSpace->allocState();
        abstractSampler->sampleNear(state, samplingRegion->state, stateRadius);

        auto sampleHostRegion = findRegion(state);

        return state;
    }

    virtual Region* addStateToClosestRegion(State* state) {
        ompl::base::ScopedState<> incomingState(
                spaceInformation->getStateSpace());
        incomingState = state;
        auto region = findRegion(incomingState);

        region->addState(state);

        return region;
    }

    void initializeGridParameters(const size_t regionCount) {
        // get dimension number
        // scott do some spetical thing on bound size see catsetup.hpp:227
        // and quadrotor.hpp:161
        dimensions = globalParameters.abstractBounds.low.size() == 6 ? 3 : 2;
        // eg: we want 2000 regions in 3D
        // then we get ceil(pow(2000, 1/3))=13
        // which means we split each dimension into 13 segment
        gridSize = (int)ceil(pow((float)regionCount, 1.0 / dimensions));
        // eg: 13^3=2197, which is adjusted region count.
        adjustedRegionCount = (int)pow((float)gridSize, dimensions);

        units.resize(dimensions);

        for (int i = 0; i < dimensions; ++i) {
            units[i] = (globalParameters.abstractBounds.high[i] -
                               globalParameters.abstractBounds.low[i]) /
                    gridSize;
        }
    }

    virtual void generateRegions(const unsigned int adjustedRegionCount,
            const unsigned int gridSize,
            const std::vector<double>& units) {
        for (unsigned int i = 0; i < adjustedRegionCount; ++i) {
            auto region = allocateRegion(
                    units, globalParameters.abstractBounds.low, i, gridSize);
            regions.push_back(region);
        }
    }

    Region* allocateRegion(const std::vector<double>& units,
            const std::vector<double>& offsets,
            const unsigned int index,
            const int gridSize) {
        std::vector<unsigned int> gridIndices =
                getGridIndicesFrom1DIndex(index);
        std::vector<double> regionBounds(dimensions * 2);
        std::vector<double> regionCenter(dimensions);

        for (int i = 0, j = 0; i < dimensions; i++, j++) {
            regionCenter[i] = offsets[i] + (gridIndices[i] + 0.5) * units[i];
            regionBounds[j] = offsets[i] + gridIndices[i] * units[i];
            regionBounds[j++] = offsets[i] + (gridIndices[i] + 1) * units[i];
        }

        auto state = abstractSpace->allocState();
        globalParameters.copyVectorToAbstractState(state, regionCenter);

        auto region = new Region(index, state, regionBounds, gridIndices);

        return region;
    }

    std::vector<unsigned int> getGridIndicesFrom1DIndex(
            const unsigned int index) {
        std::vector<unsigned int> gridIndices(dimensions, 0);

        unsigned int residualindex = index;
        for (int i = dimensions-1; i >=0 ; --i) {
            gridIndices[i] =
                    int(residualindex / pow((float)gridSize, (float)i));

            residualindex -=
                    gridIndices[i] * (int)pow((float)gridSize, (float)i);
        }

        return gridIndices;
    }

    int get1DIdexFromGridIndices(std::vector<unsigned int> indices) {
        int index = 0;

        for (int i = 0; i < dimensions; ++i) {
            index += indices[i] * pow((float)gridSize, (float)i);
        }

        return index;
	}

    void bindStartGoalRegions() {
        // bind start region
        startRegionId = findRegionIndex(abstractStartState);

        // Add startState to startRegion as a root for the motion tree
        auto startRegion = regions[startRegionId];
        startRegion->addState(fullStartState);

        // bind goal region
        goalRegionId = findRegionIndex(abstractGoalState);
    }

    virtual void connectAllRegions() {
        clearAllEdges();

        for (auto& region : regions) {
            region->outEdges.clear();
            region->inEdges.clear();
        }

        for (auto& region : regions) {
            connectToGridNeighbors(region);
        }
    }

    void connectToGridNeighbors(Region* sourceRegion) {

        // Ignore goal region as a source
        if (goalRegionId == sourceRegion->id)
            return;

        std::vector<Region*> neighbors = getGridNeighbors(sourceRegion);

        for (auto neighborRegion : neighbors) {
            connectRegions(sourceRegion, neighborRegion);
        }
    }

    std::vector<Region*> getGridNeighbors(Region* region) {
        std::vector<std::vector<unsigned int>> neighborsIndices =
                getNeighborsIndicesByRegionIndices(region->gridIndices);

        std::vector<Region*> neighbors;

        for (auto& i : neighborsIndices) {
            int index = get1DIdexFromGridIndices(i);
			neighbors.push_back(regions[index]);
        }
		
		return neighbors;
    }

    std::vector<std::vector<unsigned int>> getNeighborsIndicesByRegionIndices(
            std::vector<unsigned int> regionIndices) {
        std::vector<std::vector<unsigned int>> neighborsIndices;

        for (int i = 0; i < dimensions; ++i) {
            for (unsigned int j = regionIndices[i] - 1;
                    j <= regionIndices[i] + 1;
                    j++) {
                if (j < 1 || j > gridSize) {
                    continue;
                }

                // incrementally add neighbor for previous dimensions
                std::vector<std::vector<unsigned int>> newNeighborsIndices;
                for (auto n : neighborsIndices) {
                    std::vector<unsigned int> neighbor = n;
                    neighbor[i] = j;
                    if (neighbor != regionIndices) {
                        newNeighborsIndices.push_back(neighbor);
                    }
                }

                neighborsIndices.insert(neighborsIndices.end(),
                        newNeighborsIndices.begin(),
                        newNeighborsIndices.end());

                // add neighbor for current dimension
                std::vector<unsigned int> neighbor = regionIndices;
                neighbor[i] = j;
                if (neighbor != regionIndices) {
                    neighborsIndices.push_back(neighbor);
                }
            }
        }
		return neighborsIndices;
	}

    /**
     * Steps of region splitting:
     *
     * 1. Sample new region around the old region
     * 2. Redistribute the states
     * 3.
     * @param originalRegion
     */
    virtual void splitRegion(Region* originalRegion) {
        auto distanceFunction = nearestRegions->getDistanceFunction();

        double minDistance = std::numeric_limits<double>::max();
        for (auto outEdge : originalRegion->outEdges) {
            auto distance =
                    distanceFunction(originalRegion, outEdge->targetRegion);

            minDistance = std::min(minDistance, distance);
        }

        auto state =
                sampleAbstractState(originalRegion, std::max(1.0, minDistance));
        auto newRegion = allocateRegion(state);

        addKNeighbors(newRegion, neighborEdgeCount, true);
        nearestRegions->add(newRegion);
        inconsistentRegions.push(newRegion);
    }

    int findRegionIndex(const State* state) const {
        Region region(std::numeric_limits<RegionId>::max(), state);
        return nearestRegions->nearest(&region);
    }

    RegionId startRegionId;
    RegionId goalRegionId;

    std::vector<std::vector<double>> regionBounds;

    unsigned int dimensions;
    unsigned int gridSize;
    unsigned int adjustedRegionCount;
    std::vector<double> units;

};
