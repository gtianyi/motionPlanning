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

    virtual void initialize() override{
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

    virtual void initializeRegions(const size_t regionCount) override{
        initializeGridParameters(regionCount);

        generateRegions(adjustedRegionCount,gridSize,units);

		bindStartGoalRegions();

        connectAllRegions();

#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
    }

    virtual void sampleFullState(
            const IntegratedBeastBase::Region* samplingRegion,
            State* to) override {
        to = spaceInformation->allocState();
        fullStateSampler->sampleUniform(to);

        const Region* r = static_cast<const Region*>(samplingRegion);

        std::vector<double> abstractVector(dimensions);

        for (int i = 0, j = 0; i < dimensions; ++i) {
            abstractVector[i] =
                    randomNumbers.uniformReal(r->bounds[j], r->bounds[j + 1]);
            j += 2;
        }

        copyVectorToFullState(to, abstractVector);

        //std::cout << "target region: ";
		//printVector(r->gridIndices);
        //std::cout << "target bounds: ";
		//printVector(r->bounds);
		//spaceInformation->printState(to);
    }

    virtual IntegratedBeastBase::Region* addStateToClosestRegion(
            State* state) override {
        int index = findRegionIndex(state);

        auto region =regions[index] ;

        //auto regionGrid = static_cast<Region*>(region);
        //std::cout << "reached region:" << std::endl;
        //printVector(regionGrid->gridIndices);
        //std::cout << "target region:" << std::endl;
        //auto targetRegionGrid =
        //        static_cast<Region*>(lastSelectedEdge->targetRegion);
        //printVector(targetRegionGrid->gridIndices);
        //std::cout << "source region:" << std::endl;
        //auto sourceRegionGrid =
        //        static_cast<Region*>(lastSelectedEdge->sourceRegion);
        //printVector(sourceRegionGrid->gridIndices);
        //std::cout << "start region:" << std::endl;
        //auto startRegionGrid = static_cast<Region*>(regions[startRegionId]);
        //printVector(startRegionGrid->gridIndices);

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

        abstractLowBounds = globalParameters.abstractBounds.low;
    }

    void generateRegions(const unsigned int adjustedRegionCount,
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
            regionBounds[++j] = offsets[i] + (gridIndices[i] + 1) * units[i];
        }

        auto state = abstractSpace->allocState();
        //globalParameters.copyVectorToAbstractState(state, regionCenter);
		//abstractSampler->sample(state);
        copyVectorToAbstractState(state, regionCenter);

        auto region = new Region(index, state, regionBounds, gridIndices);

        return region;
    }

    std::vector<unsigned int> getGridIndicesFrom1DIndex(
            const unsigned int index) const {
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

    int get1DIdexFromGridIndices(std::vector<unsigned int> indices) const {
        int index = 0;

        for (int i = 0; i < dimensions; ++i) {
            index += indices[i] * pow((float)gridSize, (float)i);
        }

        return index;
	}

    void bindStartGoalRegions() {
        // bind start region
        startRegionId = findRegionIndex(fullStartState);

        // Add startState to startRegion as a root for the motion tree
        auto startRegion = regions[startRegionId];
        startRegion->addState(fullStartState);

        // bind goal region
        goalRegionId = findRegionIndex(fullGoalState);
    }

    virtual void connectAllRegions() override{
        clearAllEdges();

        for (auto& region : regions) {
            region->outEdges.clear();
            region->inEdges.clear();
        }

        for (auto& region : regions) {
            Region* r = static_cast<Region*>(region);
            connectToGridNeighbors(r);
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
			assert(index<regions.size());
			Region* r= static_cast<Region*>(regions[index]);
			neighbors.push_back(r);
        }
		
		return neighbors;
    }

    std::vector<std::vector<unsigned int>> getNeighborsIndicesByRegionIndices(
            std::vector<unsigned int> regionIndices) {
        std::vector<std::vector<unsigned int>> neighborsIndices;

        for (int i = 0; i < dimensions; ++i) {
            for (int j = regionIndices[i] - 1; j <= int(regionIndices[i] + 1);
                    j++) {
                if (j < 0 || j >= gridSize) {
                    continue;
                }

                // incrementally add neighbor for previous dimensions
                std::vector<std::vector<unsigned int>> newNeighborsIndices;
                for (auto n : neighborsIndices) {
                    std::vector<unsigned int> neighbor = n;
                    neighbor[i] = (unsigned int)j;
                    if (neighbor != regionIndices &&
                            std::find(neighborsIndices.begin(),
                                    neighborsIndices.end(),
                                    neighbor) == neighborsIndices.end() &&
                            std::find(newNeighborsIndices.begin(),
                                    newNeighborsIndices.end(),
                                    neighbor) == newNeighborsIndices.end()) {
                        newNeighborsIndices.push_back(neighbor);
                    }
                }

                neighborsIndices.insert(neighborsIndices.end(),
                        newNeighborsIndices.begin(),
                        newNeighborsIndices.end());

                // add neighbor for current dimension
                std::vector<unsigned int> neighbor = regionIndices;
                neighbor[i] = (unsigned int)j;
                if (neighbor != regionIndices &&
                        std::find(neighborsIndices.begin(),
                                neighborsIndices.end(),
                                neighbor) == neighborsIndices.end()) {
                    neighborsIndices.push_back(neighbor);
                }
            }
        }
        //std::cout << "neighborSize" << neighborsIndices.size() << std::endl;
        //std::cout << "region:" << std::endl;
        //printVector(regionIndices);
        //std::cout << "neighbors:" << std::endl;
        //for (auto i : neighborsIndices) {
        //    printVector(i);
		//}
        return neighborsIndices;
    }

	template <class T>
    void printVector(const std::vector<T>& v) {
        for (auto i : v) {
            std::cout << i << " ";
        }
        std::cout << "\n";
    }

    /**
     * Steps of region splitting:
     *
     * 1. Sample new region around the old region
     * 2. Redistribute the states
     * 3.
     * @param originalRegion
     */
    virtual void splitRegion(
            IntegratedBeastBase::Region* originalRegion) override {
        // auto distanceFunction = nearestRegions->getDistanceFunction();

        // double minDistance = std::numeric_limits<double>::max();
        // for (auto outEdge : originalRegion->outEdges) {
        //     auto distance =
        //             distanceFunction(originalRegion,
        //             outEdge->targetRegion);

        //     minDistance = std::min(minDistance, distance);
        // }

        // auto state =
        //         sampleAbstractState(originalRegion, std::max(1.0,
        //         minDistance));
        // auto newRegion = allocateRegion(state);

        // addKNeighbors(newRegion, neighborEdgeCount, true);
        // nearestRegions->add(newRegion);
        // inconsistentRegions.push(newRegion);
    }

    int findRegionIndex(const State* state) const {
        std::vector<double> abstractStateVector = getAbstractStateVector(state);
        std::vector<unsigned int> gridIndices;

        for (int i = 0; i < dimensions; ++i) {
            //int index = getIndexInOneDimension(
            //        abstractStateVector[i], allRegionLowerBounds[i]);
            int index =
                    (abstractStateVector[i] - abstractLowBounds[i]) / units[i];
            assert(index >= 0);
            assert(index < gridSize);
            gridIndices.push_back(index);
        }

        int index = get1DIdexFromGridIndices(gridIndices);

		return index;
    }

    std::vector<double> getAbstractStateVector(const State* state) const {
        std::vector<double> abstractStateVector;

        if (dimensions == 2) {
            auto s = state->as<ompl::base::CompoundStateSpace::StateType>()
                             ->as<ompl::base::SE2StateSpace::StateType>(0);
			abstractStateVector.push_back(s->getX());
			abstractStateVector.push_back(s->getY());
        } else if (dimensions == 3) {
            auto s = state->as<ompl::base::CompoundStateSpace::StateType>()
                             ->as<ompl::base::SE3StateSpace::StateType>(0);
            abstractStateVector.push_back(s->getX());
            abstractStateVector.push_back(s->getY());
            abstractStateVector.push_back(s->getZ());
        } else {
            throw ompl::Exception("IntegratedBeastGrid::getAbstractStateVector",
                    "Not a 2D or 3D abstract space");
        }

        return abstractStateVector;
    }

    int getIndexInOneDimension(double target, std::vector<double> nums) const {
        // binary search
        int low = 0, high = nums.size() - 1;

        if (target > nums[nums.size() - 1]) {
            int lastRegionIndex = nums.size() - 1;
            return lastRegionIndex;
        }

        while (low <= high) {
            int mid = (low + high) / 2;
            if (nums[mid] == target)
                // if on boundry, return the smaller region
                return mid - 1;
            if (low == high)
                return low - 1;
            if (target > nums[mid])
                low = mid + 1;
            else
                high = mid;
        }
    }

    void copyVectorToFullState(State* state, const std::vector<double>& values) {
        if (dimensions == 2) {
            auto s = state->as<ompl::base::CompoundStateSpace::StateType>()
                             ->as<ompl::base::SE2StateSpace::StateType>(0);
            s->setXY(values[0], values[1]);
        } else if (dimensions == 3) {
            auto s = state->as<ompl::base::CompoundStateSpace::StateType>()
                             ->as<ompl::base::SE3StateSpace::StateType>(0);
            s->setXYZ(values[0], values[1], values[2]);
        } else {
            throw ompl::Exception("IntegratedBeastGrid::copyVectorToState",
                    "Not a 2D or 3D abstract space");
        }
    }

    void copyVectorToAbstractState(State* state,
            const std::vector<double>& values) {
        if (dimensions == 2) {
            auto s = state->as<ompl::base::SE2StateSpace::StateType>();
            s->setXY(values[0], values[1]);
        } else if (dimensions == 3) {
            auto s = state->as<ompl::base::SE3StateSpace::StateType>();
            s->setXYZ(values[0], values[1], values[2]);
        } else {
            throw ompl::Exception("IntegratedBeastGrid::copyVectorToState",
                    "Not a 2D or 3D abstract space");
        }
    }

    unsigned int dimensions;
    unsigned int gridSize;
    unsigned int adjustedRegionCount;
    std::vector<double> units;
    std::vector<double> abstractLowBounds;

	ompl::RNG randomNumbers;
};
