#pragma once

#include "abstraction.hpp"
#include "prmlite.hpp"
#include "../../structs/filemap.hpp"
#include "../../structs/utils.hpp"

class PRMLiteGrid: public PRMLite {
public:
    PRMLiteGrid(const ompl::base::SpaceInformation* si,
                  const ompl::base::State* start,
                  const ompl::base::State* goal,
                  const FileMap& params) : PRMLite(si, start, goal, params) {}

    virtual ~PRMLiteGrid() = default;

protected:
    virtual void generateVertices() override {
        Timer timer("Vertex Generation");
        ompl::base::StateSpacePtr abstractSpace =
                globalParameters.globalAbstractAppBaseGeometric
                        ->getStateSpace();
        ompl::base::ValidStateSamplerPtr abstractSampler =
                globalParameters.globalAbstractAppBaseGeometric
                        ->getSpaceInformation()
                        ->allocValidStateSampler();

		//scott do some spetical thing on bound size see catsetup.hpp:227
		//and quadrotor.hpp:161
        const auto dimensions =
                globalParameters.abstractBounds.low.size() == 6 ? 3 : 2;
        const auto gridSize = (int)ceil(pow((float)prmSize, 1.0 / dimensions));
        const auto adjustedPRMSize = (int)pow((float)gridSize, dimensions);

        vertices.resize(adjustedPRMSize+2);

        // Add start region
        vertices[0] = new Vertex(0);
        vertices[0]->state = abstractSpace->allocState();
        abstractSpace->copyState(vertices[0]->state, start);
        nn->add(vertices[0]);

        // Add goal region
        vertices[1] = new Vertex(1);
        vertices[1]->state = abstractSpace->allocState();
        abstractSpace->copyState(vertices[1]->state, goal);
        nn->add(vertices[1]);

        std::vector<double> units(dimensions);
        for (int i = 0; i < dimensions; ++i) {
            units[i] = (globalParameters.abstractBounds.high[i] -
                               globalParameters.abstractBounds.low[i]) /
                    gridSize;
        }

        // Sample regions
        for (unsigned int i = 0; i < adjustedPRMSize; ++i) {
                    auto vertex = new Vertex(i);
                    vertex->state = abstractSpace->allocState();

                    auto grid_vector = get_grid_vector(units,
                            globalParameters.abstractBounds.low,
                            i,
                            gridSize);
                    globalParameters.copyVectorToAbstractState(
                            vertex->state, grid_vector);

                    vertices[i] = vertex;
                    nn->add(vertex);
        }
    }

private:
    std::vector<double> get_grid_vector(const std::vector<double>& units,
            const std::vector<double>& offsets,
            const int index,
			const int gridSize) {
        std::vector<int> dimensionIndex(units.size(), 0);
        std::vector<double> abstractVertex(units.size());

        int residualIndex = index;
        for (int i = units.size()-1; i >=0 ; --i) {
            dimensionIndex[i] =
                    int(residualIndex / pow((float)gridSize, (float)i));
            abstractVertex[i] = offsets[i] + dimensionIndex[i] * units[i];

            residualIndex -=
                    dimensionIndex[i] * (int)pow((float)gridSize, (float)i);
        }

        return abstractVertex;
    }
};
