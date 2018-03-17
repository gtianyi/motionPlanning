#pragma once

#include "abstraction.hpp"
#include "prmlite.hpp"
#include "../../structs/filemap.hpp"
#include "../../structs/utils.hpp"

class PRMLiteHalton: public PRMLite {
public:
    PRMLiteHalton(const ompl::base::SpaceInformation* si,
                  const ompl::base::State* start,
                  const ompl::base::State* goal,
                  const FileMap& params) : PRMLite(si, start, goal, params) {}

    virtual ~PRMLiteHalton() = default;

protected:
    virtual void generateVertices() override {
        Timer timer("Vertex Generation");
        ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
        ompl::base::ValidStateSamplerPtr abstractSampler =
            globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();

        vertices.resize(prmSize);

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

        const auto dimensions = globalParameters.abstractBounds.low.size();
        std::vector<double> ranges(dimensions);
        for (int i = 0; i < dimensions; ++i) {
            ranges[i] = globalParameters.abstractBounds.high[i] - globalParameters.abstractBounds.low[i];
        }

        // Sample regions
        for (unsigned int i = 2; i < prmSize; ++i) {
            auto vertex = new Vertex(i);
            vertex->state = abstractSpace->allocState();

            auto halton_vector = sample_halton_vector(ranges, globalParameters.abstractBounds.low, i);
            globalParameters.copyVectorToAbstractState(vertex->state, halton_vector);

            vertices[i] = vertex;
            nn->add(vertex);
        }
    }

private:
    std::vector<double>
    sample_halton_vector(const std::vector<double>& ranges, const std::vector<double>& offsets, const int index) {
        static constexpr int primes[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53};

        std::vector<double> samples(ranges.size());

        for (int i = 0; i < samples.size(); ++i) {
            const double halton_number = halton_sequence(index, primes[i]);

            // Project the 0-1 sample to the bounds
            samples[i] = offsets[i] + halton_number * ranges[i];
        }

        return samples;
    }

    double halton_sequence(int index, int base) const {
        double f = 1;
        double r = 0;

        while (index > 0) {
            f /= base;
            r += f * (index % base);
            index /= base;
        }

        return r;
    }
};