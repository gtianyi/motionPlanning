#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include "../structs/halton_sequence.hpp"

class HaltonSampler {
public:
    HaltonSampler(ompl::base::RealVectorBounds bounds)
            : ranges(bounds.low.size()),
              offsets{bounds.low.begin(), bounds.low.end()} {
        const auto dimensions = bounds.low.size();
        for (int i = 0; i < dimensions; ++i) {
            ranges[i] = bounds.high[i] - bounds.low[i];
        }
    }

    std::vector<double> sampleVector(const int index) const {
        return sampleHaltonVector(ranges, offsets, index);
    }

private:
    static double haltonSequence(int index, int base) {
        double f = 1;
        double r = 0;

        while (index > 0) {
            f /= base;
            r += f * (index % base);
            index /= base;
        }

        return r;
    }

    static std::vector<double> sampleHaltonVector(
            const std::vector<double>& ranges,
            const std::vector<double>& offsets,
            const int index) {
        static constexpr int primes[] = {
                2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53};

        std::vector<double> samples(ranges.size());

        for (int i = 0; i < samples.size(); ++i) {
            const double halton_number = haltonSequence(index, primes[i]);

            // Project the 0-1 sample to the bounds
            samples[i] = offsets[i] + halton_number * ranges[i];
        }

        return samples;
    }

    std::vector<double> ranges;
    std::vector<double> offsets;
};
