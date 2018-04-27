#include <boost/bind.hpp>
#include "structs/utils.hpp"

GlobalParameters globalParameters;

#include <flann/flann.h>

#include "configuration/configuration.hpp"
#include "structs/filemap.hpp"

//#include "planners/atemptsplanner.hpp"

void doBenchmarkRun(const BenchmarkData& benchmarkData, const FileMap& params) {
    auto plannerPointer = getPlanner(benchmarkData, params);

    // allow unpenalized time for precomputation -- which is logged to the
    // output file
    //  plannerPointer->setProblemDefinition(benchmarkData.simplesetup->getProblemDefinition());
    // plannerPointer->setProblemDefinition(benchmarkData.gsetup->getProblemDefinition());
    //  plannerPointer->solve(0);

    if (params.boolVal("AddIntermediateStates") &&
            plannerPointer->params().hasParam("intermediate_states")) {
        plannerPointer->params().setParam("intermediate_states", "true");
    }
    benchmarkData.benchmark->addPlanner(plannerPointer);
    ompl::tools::Benchmark::Request req;
    req.maxTime = params.doubleVal("Timeout");
    req.maxMem = params.doubleVal("Memory");
    req.runCount = params.doubleVal("Runs");
    req.displayProgress = true;
    req.saveConsoleOutput = false;

    std::cout << "Do benchmark\n";

    benchmarkData.benchmark->benchmark(req);

    // save ompl output file to some place,
    // we don't need this for new experiment system
    // Mar - 11 - 2018,  by Tianyi Gu
    // benchmarkData.benchmark->saveResultsToFile(params.stringVal("Output").c_str());

    std::cout << "Results: \n";

    benchmarkData.benchmark->saveResultsToStream();

	std::cout<<globalParameters.solutionStream.propagationInfo<<"\n";

    std::cout << std::endl;

    // If there were multiple solutions being logged to global parameters,
    // append them to the output file
    // for some reason the OMPL benchmarking doesn't include all the solutions
    // added, just the final one
    if (globalParameters.solutionStream.solutions.size() > 0) {
        std::ofstream outfile;
        outfile.open(params.stringVal("Output").c_str(), std::ios_base::app);

        outfile << "Solution Stream\n";

        for (const auto& solution : globalParameters.solutionStream.solutions) {
            outfile << solution.second << " " << solution.first.value() << "\n";
        }
        outfile.close();
    }
}

int main(int argc, char* argv[]) {
    FileMap params;
    if (argc > 1) {
        params.append(argv[1]);
    } else {
        params.append(std::cin);
    }

    const auto val = static_cast<unsigned int>(params.integerVal("Seed"));
    srand(val);
    flann::seed_random(val);
    ompl::RNG::setSeed(val);

    if (params.exists("NumControls"))
        howManyControls = params.integerVal("NumControls");

    const BenchmarkData benchmarkData = getBenchmarkData(params);
    doBenchmarkRun(benchmarkData, params);
    return 0;
}
