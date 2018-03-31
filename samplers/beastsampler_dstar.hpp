#pragma once

#include "beastsamplerbase.hpp"

#ifdef STREAM_GRAPH
#include "../dependencies/httplib.hpp"
#endif

namespace ompl {

namespace base {

class BeastSampler_dstar : public ompl::base::BeastSamplerBase {
public:
    BeastSampler_dstar(ompl::base::SpaceInformation* base,
            ompl::base::State* start,
            const ompl::base::GoalPtr& goal,
            base::GoalSampleableRegion* gsr,
            const FileMap& params)
            : BeastSamplerBase(base, start, goal, gsr, params),
              addedGoalEdge(false) {}

    ~BeastSampler_dstar() {}

    virtual void initialize() {
        BeastSamplerBase::initialize();

        unsigned int abstractionSize = abstraction->getAbstractionSize();
        vertices.reserve(abstractionSize);

        for (unsigned int i = 0; i < abstractionSize; ++i) {
            vertices.emplace_back(i);
            auto neighbors = abstraction->getNeighboringCells(i);
            for (auto n : neighbors) {
                getEdge(i, n);
                getEdge(n, i);
            }
        }

        vertices[goalID].rhs = 0;
        vertices[goalID].key = calculateKey(goalID);
        U.push(&vertices[goalID]);

        {
            Timer t("D* lite");
            computeShortestPath();
        }

        for (auto eset : edges) {
            for (auto e : eset.second) {
                e.second->initialEffort = e.second->effort;
            }
        }

        vertices[startID].addState(startState);
        addOutgoingEdgesToOpen(startID);

#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
    }

    virtual bool sample(ompl::base::State* from, ompl::base::State* to) {
#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
#ifdef STREAM_GRAPHICS
        static unsigned int sampleCount = 0;
        if (sampleCount++ % 100 == 0) {
            // fprintf(stderr, "open: %u\n", open.getFill());
            // writeVertexFile(sampleCount / 1000);
            // writeOpenEdgeFile(sampleCount / 1000);
            // writeUpdatedEdgeFile(sampleCount / 10);
            writeEdgeFile(sampleCount / 100, 1);
        }
#endif

        if (targetEdge != NULL) { // only will fail the first time through

            if (targetSuccess) {
                if (!addedGoalEdge && targetEdge->endID == goalID) {
                    Edge* goalEdge = new Edge(goalID, goalID);
                    goalEdge->updateEdgeStatusKnowledge(
                            Abstraction::Edge::VALID);
                    goalEdge->effort = 1;
                    open.push(goalEdge);
                    addedGoalEdge = true;
                }

                if (targetEdge->interior) {
                    updateSuccesfulInteriorEdgePropagation(targetEdge);
                    updateEdgeEffort(
                            targetEdge, getInteriorEdgeEffort(targetEdge));
                } else {
                    // edge has become interior
                    targetEdge->interior = true;
                    targetEdge->succesfulPropagation();
                    updateEdgeEffort(
                            targetEdge, getInteriorEdgeEffort(targetEdge));
                }
            } else {
                targetEdge->failurePropagation();
                updateEdgeEffort(targetEdge,
                        targetEdge->getEstimatedRequiredSamples() +
                                vertices[targetEdge->endID].g);
            }

            updateVertex(targetEdge->startID);
            computeShortestPath();

            if (targetSuccess) {
                addOutgoingEdgesToOpen(targetEdge->endID);
            }
        }

        bool getNextEdge = true;
        while (getNextEdge) {
            assert(!open.isEmpty());

            targetEdge = open.peek();

            if (targetEdge->status == Abstraction::Edge::UNKNOWN) {
                Abstraction::Edge::CollisionCheckingStatus status =
                        abstraction->isValidEdge(
                                targetEdge->startID, targetEdge->endID) ?
                        Abstraction::Edge::VALID :
                        Abstraction::Edge::INVALID;
                targetEdge->updateEdgeStatusKnowledge(status);

                // yes this looks weird but we need it for right now to do some
                // debugging
                updateEdgeEffort(targetEdge, targetEdge->effort);

                updateVertex(targetEdge->startID);
                computeShortestPath();
            } else {
                getNextEdge = false;
            }
        }

        targetSuccess = false;

        if (targetEdge->startID == targetEdge->endID &&
                targetEdge->startID == goalID) {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            goalSampler->sampleGoal(to);
        } else {
            si_->copyState(from, vertices[targetEdge->startID].sampleState());
            ompl::base::ScopedState<> vertexState(
                    globalParameters.globalAppBaseControl
                            ->getGeometricComponentStateSpace());
            // guty: need to use globalAppBaseGeometric for linkage
            if (abstraction->supportsSampling()) {
                vertexState =
                        abstraction->sampleAbstractState(targetEdge->endID);
            } else {
                vertexState = abstraction->getState(targetEdge->endID);
                ompl::base::ScopedState<> fullState =
                        globalParameters.globalAppBaseControl
                                ->getFullStateFromGeometricComponent(
                                        vertexState);
                fullStateSampler->sampleUniformNear(
                        to, fullState.get(), stateRadius);
                // guty: add a sampler for linkage here
            }
        }

        return true;
    }

    virtual bool sample(ompl::base::State*) {
        throw ompl::Exception("NewSampler::sample", "not implemented");
    }

    virtual bool sampleNear(ompl::base::State*,
            const ompl::base::State*,
            const double) {
        throw ompl::Exception("NewSampler::sampleNear", "not implemented");
    }

    void reached(ompl::base::State* state) {
        ompl::base::ScopedState<> incomingState(si_->getStateSpace());
        incomingState = state;
        unsigned int newCellId =
                abstraction->mapToAbstractRegion(incomingState);

        vertices[newCellId].addState(state);

        // if the planner chose the goal region first be careful not to
        // dereference a null pointer
        if (targetEdge != NULL && newCellId == targetEdge->endID) {
            // this region will be added to open when sample is called again
            targetSuccess = true;
        } else {
            addOutgoingEdgesToOpen(newCellId);
        }
    }

protected:
    void vertexMayBeInconsistent(unsigned int id) { updateVertex(id); }

    void vertexHasInfiniteValue(unsigned int id) { computeShortestPath(); }

    Key calculateKey(unsigned int id) {
        Vertex& s = vertices[id];
        Key key;
        key.first = std::min(s.g, s.rhs);
        key.second = std::min(s.g, s.rhs);
        return key;
    }

    void updateVertex(unsigned int id) {
        Vertex& s = vertices[id];
        if (s.id != goalID) {
            double minValue = std::numeric_limits<double>::infinity();
            auto neighbors = abstraction->getNeighboringCells(id);
            for (auto n : neighbors) {
                Edge* e = getEdge(id, n);
                double value = vertices[n].g + e->getEstimatedRequiredSamples();
                if (value < minValue) {
                    minValue = value;
                }
            }
            s.rhs = minValue;
        }

        if (U.inHeap(&vertices[id])) {
            U.remove(&vertices[id]);
        }

        if (s.g != s.rhs) {
            s.key = calculateKey(id);
            U.push(&vertices[id]);
        }
    }

#ifdef STREAM_GRAPH
    void publishAbstractGraph() {
        static int counter = -1;
        ++counter;
        counter %= 100;
        if (counter > 0) {
            return;
        }

        std::cout << "Graph test" << std::endl;
        httplib::Client cli("localhost", 8080, 300, httplib::HttpVersion::v1_1);

        std::ostringstream commandBuilder;

        const unsigned int dimension =
                globalParameters.globalAbstractAppBaseGeometric
                        ->getSpaceInformation()
                        .get()
                        ->getStateDimension();

        for (auto vertex : vertices) {
            auto region = vertex.id;
            auto state = abstraction->getState(region);
            auto vectorState =
                    state->as<ompl::base::CompoundStateSpace::StateType>()
                            ->as<ompl::base::RealVectorStateSpace::StateType>(
                                    0);

            commandBuilder << "{\"" << (vertex.alreadyVisualized ? "cn" : "an")
                           << "\":{\"" << region << "\":{"
                           << "\"label\":\""
                           << "g: " << vertex.g << "\""
                           << ",\"size\":2"
                           << ",\"x\":" << vectorState->values[0] * 100
                           << ",\"y\":" << vectorState->values[1] * 100
                           << ",\"z\":"
                           << (dimension == 3 ? vectorState->values[2] * 100 :
                                                0)
                           << "}}}\r\n";

            vertex.alreadyVisualized = true;
        }

        for (auto keypair : edges) {
            const unsigned int source = keypair.first;
            auto outgoing = keypair.second;
            for (auto outgoing_keypair : outgoing) {
                const unsigned int target = outgoing_keypair.first;
                Edge* const edge = outgoing_keypair.second;

                commandBuilder << "{\""
                               << (edge->alreadyVisualized ? "ce" : "ae")
                               << "\":{\"" << edge << "\":{"
                               << "\"source\":\"" << source << "\","
                               << "\"target\":\"" << target << "\","
                               << "\"directed\":true,"
                               << "\"label\":\"Te: " << edge->effort << "\","
                               << "\"weight\":\"" << edge->alpha + edge->beta
                               << "\"}}}\r\n";
                edge->alreadyVisualized = true;
            }
        }

        for (auto keypair : reverseEdges) {
            const unsigned int source = keypair.first;
            auto outgoing = keypair.second;
            for (auto outgoing_keypair : outgoing) {
                const unsigned int target = outgoing_keypair.first;
                Edge* const edge = outgoing_keypair.second;

                commandBuilder << "{\""
                               << (edge->alreadyVisualized ? "ce" : "ae")
                               << "\":{\"" << edge << "\":{"
                               << "\"source\":\"" << source << "\","
                               << "\"target\":\"" << target << "\","
                               << "\"directed\":true,"
                               << "\"label\":\"Te: " << edge->effort << "\","
                               << "\"weight\":\"" << edge->alpha + edge->beta
                               << "\"}}}\r\n";
                edge->alreadyVisualized = true;
            }
        }

        commandBuilder << std::endl;

        std::string commandString = commandBuilder.str();

        //        std::cout << commandString << std::endl;
        auto res = cli.post("/workspace1?operation=updateGraph",
                commandString,
                "plain/text");

        std::cout << res->status << std::endl;
        std::cout << res->body << std::endl;
        std::cout << "Graph test end" << std::endl;
    }
#endif

    void computeShortestPath() {
        while (!U.isEmpty()) {
            Vertex& u = vertices[U.pop()->id];
            Key k_old = u.key;
            Key k_new = calculateKey(u.id);

            if (k_old < k_new) {
                u.key = k_new;
                U.push(&vertices[u.id]);
            } else if (u.g > u.rhs) {
                u.g = u.rhs;
                for (auto e : reverseEdges[u.id]) {
                    if (e.second->interior) {
                        updateEdgeEffort(e.second,
                                getInteriorEdgeEffort(e.second),
                                false);
                    } else {
                        updateEdgeEffort(e.second,
                                u.g + e.second->getEstimatedRequiredSamples(),
                                false);
                    }
                }

                auto neighbors = getNeighboringCells(u.id);
                for (auto n : neighbors) {
                    updateVertex(n);
                }
            } else {
                u.g = std::numeric_limits<double>::infinity();

                for (auto e : reverseEdges[u.id]) {
                    if (e.second->interior) {
                        updateEdgeEffort(e.second,
                                getInteriorEdgeEffort(e.second),
                                false);
                    } else {
                        updateEdgeEffort(e.second, u.g, false);
                    }
                }

                updateVertex(u.id);
                auto neighbors = abstraction->getNeighboringCells(u.id);
                for (auto n : neighbors) {
                    updateVertex(n);
                }
            }
        }
#ifdef STREAM_GRAPH
        publishAbstractGraph();
#endif
    }

    bool addedGoalEdge;
};
}
}
