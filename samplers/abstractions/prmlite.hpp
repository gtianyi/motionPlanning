#pragma once

#include <limits>
#include "abstraction.hpp"

#include <ompl/base/SpaceInformation.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>


class PRMLite : public Abstraction {
public:
	PRMLite(const ompl::base::SpaceInformation *si, const ompl::base::State *start, const ompl::base::State *goal, const FileMap &params) :
		Abstraction(start, goal), prmSize(params.integerVal("PRMSize")), numEdges(params.integerVal("NumEdges")),
		stateRadius(params.doubleVal("StateRadius")) {

		resizeFactor = params.exists("PRMResizeFactor") ? params.doubleVal("PRMResizeFactor") : 2;

		//Stolen from tools::SelfConfig::getDefaultNearestNeighbors
		if(si->getStateSpace()->isMetricSpace()) {
			// if (specs.multithreaded)
			//  nn.reset(new NearestNeighborsGNAT<Vertex*>());
			//else
			nn.reset(new ompl::NearestNeighborsGNATNoThreadSafety<Vertex *>());
		} else {
			nn.reset(new ompl::NearestNeighborsSqrtApprox<Vertex *>());
		}

		nn->setDistanceFunction(boost::bind(&Abstraction::abstractDistanceFunction, this, _1, _2));
	}
	
	virtual ~PRMLite() = default;

	virtual void initialize(bool forceConnectedness = true) {
		generateVertices();
		generateEdges();

		while(forceConnectedness && !checkConnectivity()) {
			grow();
		}
	}

	virtual void grow() {
		for(unsigned int i = 0; i < prmSize; ++i) {
			vertices[i]->populatedNeighors = false;
			vertices[i]->neighbors.clear();
		}

		unsigned int oldPRMSize = prmSize;
		prmSize *= resizeFactor;
		vertices.resize(prmSize);
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		ompl::base::ValidStateSamplerPtr abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();
		for(unsigned int i = oldPRMSize; i < prmSize; ++i) {
			vertices[i] = new Vertex(i);
			vertices[i]->state = abstractSpace->allocState();
			abstractSampler->sample(vertices[i]->state);
			nn->add(vertices[i]);
		}

		generateEdges();
	}

	virtual unsigned int getStartIndex() const {
		return 0;
	}

	virtual unsigned int getGoalIndex() const {
		return 1;
	}

	virtual bool supportsSampling() const {
		return false;
	}

	virtual ompl::base::State* sampleAbstractState(unsigned int index) {
		throw ompl::Exception("PRMLite::sampleAbstractState", "not supported");
	}

	virtual unsigned int mapToAbstractRegion(const ompl::base::ScopedState<> &s) const {
		Vertex v(0);
		auto ss = globalParameters.globalAppBaseControl->
			getGeometricComponentState(s, std::numeric_limits<unsigned int>::max());
		
		v.state = ss.get();
		return nn->nearest(&v)->id;
	}
    
    Vertex* splitRegion(const unsigned int region) {
        
        Vertex* newRegionCenter = allocateVertex();
	    const auto oldRegionCenter = vertices[region];

	    auto abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();
	    abstractSampler->sampleNear(newRegionCenter->state, oldRegionCenter->state, stateRadius);
        
	    nn->add(newRegionCenter);

	    // Add edges between the two new nodes
	    edges[newRegionCenter->id][oldRegionCenter->id] = Edge(oldRegionCenter->id);
	    edges[oldRegionCenter->id][newRegionCenter->id] = Edge(newRegionCenter->id);
	    
	    auto distanceFunction = nn->getDistanceFunction();

	    const auto oldEdges = edges[oldRegionCenter->id];
	    
	    for (auto idEdgePair : oldEdges) {
		    const unsigned int endVertexId = idEdgePair.first;
		    const Edge& edge = idEdgePair.second;
		    
            const double oldCenterDistance = distanceFunction(oldRegionCenter, vertices[edge.endpoint]);
		    const double newCenterDistance = distanceFunction(newRegionCenter, vertices[edge.endpoint]);

		    if (newCenterDistance < oldCenterDistance) {
			    // Remove old-target edge in both directions
			    edges[endVertexId].erase(oldRegionCenter->id);
			    edges[oldRegionCenter->id].erase(endVertexId);

			    // Add new-target edges in both directions
			    edges[newRegionCenter->id][endVertexId] = Edge(endVertexId);
			    edges[endVertexId][newRegionCenter->id] = Edge(newRegionCenter->id);
		    }
        }
        
        // Make sure that they have at least a few edges. 
	    if (edges[newRegionCenter->id].size() < numEdges) {
		    addKNeighbors(newRegionCenter, numEdges);
	    }
        
	    if (edges[oldRegionCenter->id].size() < numEdges) {
		    addKNeighbors(oldRegionCenter, numEdges);
	    }

	    // Reset neighbor lists
        oldRegionCenter->neighbors.clear();
	    oldRegionCenter->populatedNeighors = false;
	    newRegionCenter->neighbors.clear();
	    newRegionCenter->populatedNeighors = false;       
        
        return newRegionCenter;
    }
    

protected:
	Vertex* allocateVertex(const unsigned int vertexId = vertices.size()) {
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		
        vertices.push_back(new Vertex(vertexId));
		Vertex* vertex = vertices[vertexId];
       
		vertex->state = abstractSpace->allocState();
		
		return vertex;
	}
	
	virtual void generateVertices() {
		Timer timer("Vertex Generation");
		ompl::base::StateSpacePtr abstractSpace = globalParameters.globalAbstractAppBaseGeometric->getStateSpace();
		ompl::base::ValidStateSamplerPtr abstractSampler = globalParameters.globalAbstractAppBaseGeometric->getSpaceInformation()->allocValidStateSampler();

		vertices.resize(prmSize);

		vertices[0] = new Vertex(0);
		vertices[0]->state = abstractSpace->allocState();
		abstractSpace->copyState(vertices[0]->state, start);
		nn->add(vertices[0]);

		vertices[1] = new Vertex(1);
		vertices[1]->state = abstractSpace->allocState();
		abstractSpace->copyState(vertices[1]->state, goal);
		nn->add(vertices[1]);

		for(unsigned int i = 2; i < prmSize; ++i) {
			vertices[i] = new Vertex(i);
			vertices[i]->state = abstractSpace->allocState();
			abstractSampler->sample(vertices[i]->state);
			nn->add(vertices[i]);
		}
	}

	void generateEdges() {
		Timer timer("Edge Generation");
		edges.clear();

		auto distanceFunc = nn->getDistanceFunction();

		for (Vertex* vertex : vertices) {
			edges[vertex->id];

			addKNeighbors(vertex);
		}
	}
    
	void addKNeighbors(Vertex* vertex, const unsigned int edgeCount = numEdges + 1) const {
		std::vector<Vertex*> neighbors;
		nn->nearestK(vertex, edgeCount, neighbors);

		for (Vertex* neighbor : neighbors) {
			if (vertex->id == neighbor->id) continue;
			edges[vertex->id][neighbor->id] = Edge(neighbor->id);
			edges[neighbor->id][vertex->id] = Edge(vertex->id);
		}
	}

	boost::shared_ptr< ompl::NearestNeighbors<Vertex*> > nn;
	unsigned int prmSize, numEdges;
	double stateRadius, resizeFactor;
};