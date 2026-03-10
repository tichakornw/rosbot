#ifndef MULTICOST_GRAPH_H
#define MULTICOST_GRAPH_H

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include "multicost_array.hpp"
#include "multicost_compute.hpp"

struct MulticostEdge {
    uint32_t frNodeId;
    uint32_t toNodeId;
    unsigned int computedCostIndexBegin;
    unsigned int edgeCostId;
};


class IMulticostGraph {
public:
    virtual std::vector<MulticostEdge>& getNextEdges(uint32_t id, unsigned int computeIndex) = 0;
    virtual std::vector<MulticostEdge>& getPrevEdges(uint32_t id, unsigned int computeIndex) = 0;

    virtual void computeEdgesAtIndex(uint32_t id, unsigned int computeIndex) = 0;

    virtual const std::unique_ptr<MulticostID>& getEdgeCost(unsigned int edgeId) = 0;
};



template<typename S>
class LazyMulticostGraph : public IMulticostGraph {
public:
    LazyMulticostGraph (
        std::shared_ptr<IMulticostArray> multicostArray, 
        std::shared_ptr<IMulticostCompute<S>> compute
    ) : multicostArray(multicostArray), compute(compute) { } ;

    void computeEdgesAtIndex(uint32_t id, unsigned int computeIndex) override {
        S currentState = nodes[id];

        for (MulticostEdge nextEdge : mapNextEdges[id]) {
            unsigned computedCostIndexBegin = nextEdge.computedCostIndexBegin;
            if (!computedCost[computedCostIndexBegin + computeIndex]) {
                compute->computeCost(currentState, nodes[nextEdge.toNodeId], edgeCosts[nextEdge.edgeCostId], computeIndex);
                computedCost[computedCostIndexBegin + computeIndex] = true;
            }
        }
    }

    std::vector<MulticostEdge>& getNextEdges(uint32_t id, unsigned int computeIndex) override {

        S currentState = nodes[id];

        if (mapNextEdges.find(id) == mapNextEdges.end()) {
            addNextEdges(currentState, computeIndex);
        } else {
            computeEdgesAtIndex(id, computeIndex);
        }

        return mapNextEdges[id];
    };
    

    // This function is being held up by clear tapes logic
    std::vector<MulticostEdge>& getPrevEdges(uint32_t id, unsigned int computeIndex) override {
        // where this function can fail
        // this function assume that the backward edges computation on computeIndex are computed in the getNextEdges
        // id does not exist in backward edges

        return mapPrevEdges[id];
    };


    void addNode(S state) {
        nodes[state.getUniqueId()] = state;
    };


    const std::unordered_map<uint32_t, S>& getNodes() {
        return nodes;
    };

    bool isNodeExists(uint32_t nodeId) {
        return nodes.find(nodeId) != nodes.end();
    } 

    // Clear all multicosts
    void clear() {
        computedCost.clear();
        nodes.clear();
        mapNextEdges.clear();
        mapPrevEdges.clear();
    }

    const std::unique_ptr<MulticostID>& getEdgeCost(unsigned int edgeId) {
        return edgeCosts[edgeId];
    }

private:
    std::shared_ptr<IMulticostArray> multicostArray;
    std::shared_ptr<IMulticostCompute<S>> compute;
    
    std::vector<std::unique_ptr<MulticostID>> edgeCosts;

    std::vector<bool> computedCost;

    std::unordered_map<uint32_t, S> nodes;

    std::unordered_map<uint32_t, std::vector<MulticostEdge>> mapNextEdges;
    std::unordered_map<uint32_t, std::vector<MulticostEdge>> mapPrevEdges;


    void addNextEdges(S currentState, unsigned int computeIndex) {
        std::vector<S> nextStates = currentState.getNextStates();
        uint32_t frNodeId = currentState.getUniqueId();
        mapNextEdges[frNodeId] = std::vector<MulticostEdge>(nextStates.size());

        for (unsigned int i = 0; i < nextStates.size(); ++i) {
            S nextState = nextStates[i];
            uint32_t toNodeId = nextState.getUniqueId();

            nodes[toNodeId] = nextState;
            
            std::unique_ptr<MulticostID> costId = compute->computeCost(currentState, nextState, computeIndex);
            
            edgeCosts.push_back(std::move(costId));

            MulticostEdge edge;
            edge.edgeCostId = edgeCosts.size() - 1;
            edge.frNodeId = frNodeId;
            edge.toNodeId = toNodeId;
            edge.computedCostIndexBegin = computedCost.size();

            // Compute edge cost monoid at computeIndex
            for (unsigned k = 0; k < multicostArray->num_monoids(); ++k) computedCost.push_back(false);
            computedCost[edge.computedCostIndexBegin + computeIndex] = true;

            
            if (mapPrevEdges.find(toNodeId) == mapPrevEdges.end()) mapPrevEdges[toNodeId] = std::vector<MulticostEdge>();
            mapPrevEdges[toNodeId].push_back(edge);

            mapNextEdges[frNodeId][i] = edge;
        }
    };

};


class OptimalSubgraph {
public:
    OptimalSubgraph(IMulticostGraph& graph) : multicostGraph(graph) {
        isInitial = true;
    };

    const std::vector<MulticostEdge>& getOptimalEdges() {
        return optimalEdges;
    }
    
    const std::vector<MulticostEdge>& getOptimalNextEdges(uint32_t id) {
        return optimalNextEdges[id];
    };


    const std::vector<MulticostEdge>& getOptimalPrevEdges(uint32_t id) {
        return optimalPrevEdges[id];
    };


    const std::unique_ptr<MulticostID>& getEdgeCost(unsigned int edgeId) {
        return multicostGraph.getEdgeCost(edgeId);
    }


    std::vector<MulticostEdge>& getOptimalNextEdges(uint32_t id, unsigned int computeIndex) {
        if (isInitial) {
            return multicostGraph.getNextEdges(id, computeIndex);
        } else {
            // Make sure that the monoid at computeIndex is computed
            multicostGraph.computeEdgesAtIndex(id, computeIndex);
            return optimalNextEdges[id];
        }
    };


    std::vector<MulticostEdge>& getOptimalPrevEdges(uint32_t id, unsigned int computeIndex) {
        // where this function can fail
        // this function assume that the backward edges computation on computeIndex are computed in the getNextEdges
        // id does not exist in backward edges

        if (isInitial) {
            return multicostGraph.getPrevEdges(id, computeIndex);
        } else {
            // assume that the backward edges computation on computeIndex are computed in the getNextEdges
            // multicostGraph.computeEdgesAtIndex(id, computeIndex);
            return optimalPrevEdges[id];
        }
    };


    void addTempNextEdge(MulticostEdge edge) {
        uint32_t frNodeId = edge.frNodeId;
        if (tempNextEdges.find(frNodeId) == tempNextEdges.end()) tempNextEdges[frNodeId] = std::vector<MulticostEdge>();
        tempNextEdges[frNodeId].push_back(edge);
    };
    

    void addTempPrevEdge(MulticostEdge edge) {
        uint32_t toNodeId = edge.toNodeId;
        if (tempPrevEdges.find(toNodeId) == tempPrevEdges.end()) tempPrevEdges[toNodeId] = std::vector<MulticostEdge>();
        tempPrevEdges[toNodeId].push_back(edge);
    };


    void addOptimalEdge(MulticostEdge edge) {
        if (optimalNextEdges.find(edge.frNodeId) == optimalNextEdges.end()) optimalNextEdges[edge.frNodeId] = std::vector<MulticostEdge>();
        if (optimalPrevEdges.find(edge.toNodeId) == optimalPrevEdges.end()) optimalPrevEdges[edge.toNodeId] = std::vector<MulticostEdge>();

        optimalNextEdges[edge.frNodeId].push_back(edge);
        optimalPrevEdges[edge.toNodeId].push_back(edge);
        optimalEdges.push_back(edge);
    };

    void setNextWeight(uint32_t id, std::unique_ptr<MulticostID> cost) {
        nextWeights[id] = std::move(cost);
    };

    void setPrevWeight(uint32_t id, std::unique_ptr<MulticostID> cost) {
        prevWeights[id] = std::move(cost);
    };

    void clearOptimalEdges() {
        optimalNextEdges.clear();
        optimalPrevEdges.clear();
        optimalEdges.clear();
    };

    void clearPropagationEdges() {
        tempNextEdges.clear();
        tempPrevEdges.clear();
    };

    void clearWeights() {
        nextWeights.clear();
        prevWeights.clear();
    };

    bool isNextWeightInf(uint32_t frNodeId) {
        return nextWeights.find(frNodeId) == nextWeights.end();
    }

    const std::unique_ptr<MulticostID>& getNextWeight(uint32_t frNodeId) {
        return nextWeights[frNodeId];
    }

    bool isPrevWeightInf(uint32_t toNodeId) {
        return prevWeights.find(toNodeId) == prevWeights.end();
    }

    const std::unique_ptr<MulticostID>& getPrevWeight(uint32_t toNodeId) {
        return prevWeights[toNodeId];
    }

    bool isGraphExists() {
        return optimalNextEdges.size();
    };

    void notInitial() {
        isInitial = false;
    }

    std::unordered_map<uint32_t, std::vector<MulticostEdge>>& getTempNextEdges() {
        return tempNextEdges;
    }

    std::unordered_map<uint32_t, std::vector<MulticostEdge>>& getTempPrevEdges() {
        return tempPrevEdges;
    }

private:
    bool isInitial;
    IMulticostGraph& multicostGraph;

    std::vector<MulticostEdge> optimalEdges;

    std::unordered_map<uint32_t, std::vector<MulticostEdge>> optimalNextEdges;
    std::unordered_map<uint32_t, std::vector<MulticostEdge>> optimalPrevEdges;

    std::unordered_map<uint32_t, std::vector<MulticostEdge>> tempNextEdges;
    std::unordered_map<uint32_t, std::vector<MulticostEdge>> tempPrevEdges;

    std::unordered_map<uint32_t, std::unique_ptr<MulticostID>> nextWeights;
    std::unordered_map<uint32_t, std::unique_ptr<MulticostID>> prevWeights;

};


#endif