#include "../../include/multicost_array.hpp"
#include "../../include/multicost_graph.hpp"
#include "../../include/heap.hpp"

#include "../../include/iterated_dijkstra_propagation.hpp"

#include <cstdint>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>


std::vector<uint32_t> IteratedDijkstraPropagation::getOptimalPath(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) {
    OptimalSubgraph optimalSubgraph = this->optimalSubgraph(graph, multicostArray, start, end);

    if (!optimalSubgraph.isGraphExists()) return std::vector<uint32_t>();

    std::unordered_map<uint32_t, uint32_t> parent;
    std::vector<uint32_t> queueNodes;
    std::set<uint32_t> closed;
    queueNodes.push_back(end);
    closed.insert(end);

    while(queueNodes.size() > 0) {
        uint32_t currentNodeId = queueNodes.back();

        queueNodes.pop_back();
    
        const std::vector<MulticostEdge>& edges = optimalSubgraph.getOptimalPrevEdges(currentNodeId);

        bool startFound = false;
        for (const MulticostEdge edge : edges) {
            if (closed.find(edge.frNodeId) == closed.end()) {
                queueNodes.push_back(edge.frNodeId);
                closed.insert(edge.frNodeId);
            }

            parent[edge.frNodeId] = currentNodeId;
            
            if (edge.frNodeId == start) {
                startFound = true;
                break;
            }
        }
        if (startFound) break;
    }

    std::vector<uint32_t> optimalPath;

    uint32_t nextNodeId = parent[start];
    optimalPath.push_back(start);
    while (nextNodeId != end) {
        optimalPath.push_back(nextNodeId);
        nextNodeId = parent[nextNodeId];
    }
    optimalPath.push_back(end);

    return optimalPath;
}



std::vector<uint32_t> IteratedDijkstraPropagation::getOptimalEdges(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) {
    OptimalSubgraph optimalSubgraph = this->optimalSubgraph(graph, multicostArray, start, end);
   
    std::vector<uint32_t> optimalEdges;

    for (const MulticostEdge edge : optimalSubgraph.getOptimalEdges()) {
        optimalEdges.push_back(edge.frNodeId);
        optimalEdges.push_back(edge.toNodeId);
    }

    return optimalEdges;
}



void IteratedDijkstraPropagation::forwardDijkstra(OptimalSubgraph& optimalGraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t source, unsigned int monoidIndex) {
    auto heap = Heap<std::unique_ptr<MulticostID>> (
        [multicostArray, monoidIndex](const std::unique_ptr<MulticostID>& a, const std::unique_ptr<MulticostID>& b){ 
            return !(multicostArray->compare(a, b, monoidIndex) < 0); 
        }
    );
    
    std::set<uint32_t> closed;

    heap.push(std::move(multicostArray->identity()), source);

    while (heap.get_size() > 0) {


        std::unique_ptr<MulticostID> cost = heap.top_item();
        uint32_t id = heap.top_item_id();
        heap.pop();

        closed.insert(id);
        
        std::vector<MulticostEdge>& nextEdges = optimalGraph.getOptimalNextEdges(id, monoidIndex);
        
        for (int i = 0; i < nextEdges.size(); ++i) {
            const std::unique_ptr<MulticostID>& edgeCost = optimalGraph.getEdgeCost(nextEdges[i].edgeCostId);

            if (closed.find(nextEdges[i].toNodeId) == closed.end()) {
                std::unique_ptr<MulticostID> weight = multicostArray->op(cost, edgeCost, monoidIndex);
                
                bool success = heap.push(std::move(weight), nextEdges[i].toNodeId);
                
                if (success) {
                    optimalGraph.addTempNextEdge(nextEdges[i]);
                }
            } 
            else {
                // Going back does not incur additional costs
                if (multicostArray->is_identity(edgeCost, monoidIndex)) {
                    optimalGraph.addTempNextEdge(nextEdges[i]);
                } 
            }
        }
        
        optimalGraph.setNextWeight(id, std::move(cost));
    }
};



void IteratedDijkstraPropagation::backwardDijkstra(OptimalSubgraph& optimalGraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t source, unsigned int monoidIndex) {
    auto heap = Heap<std::unique_ptr<MulticostID>> (
        [multicostArray, monoidIndex](const std::unique_ptr<MulticostID>& a, const std::unique_ptr<MulticostID>& b){ 
            return !(multicostArray->compare(a, b, monoidIndex) < 0); 
        }
    );

    std::set<uint32_t> closed;

    heap.push(std::move(multicostArray->identity()), source);

    while (heap.get_size() > 0) {


        std::unique_ptr<MulticostID> cost = heap.top_item();
        uint32_t id = heap.top_item_id();
        heap.pop();

        closed.insert(id);
        
        std::vector<MulticostEdge>& prevEdges = optimalGraph.getOptimalPrevEdges(id, monoidIndex);
        
        for (int i = 0; i < prevEdges.size(); ++i) {
            const std::unique_ptr<MulticostID>& edgeCost = optimalGraph.getEdgeCost(prevEdges[i].edgeCostId);

            if (closed.find(prevEdges[i].frNodeId) == closed.end()) {
    
                std::unique_ptr<MulticostID> weight = multicostArray->op(cost, edgeCost, monoidIndex);

                bool success = heap.push(std::move(weight), prevEdges[i].frNodeId);
                
                if (success) {
                    optimalGraph.addTempPrevEdge(prevEdges[i]);
                }
            } 
            else {
                // Going back does not incur additional costs
                if (multicostArray->is_identity(edgeCost, monoidIndex)) {
                    optimalGraph.addTempPrevEdge(prevEdges[i]);
                } 
            }
        }
        
        optimalGraph.setPrevWeight(id, std::move(cost));
    }
};


void IteratedDijkstraPropagation::bfsOptimalEdgeRetrieval(OptimalSubgraph& optimalSubgraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, unsigned int monoidIndex) {
    const std::unique_ptr<MulticostID>& optimalCost = optimalSubgraph.getPrevWeight(start);
   
    std::unordered_map<uint32_t, std::vector<MulticostEdge>>& tempNextEdges = optimalSubgraph.getTempNextEdges();
    std::unordered_map<uint32_t, std::vector<MulticostEdge>>& tempPrevEdges = optimalSubgraph.getTempPrevEdges();


    std::queue<uint32_t> queueNodes;
    std::set<uint32_t> closed;
    queueNodes.push(start);
    closed.insert(start);

    std::unique_ptr<MulticostID> totalCost = multicostArray->identity();

    while(queueNodes.size() > 0) {
        uint32_t nodeId = queueNodes.front();
        queueNodes.pop();

        const std::unique_ptr<MulticostID>& nextWeight = optimalSubgraph.getNextWeight(nodeId);

        std::vector<MulticostEdge>& nextEdges = tempNextEdges[nodeId];

        for (const MulticostEdge edge : nextEdges) {
            if (optimalSubgraph.isPrevWeightInf(edge.toNodeId)) {
                continue;
            }

            const std::unique_ptr<MulticostID>& prevWeight = optimalSubgraph.getPrevWeight(edge.toNodeId);
            const std::unique_ptr<MulticostID>& edgeCost = optimalSubgraph.getEdgeCost(edge.edgeCostId);

            // TESTING TESTING TESTING
            // auto mArray = std::dynamic_pointer_cast<MonoMulticostArray<int, 2>>(multicostArray);
            // auto nextCostTest = mArray->get_values(nextWeight);
            // auto prevCostTest = mArray->get_values(prevWeight);
            // auto edgeCostTest = mArray->get_values(edgeCost);
            // auto optmCostTest = mArray->get_values(optimalCost);
            // std::printf("%s {%d, %d}\n", "n", nextCostTest[0], nextCostTest[1]);
            // std::printf("%s {%d, %d}\n", "p", prevCostTest[0], prevCostTest[1]);
            // std::printf("%s {%d, %d}\n", "e", edgeCostTest[0], edgeCostTest[1]);
            // std::printf("%s {%d, %d}\n", "o", optmCostTest[0], optmCostTest[1]);


            multicostArray->op(prevWeight, nextWeight, totalCost, monoidIndex);
            multicostArray->op(edgeCost, totalCost, totalCost, monoidIndex);


            if (multicostArray->compare(totalCost, optimalCost, monoidIndex) == 0) {
                optimalSubgraph.addOptimalEdge(edge);

                if (closed.find(edge.toNodeId) == closed.end()) {
                    queueNodes.push(edge.toNodeId);
                    closed.insert(edge.toNodeId);
                }
            }
        }

    }

}



void IteratedDijkstraPropagation::iterate(OptimalSubgraph& optimalSubgraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end, unsigned int index) {

    optimalSubgraph.clearPropagationEdges();
    optimalSubgraph.clearWeights();

    forwardDijkstra(optimalSubgraph, multicostArray, start, index);
    if (optimalSubgraph.isNextWeightInf(end)) return;

    backwardDijkstra(optimalSubgraph, multicostArray, end, index);    
    if (optimalSubgraph.isPrevWeightInf(start)) return;

    optimalSubgraph.clearOptimalEdges();

    bfsOptimalEdgeRetrieval(optimalSubgraph, multicostArray, start, index);

    optimalSubgraph.notInitial();
}


OptimalSubgraph
IteratedDijkstraPropagation::optimalSubgraph(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) {


    unsigned int numMonoids = multicostArray->num_monoids();
     
    OptimalSubgraph optimalSubgraph(graph);
    
    iterate(optimalSubgraph, multicostArray, start, end, 0);

    if (!optimalSubgraph.isGraphExists()) return optimalSubgraph;

    for (unsigned int i = 1; i < numMonoids; ++i) {
        iterate(optimalSubgraph, multicostArray, start, end, i);
        if (!optimalSubgraph.isGraphExists()) return optimalSubgraph;
    }

    return optimalSubgraph;
}

