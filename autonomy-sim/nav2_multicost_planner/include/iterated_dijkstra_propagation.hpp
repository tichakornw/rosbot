#ifndef ITERATED_DIJKSTRA_PROPAGATION_H
#define ITERATED_DIJKSTRA_PROPAGATION_H

#include "multicost_graph.hpp"
#include "multicost_array.hpp"
#include "multicost_pathfind.hpp"


#include <cstdint>
#include <memory>


class IteratedDijkstraPropagation : public IMulticostPathfind {
public:
    std::vector<uint32_t> getOptimalPath(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) override;
    std::vector<uint32_t> getOptimalEdges(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) override;

private:
    OptimalSubgraph optimalSubgraph(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end);
   
    void forwardDijkstra(OptimalSubgraph& optimalGraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t source, unsigned int monoidIndex);
    void backwardDijkstra(OptimalSubgraph& optimalGraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t source, unsigned int monoidIndex);
    
    void bfsOptimalEdgeRetrieval(OptimalSubgraph& optimalSubgraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, unsigned int monoidIndex);
    
    void iterate(OptimalSubgraph& optimalSubgraph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end, unsigned int index);

};

#endif