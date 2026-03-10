#ifndef MUTICOST_PATHFIND_H
#define MUTICOST_PATHFIND_H

#include <cstdint>
#include <vector>
#include "multicost_graph.hpp"

class IMulticostPathfind {
public:
    virtual std::vector<uint32_t> getOptimalPath(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) = 0;
    virtual std::vector<uint32_t> getOptimalEdges(IMulticostGraph& graph, std::shared_ptr<IMulticostArray> multicostArray, uint32_t start, uint32_t end) = 0;
};

#endif