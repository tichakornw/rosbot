#ifndef BFS_PLANNER_H
#define BFS_PLANNER_H

#include "multicost_pathfind.hpp"
#include "multicost_graph.hpp"
#include "multicost_array.hpp"
#include <memory>
#include <vector>
#include <cstdint>

/**
 * Simple Breadth-First Search planner.
 * Explores nodes level by level until goal is found.
 * Guaranteed to find shortest path (in terms of number of steps).
 * Very simple and easy to verify!
 */
class BFSPlanner : public IMulticostPathfind {
public:
    BFSPlanner() = default;
    ~BFSPlanner() = default;

    /**
     * Find path using BFS.
     * @param graph Navigation graph
     * @param multicostArray Cost management system (not really used in BFS)
     * @param start Start node ID
     * @param end Goal node ID
     * @return Vector of node IDs from start to end
     */
    std::vector<uint32_t> getOptimalPath(
        IMulticostGraph& graph, 
        std::shared_ptr<IMulticostArray> multicostArray, 
        uint32_t start, 
        uint32_t end) override;

    /**
     * Get edges in the path.
     * @param graph Navigation graph
     * @param multicostArray Cost management system
     * @param start Start node ID
     * @param end Goal node ID
     * @return Vector of node IDs representing edges
     */
    std::vector<uint32_t> getOptimalEdges(
        IMulticostGraph& graph, 
        std::shared_ptr<IMulticostArray> multicostArray, 
        uint32_t start, 
        uint32_t end) override;
};

#endif