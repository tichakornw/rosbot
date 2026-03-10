#include "bfs_planner/bfs_planner.hpp"
#include "multicost_pathfind_factory.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>

// Register BFS algorithm with the factory
REGISTER_PATHFIND_ALGORITHM("bfs", BFSPlanner);

std::vector<uint32_t> BFSPlanner::getOptimalPath(
    IMulticostGraph& graph, 
    std::shared_ptr<IMulticostArray> multicostArray, 
    uint32_t start, 
    uint32_t end) 
{
    std::cout << "[BFS Planner] Starting search from " << start << " to " << end << std::endl;

    // Special case: start is goal
    if (start == end) {
        std::cout << "[BFS Planner] Start is goal!" << std::endl;
        return {start};
    }

    // Queue for BFS - contains node IDs to explore
    std::queue<uint32_t> to_explore;
    
    // Track visited nodes to avoid revisiting
    std::unordered_set<uint32_t> visited;
    
    // Track parent of each node for path reconstruction
    std::unordered_map<uint32_t, uint32_t> parent;

    // Start BFS from the start node
    to_explore.push(start);
    visited.insert(start);

    int nodes_explored = 0;

    // BFS main loop
    while (!to_explore.empty()) {
        // Get next node to explore
        uint32_t current = to_explore.front();
        to_explore.pop();
        
        nodes_explored++;

        // Check if we reached the goal
        if (current == end) {
            std::cout << "[BFS Planner] Goal found! Explored " << nodes_explored << " nodes" << std::endl;
            
            // Reconstruct path by following parent pointers backwards
            std::vector<uint32_t> path;
            uint32_t node = end;
            
            while (node != start) {
                path.push_back(node);
                node = parent[node];
            }
            path.push_back(start);
            
            // Reverse to get path from start to end
            std::reverse(path.begin(), path.end());
            
            std::cout << "[BFS Planner] Path length: " << path.size() << " nodes" << std::endl;
            return path;
        }

        // Get all neighbors of current node
        // Use index 0 (distance cost) to get neighbors
        std::vector<MulticostEdge>& neighbors = graph.getNextEdges(current, 0);

        // Explore each neighbor
        for (const MulticostEdge& edge : neighbors) {
            uint32_t neighbor = edge.toNodeId;
            
            // Skip if already visited
            if (visited.find(neighbor) != visited.end()) {
                continue;
            }

            // Mark as visited
            visited.insert(neighbor);
            
            // Set parent for path reconstruction
            parent[neighbor] = current;
            
            // Add to queue for exploration
            to_explore.push(neighbor);
        }
    }

    // No path found
    std::cout << "[BFS Planner] No path found after exploring " << nodes_explored << " nodes" << std::endl;
    return std::vector<uint32_t>();
}

std::vector<uint32_t> BFSPlanner::getOptimalEdges(
    IMulticostGraph& graph, 
    std::shared_ptr<IMulticostArray> multicostArray, 
    uint32_t start, 
    uint32_t end) 
{
    // Get the path first
    std::vector<uint32_t> path = getOptimalPath(graph, multicostArray, start, end);
    
    // Convert path to edges (pairs of consecutive nodes)
    std::vector<uint32_t> edges;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        edges.push_back(path[i]);
        edges.push_back(path[i + 1]);
    }
    
    return edges;
}