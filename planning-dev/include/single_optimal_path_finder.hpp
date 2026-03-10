#ifndef SINGLE_OPTIMAL_PATH_FINDER_H
#define SINGLE_OPTIMAL_PATH_FINDER_H

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>
#include "multicost.hpp"
#include "multicost_array.hpp"
#include "multicost_compute.hpp"
#include "multicost_graph.hpp"
#include "multicost_pathfind.hpp"

template<typename S>
class SingleOptimalPathFinder {

public:
    SingleOptimalPathFinder() {};

    template<typename... Ts>
    SingleOptimalPathFinder(std::tuple<Ts...> identity, 
        std::tuple<std::function<int(Ts a, Ts b)>...> compares,
        std::tuple<std::function<Ts(Ts a, Ts b)>...> ops,
        std::tuple<std::function<Ts(S a, S b)>...> computes
    ) {
        PolyMulticostProps<Ts...> props(identity, compares, ops);
        std::shared_ptr<PolyMulticostArray<Ts...>> polyArray = std::make_shared<PolyMulticostArray<Ts...>>(props);
        std::shared_ptr<IMulticostCompute<S>> compute = std::make_shared<PolyMulticostCompute<S, Ts...>>(polyArray, computes);

        multicostArray = polyArray;

        graph = std::make_unique<LazyMulticostGraph<S>>(multicostArray, compute);
    };


    template<typename T, size_t SIZE>
    SingleOptimalPathFinder(std::array<T, SIZE> identity, 
        std::array<std::function<int(T a, T b)>, SIZE> compares,
        std::array<std::function<T(T a, T b)>, SIZE> ops,
        std::array<std::function<T(S& a, S& b)>, SIZE> computes
    ) {

        MonoMulticostProps<T, SIZE> props(identity, compares, ops);
        std::shared_ptr<MonoMulticostArray<T, SIZE>> monoArray = std::make_shared<MonoMulticostArray<T, SIZE>>(props);
        std::shared_ptr<IMulticostCompute<S>> compute = std::make_shared<MonoMulticostCompute<S, T, SIZE>>(monoArray, computes);

        multicostArray = monoArray;        
        
        graph = std::make_unique<LazyMulticostGraph<S>>(multicostArray, compute);

    };

    
    // Translating node ids into path of states
    std::vector<S> getOptimalPath(IMulticostPathfind& algorithm, S start, S end) {
        graph->addNode(start);
        
        std::vector<uint32_t> rawPath = algorithm.getOptimalPath(*graph, multicostArray, start.getUniqueId(), end.getUniqueId());
        std::vector<S> statesPath(rawPath.size());

        const std::unordered_map<uint32_t, S>& states = graph->getNodes();
        
        for (unsigned int i = 0; i < rawPath.size(); ++i) {
            statesPath[i] = states.at(rawPath[i]); 
        }
        
        return statesPath;
    };

    
    
    std::vector<S> getOptimalEdges(IMulticostPathfind& algorithm, S start, S end) {
        graph->addNode(start);
        
        std::vector<uint32_t> rawEdges = algorithm.getOptimalEdges(*graph, multicostArray, start.getUniqueId(), end.getUniqueId());
        std::vector<S> statesPath(rawEdges.size());

        const std::unordered_map<uint32_t, S>& states = graph->getNodes();
        
        for (unsigned int i = 0; i < rawEdges.size(); ++i) {
            statesPath[i] = states.at(rawEdges[i]); 
        }
        
        return statesPath;
    };

    // Clear all cached multicost computes
    void clearGraph() {
        graph->clear();
    }

private:
    std::unique_ptr<LazyMulticostGraph<S>> graph;
    std::shared_ptr<IMulticostArray> multicostArray;
    std::unique_ptr<IMulticostCompute<S>> multicostCompute;
};


#endif