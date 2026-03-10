#ifndef EXAMPLE_SETUP_H
#define EXAMPLE_SETUP_H

#include "grid_state.hpp"
#include "iterated_dijkstra_propagation.hpp"
#include "single_optimal_path_finder.hpp"
#include <vector>

class ExampleSetup {
public:
    ExampleSetup(int gridWidth, int gridHeight);

    std::vector<GridState> getOptimalPath(GridState start, GridState end);
    std::vector<GridState> getOptimalEdges(GridState start, GridState end);

    // there is obstacle at x, y
    void setObstacle(int x, int y);

    // there is no obstacle at x, y
    void noObstacle(int x, int y);

    // If there any update in the graph, clear everything
    void resetGraph();

private:
    SingleOptimalPathFinder<GridState> singleOptimalPathFinder;
    IteratedDijkstraPropagation idpAlgorithm;

    static int compareDistanceCost(int c1, int c2);
    static int compareObstacleCost(int c1, int c2);
    static int addDistanceCost(int c1, int c2);
    static int addObstacleCost(int c1, int c2);
    static int computeDistanceCost(GridState& fromState, GridState& toState);
    static int computeObstacleCost(GridState& fromState, GridState& toState);

};

#endif