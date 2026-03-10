#include "../../include/example_setup.hpp"
#include <vector>

ExampleSetup::ExampleSetup(int gridWidth, int gridHeight)  {

    GridState::GRID_HEIGHT = gridHeight;
    GridState::GRID_WIDTH = gridWidth;

    GridState::CELL_STATES = std::vector<bool>(gridWidth * gridHeight, false);

    constexpr unsigned int numMonoids = 2;

    std::array<int, numMonoids> identity = {0, 0};

    // All of the functions can also be replaced by lambdas

    // Comparator (Ordering)
    std::array<std::function<int(int a, int b)>, numMonoids> compares = {
        compareDistanceCost,
        compareObstacleCost,
    };

    // Binary operators
    std::array<std::function<int(int a, int b)>, numMonoids> binaryOperators = {
        addDistanceCost,
        addObstacleCost
    };

    // Computing the edge monoid cost based on the state
    std::array<std::function<int(GridState& a, GridState& b)>, numMonoids> computes = {
        computeDistanceCost,
        computeObstacleCost
    };    


    singleOptimalPathFinder = SingleOptimalPathFinder<GridState>(identity, compares, binaryOperators, computes);
    idpAlgorithm = IteratedDijkstraPropagation();
}


// compute and outputs a sequence of states of a single optimal path: StartState, State, State, ..., GoalState
std::vector<GridState> ExampleSetup::getOptimalPath(GridState start, GridState end)  {
    return singleOptimalPathFinder.getOptimalPath(idpAlgorithm, start, end);
}

// compute and outputs all optimal edges: State of edge 1, State of edge 1, State of edge2, State of edge2, ....
std::vector<GridState> ExampleSetup::getOptimalEdges(GridState start, GridState end)  {
    return singleOptimalPathFinder.getOptimalEdges(idpAlgorithm, start, end);
}

// Setup functions

// there is obstacle at x, y
void ExampleSetup::setObstacle(int x, int y) {
    GridState::CELL_STATES[y * GridState::GRID_WIDTH + x] = true;
}


// there is no obstacle at x, y
void ExampleSetup::noObstacle(int x, int y) {
    GridState::CELL_STATES[y * GridState::GRID_WIDTH + x] = false;
}


void ExampleSetup::resetGraph() {
    singleOptimalPathFinder.clearGraph();
}



// Multicost functions

// positive is larger, negative is smaller, 0 is equal.
int ExampleSetup::compareDistanceCost(int c1, int c2) {
    return c1 - c2;
}

int ExampleSetup::compareObstacleCost(int c1, int c2) {
    return c1 - c2;
}


int ExampleSetup::addDistanceCost(int c1, int c2) {
    return c1 + c2;
}


int ExampleSetup::addObstacleCost(int c1, int c2) {
    return c1 + c2;
}


// Can be more complex to account for actual positions of the cells
// However, I am assuming that the toCell is directly adjacent to fromCell grid
int ExampleSetup::computeDistanceCost(GridState& fromCell, GridState& toCell) {
    return 1;
}


int ExampleSetup::computeObstacleCost(GridState& fromState, GridState& toState) {
    return fromState.numberOfNearbyObstacles() + toState.numberOfNearbyObstacles();
}
