#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pyframe.h>

#include "../include/example_setup.hpp"

// pt.compute(grid, size, size, startx, starty, goalx, goaly)

void compute(std::vector<std::vector<int>> grid2d, int width, int height, int startx, int starty, int goalx, int goaly) {
    
    ExampleSetup exampleSetup = ExampleSetup(width, height);

    // Set up graphs again
    exampleSetup.resetGraph();
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            if (grid2d[i][j] == 1) {
                exampleSetup.setObstacle(j, i);
            } else {
                exampleSetup.noObstacle(j, i);
            }
        }
    }

    auto edges = exampleSetup.getOptimalEdges(GridState(startx, starty), GridState(goalx, goaly));
}

int main() {
    return 0;
}

PYBIND11_MODULE(multicost_pathfind_bindings, m) {
    m.def("compute", &compute);
}

