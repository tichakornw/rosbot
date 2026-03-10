#ifndef GRID_STATE_H
#define GRID_STATE_H

#include <cstdint>
#include <vector>

class GridState {
public:
    static int GRID_WIDTH;
    static int GRID_HEIGHT;
    static std::vector<bool> CELL_STATES; // true -> obstacle, false -> no obstacle
    
    int x;
    int y;

    GridState();
    GridState(int x, int y);
    uint32_t getUniqueId();
    std::vector<GridState> getNextStates();

    int numberOfNearbyObstacles();

private:
    uint32_t linearPos;
    
};


#endif