#include "../../include/grid_state.hpp"
#include <cstdint>
#include <iostream>
#include <vector>

int GridState::GRID_HEIGHT = 0;
int GridState::GRID_WIDTH = 0;
std::vector<bool> GridState::CELL_STATES = std::vector<bool>();

GridState::GridState() {
    this->x = 0;
    this->y = 0;
    this->linearPos = 0;
}


GridState::GridState(int x, int y) {
    this->linearPos = y * GridState::GRID_WIDTH + x;
    this->x = x;
    this->y = y;
}


std::vector<GridState> GridState::getNextStates() {
    std::vector<GridState> nextStates = std::vector<GridState>();
  
    if (GridState::CELL_STATES[this->y * GridState::GRID_WIDTH + this->x]) return nextStates;    
    
    bool isTop, isBot, isLft, isRgt;
    isTop = isBot = isLft = isRgt = false;

    if (this->y > 0 && !GridState::CELL_STATES[(this->y - 1) * GridState::GRID_WIDTH + this->x]) isTop = true;
    if (this->x > 0 && !GridState::CELL_STATES[this->y * GridState::GRID_WIDTH + (this->x - 1)]) isLft = true;
    if (this->y + 1 < GridState::GRID_HEIGHT && !GridState::CELL_STATES[(this->y + 1) * GridState::GRID_WIDTH + this->x]) isBot = true;
    if (this->x + 1 < GridState::GRID_WIDTH && !GridState::CELL_STATES[this->y * GridState::GRID_WIDTH + (this->x + 1)]) isRgt = true;


    if (isTop) nextStates.push_back(GridState(this->x, this->y - 1));
    if (isBot) nextStates.push_back(GridState(this->x, this->y + 1));
    if (isLft) nextStates.push_back(GridState(this->x - 1, this->y));
    if (isRgt) nextStates.push_back(GridState(this->x + 1, this->y));
    
    
    return nextStates;
}

uint32_t GridState::getUniqueId() {
    return this->linearPos;
}



int GridState::numberOfNearbyObstacles() {
    bool isTop, isBot, isLft, isRgt;
    isTop = isBot = isLft = isRgt = false;

    if (this->y > 0) isTop = true;
    if (this->y + 1 < GridState::GRID_HEIGHT) isBot = true;
    if (this->x > 0) isLft = true;
    if (this->x + 1 < GridState::GRID_WIDTH) isRgt = true;

    int result = 0;

    if (isTop) GridState::CELL_STATES[(y - 1) * GRID_WIDTH + x] ? result++ : result;
    if (isBot) GridState::CELL_STATES[(y + 1) * GRID_WIDTH + x] ? result++ : result;
    if (isLft) GridState::CELL_STATES[y * GRID_WIDTH + (x - 1)] ? result++ : result;
    if (isRgt) GridState::CELL_STATES[y * GRID_WIDTH + (x + 1)] ? result++ : result;
    
    return result;
}

