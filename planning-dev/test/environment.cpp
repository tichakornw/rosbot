#include "raylib.h"
#include "raymath.h"

#include "../include/example_setup.hpp"
#include <cstdint>
#include <vector>

#define GRID_COLS 10
#define GRID_ROWS 10
#define CELL_SIZE 32

#define EMT 0
#define OBS 1
#define STR 2
#define END 3
#define FND 4



float CellToRealPos(int cellPos) {
    return cellPos * CELL_SIZE + (int)(CELL_SIZE / 2);
}


void Setup(ExampleSetup &exampleSetup, uint8_t grid[]) {
    //ExampleSetup exampleSetup(GRID_COLS, GRID_ROWS);
    for (int i = 0; i < GRID_COLS * GRID_ROWS; ++i) {
        if (grid[i] == OBS) {
            exampleSetup.setObstacle(i % GRID_COLS, i / GRID_COLS);
        } else {
            exampleSetup.noObstacle(i % GRID_COLS, i / GRID_COLS);
        }
    }
}


void DrawArrow(Vector2 start, Vector2 end, Color color)
{
    const float headSize = 10.0;
    const float thickness = 2.0;

    // Draw main line
    DrawLineEx(start, end, thickness, color);

    // Direction vector
    Vector2 dir = Vector2Normalize(Vector2Subtract(end, start));

    // Perpendicular vectors for arrow head
    Vector2 right = { -dir.y, dir.x };
    Vector2 left  = {  dir.y, -dir.x };

    // Arrow head endpoints
    Vector2 p1 = {
        end.x - dir.x * headSize + right.x * headSize * 0.5f,
        end.y - dir.y * headSize + right.y * headSize * 0.5f
    };

    Vector2 p2 = {
        end.x - dir.x * headSize + left.x * headSize * 0.5f,
        end.y - dir.y * headSize + left.y * headSize * 0.5f
    };

    DrawLineEx(end, p1, thickness, color);
    DrawLineEx(end, p2, thickness, color);
}



void Update(uint8_t &mode, ExampleSetup &setup, uint8_t grid[], Vector2 &startPos, Vector2 &endPos, std::vector<Vector2>& path, std::vector<Vector2>& edges) {
    if (mode != FND) return;

    if (startPos.x != -1 && endPos.x != -1) {
        path.clear();
        edges.clear();
        setup.resetGraph();

        Setup(setup, grid);

        std::vector<GridState> p = setup.getOptimalPath(GridState(startPos.x, startPos.y), GridState(endPos.x, endPos.y));

        for (GridState state : p) {
            Vector2 pos = {CellToRealPos(state.x), CellToRealPos(state.y)};
            path.push_back(pos);
        }

        std::vector<GridState> e = setup.getOptimalEdges(GridState(startPos.x, startPos.y), GridState(endPos.x, endPos.y));

        for (GridState state : e) {
            Vector2 pos = {CellToRealPos(state.x), CellToRealPos(state.y)};
            edges.push_back(pos);
        }
    }

    mode = OBS;
}



// Main loop input
void Input(uint8_t &currentMode, Vector2 &startPos, Vector2 &endPos, uint8_t grid[]) {

    if (IsKeyPressed(KEY_A)) currentMode = STR;
    if (IsKeyPressed(KEY_S)) currentMode = END;
    if (IsKeyPressed(KEY_D)) currentMode = OBS;

    if (IsKeyPressed(KEY_SPACE)) currentMode = FND;

    // Handle clicks
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        Vector2 mouse = GetMousePosition();

        // Convert mouse to grid coords
        int gx = mouse.x / CELL_SIZE;
        int gy = mouse.y / CELL_SIZE;
        
        if (gx >= 0 && gx < GRID_COLS && gy >= 0 && gy < GRID_ROWS) {
            switch(currentMode) {
                case STR:
                    if (startPos.x != -1) {
                        grid[(int)(startPos.y * GRID_COLS + startPos.x)] = EMT;
                    }

                    startPos.x = gx;
                    startPos.y = gy;
                    grid[(int)(startPos.y * GRID_COLS + startPos.x)] = STR;
                    break; 

                case END:
                    if (endPos.x != -1) {
                        grid[(int)(endPos.y * GRID_COLS + endPos.x)] = EMT;
                    }

                    endPos.x = gx;
                    endPos.y = gy;
                    grid[(int)(endPos.y * GRID_COLS + endPos.x)] = END;
                    break;

                case OBS:
                    if ((gx == endPos.x && gy == endPos.y)) break;
                    if ((gx == startPos.x && gy == startPos.y)) break;
                    
                    if (grid[(int)(gy * GRID_COLS + gx)] == OBS) {
                        grid[(int)(gy * GRID_COLS + gx)] = EMT;
                    } else if (grid[(int)(gy * GRID_COLS + gx)] != OBS) {
                        grid[(int)(gy * GRID_COLS + gx)] = OBS;
                    }

                    break;
                
                default:
                    break;
            }        
        }
    }
    
}




// Main loop draw
void Draw(uint8_t grid[], std::vector<Vector2>& path, std::vector<Vector2>& edges) {
    BeginDrawing();
    ClearBackground(RAYWHITE);


    // Draw grid
    for (int y = 0; y < GRID_ROWS; y++) {
        for (int x = 0; x < GRID_COLS; x++) {
            uint8_t state = grid[y * GRID_COLS + x];

            Rectangle cell = {(float)(x * CELL_SIZE), (float)(y * CELL_SIZE), CELL_SIZE, CELL_SIZE};

            Color cellColor;

            switch (state) {
                case STR: cellColor = GREEN; break;
                case END: cellColor = RED; break;
                case OBS: cellColor = BLACK; break;
                default: cellColor = LIGHTGRAY; break;
            }

            DrawRectangleRec(cell, Fade(cellColor, 0.6f));
            DrawRectangleLines(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE, GRAY);
        }
    }

    // Draw path
    for (int i = 0; i < path.size(); i++) {
        DrawCircle(path[i].x, path[i].y, 5.0, Color(BLUE));
    }

    // Draw edges
    for (int i = 0; i < edges.size(); i += 2) {
        DrawArrow(edges[i], edges[i+1], Color(BLACK));
    }

    EndDrawing();

}



// main
int main(void) {
    InitWindow(GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE, "Grid Environment");

    Vector2 startPos = {-1, -1};
    Vector2 endPos = {-1, -1};
    uint8_t grid[GRID_COLS * GRID_ROWS];
    for (int i = 0; i < GRID_COLS * GRID_ROWS; ++i) grid[i] = EMT;
    uint8_t mode = OBS;
    std::vector<Vector2> path;
    std::vector<Vector2> edges;
    ExampleSetup exampleSetup = ExampleSetup(GRID_COLS, GRID_ROWS);


    SetTargetFPS(30);

    while (!WindowShouldClose()) {
        Update(mode, exampleSetup, grid, startPos, endPos, path, edges);
        Input(mode, startPos, endPos, grid);
        Draw(grid,path, edges);
    }

    CloseWindow();
    return 0;
}
