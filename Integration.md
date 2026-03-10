# Integrating your own planning algorithms

## Overview

The system now implements a simpler, easy modifiable planner architecture which allows you easily switch between multiple algorithms without major changes in the original architecture. With the new setup you will be able to:

-   Switch algorithms via YAML configuration
-   Add new algorithms in a 'plug-and-play' manner.
-   Test multiple algorithms simply by changing a config parameter

## How It Works

### 1. Factory Pattern

The `MulticostPathfindFactory` maintains a registry of all available algorithms:

```cpp
auto& factory = MulticostPathfindFactory::getInstance();
auto algorithm = factory.create("iterated_dijkstra");

```

### 2. Automatic Registration

Each algorithm registers itself automatically when the library loads:

```cpp
// In your algorithm's .cpp file
REGISTER_PATHFIND_ALGORITHM("iterated_dijkstra", IteratedDijkstraPropagation);

```

### 3. Configuration-Based Selection

Simply specify the algorithm in your YAML file:

```yaml
GridBased:
  plugin: "nav2_multicost_planner/MulticostPlanner"
  algorithm_name: "iterated_dijkstra"  # Change this to switch algorithms!

```

## Directory Structure

```
nav2_multicost_planner/
├── include/
│   ├── nav2_multicost_planner/
│   │   └── nav2_multicost_planner.hpp          # Main planner (no algo-specific code!)
│   ├── your_planner/                           # Headers for the new planner
│   │   └── your_planner.hpp 
│   └── [other headers...]
└── src/
    ├── nav2_multicost_planner.cpp               # Main planner implementation
    └── your_planner/                            # User-defined planner
    │   └── your_planner.cpp                     
    │   └── [other folders...]

```

## Adding a New Algorithm - Step by Step

### Step 1: Create Header File

Create `include/your_planner/your_algorithm.hpp`:

```cpp
#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "multicost_pathfind.hpp"
#include "multicost_graph.hpp"
#include "multicost_array.hpp"
#include <memory>
#include <vector>
#include <cstdint>

class YourAlgorithm : public IMulticostPathfind {
public:
    YourAlgorithm() = default;
    ~YourAlgorithm() = default;

    // Required interface methods
    std::vector<uint32_t> getOptimalPath(
        IMulticostGraph& graph, 
        std::shared_ptr<IMulticostArray> multicostArray, 
        uint32_t start, 
        uint32_t end) override;

    std::vector<uint32_t> getOptimalEdges(
        IMulticostGraph& graph, 
        std::shared_ptr<IMulticostArray> multicostArray, 
        uint32_t start, 
        uint32_t end) override;

    // Your algorithm-specific params
    void setCustomParameter(double value) { custom_param_ = value; }

private:
    double custom_param_ = 1.0;
    
    // Your helper methods
    void yourHelperMethod();
};

#endif

```

This `include/your_planner` folder can also include additional header files that the other `.cpp` files in `src/your_planner` would depend on.

### Step 2: Create Implementation File

Create `src/your_planner/your_algorithm.cpp`:

```cpp
#include "your_algorithm.hpp"
#include "multicost_pathfind_factory.hpp"
#include "heap.hpp"

// IMPORTANT: Register your algorithm here!
REGISTER_PATHFIND_ALGORITHM("your_algorithm", YourAlgorithm); 

std::vector<uint32_t> YourAlgorithm::getOptimalPath(
    IMulticostGraph& graph, 
    std::shared_ptr<IMulticostArray> multicostArray, 
    uint32_t start, 
    uint32_t end) 
{
    std::vector<uint32_t> path;
    
    // Your algorithm implementation here
    // 1. Initialize data structures
    // 2. Search for path
    // 3. Reconstruct and return path
    
    return path;
}

std::vector<uint32_t> YourAlgorithm::getOptimalEdges(
    IMulticostGraph& graph, 
    std::shared_ptr<IMulticostArray> multicostArray, 
    uint32_t start, 
    uint32_t end) 
{
    std::vector<uint32_t> edges;
    
    // Your edge extraction implementation
    
    return edges;
}

void YourAlgorithm::yourHelperMethod() {
    // Helper methods go here
}

```

### Step 3: Add to CMakeLists.txt

Edit `CMakeLists.txt`, find the `multicost_search` library and add your file:

```cmake
add_library(multicost_search SHARED
  src/search/iterated_dijkstra_propagation.cpp
  src/state/grid_state.cpp
  src/your_planner/your_algorithm.cpp  # Add this line
)

```

### Step 4: Build

The `.dockerfile`  associated with the container running the system inherently builds the packages, you will simply have to run the following command to update the package changes:

#### For Simulation (Gazebo):
```bash
docker compose build -f compose.sim.gazebo.yaml build navigation
```

#### For Hardware:
```bash
docker compose build navigation
```

### Step 5: Configure

Edit your YAML config file (`nav2_pp_params.yaml`):

```yaml
GridBased:
  plugin: "nav2_multicost_planner/MulticostPlanner"
  algorithm_name: "your_algorithm"  # Use your registered name
  # ... other parameters
```

### Step 6: Run and Test

#### For Simulation (Gazebo):
```bash
just start-gazebo-sim
```

#### For Hardware:

**In the ROSbot terminal**:
```bash
just flash-firmware
```
To flash the Micro-ROS based firmware for STM32F4 microcontroller of the ROSbot 2/2R/2 PRO

```bash
just start-rosbot
```
To bring up the containers that run the sensors and Nav2 stack in the ROSbot.

**In the workstation terminal**:
```bash
just start-pc
```
To establish the connection between the ROSbot and the workstation as well as bring up the RViz2 window using which the user can get a clearer picture of the ROSbot environment.

(Further details are available in [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy), where a similar procedure is followed, excluding the Husarnet VPN Setup)

## Testing Your Algorithm


When the planner starts, it logs available algorithms. You should see the user-configured alias for your custom algorithm here:

```
[rosbot2r.planner_server]: Available pathfinding algorithms: iterated_dijkstra your_algorithm
[rosbot2r.planner_server]: Successfully loaded pathfinding algorithm: your_algorithm

```


