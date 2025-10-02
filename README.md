# AI Treasure Hunt - A* Pathfinding Implementation
## Overview
An A* search algorithm implementation for navigating a virtual world filled with obstacles, traps, rewards, and treasures. The agent must collect all treasures while avoiding traps and managing dynamic constraints.
## Problem Description
- Navigate through a hexagonal grid-based virtual world
- Collect treasures while avoiding obstacles
- Handle dynamic traps that affect movement speed and energy consumption
- Utilize rewards that improve navigation efficiency
- Find optimal path considering all constraints
## Features
- **A* Search Algorithm**: Efficient pathfinding with heuristic guidance
- **State Management**: Tracks agent location, energy, collected treasures, and active effects
- **Dynamic Constraints**: Handles traps and rewards that modify game state
- **Hexagonal Grid Navigation**: Custom neighbor calculation for hex-based movement
## Files
- `Game.py` - Main game logic and A* algorithm implementation
- `State.py` - State representation and validation
- `Node.py` - Node class for grid cells
- `Main.py` - Entry point and execution
- `locations.txt` - Map configuration file
## Technologies Used
- Python 3
- Custom A* implementation with priority queue (heapq)
## How to Run
```bash
python Main.py
