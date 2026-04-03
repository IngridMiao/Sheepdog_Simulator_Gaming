# PersonalHW - SheepDog Simulation

## Project Description

This is a sheepdog simulation project based on Pygame, implementing AI behavior algorithms including Kinematic, Steering, and Behavior Blending. The project simulates a scenario where a sheepdog herds sheep into a pen, featuring obstacles, stones, fences, and bushes.

This project is a personal homework assignment (HW1), demonstrating basic implementations of AI agent behaviors.

## Features

- **Multiple Behavior Modes**:
  - KINEMATIC: Kinematic behaviors
  - STEERING: Steering behaviors
  - COMBINED: Behavior blending mode

- **Entity Objects**:
  - SheepDog
  - Sheep
  - Obstacles (stones and fences)
  - Bushes (sheep's targets)

- **Interactive Features**:
  - Keyboard controls to switch modes (1: KINEMATIC, 2: STEERING, 3: COMBINED)
  - Debug mode toggle (D key)
  - Visual debug information

## Installation

### System Requirements
- Python 3.x
- Pygame

### Installation Steps
1. Ensure Python 3.x is installed
2. Install Pygame:
   ```bash
   pip install pygame
   ```
3. Download or clone the project locally
4. Navigate to the project directory

## Usage

### Running the Project
```bash
python main.py
```

### Controls (Warning: you should be sure your keyboard is in English mode)
- **1 Key**: Switch to KINEMATIC mode
- **2 Key**: Switch to STEERING mode
- **3 Key**: Switch to COMBINED mode
- **D Key**: Toggle debug mode on/off
- **Close Window**: Exit the program

### Game Logic
- The sheepdog attempts to herd the sheep into the pen
- Sheep randomly move between bushes
- Obstacles affect the agents' movement paths

## Project Structure

```
personalHW/
├── main.py                 # Main program entry point
├── simulation.py           # Core simulation engine logic
├── behaviors/              # Behavior algorithm modules
│   ├── avoidance.py        # Avoidance behaviors
│   ├── blender.py          # Behavior blending
│   ├── kinematic.py        # Kinematic behaviors
│   └── steering.py         # Steering behaviors
├── data/                   # Data processing modules
│   └── metrics_logger.py   # Metrics logger
├── entity/                 # Entity object modules
│   ├── base_agent.py       # Base agent class
│   ├── bush.py             # Bush entity
│   ├── obstacle.py         # Obstacle entity
│   ├── sheep.py            # Sheep entity
│   └── sheepdog.py         # Sheepdog entity
├── imgs/                   # Image resources
├── utils/                  # Utility modules
│   ├── debugger.py         # Debug tools
│   └── vector_math.py      # Vector math utilities
└── README.md               # Project documentation
```

## Technical Implementation

- **Programming Language**: Python
- **Graphics Framework**: Pygame
- **AI Algorithms**: Kinematic, Steering behaviors, Behavior blending
- **Architecture**: Object-oriented design, modular structure

## Development Environment

- Python 3.9.0
- Pygame 2.6.1
- Supports Windows/Linux/macOS

## License

This project is for personal learning purposes and follows the MIT License.

## Author

[Ingrid Miao]

## Changelog

- v1.0: Initial version, implements basic sheepdog simulation features