# SheepDog Simulation — HW2 Navigation & Pathfinding

## Project Description

A real-time 2D sheepdog simulation built with Pygame, extended from HW1 to include a full navigation system with global pathfinding, map representation, and path following.

**Scenario:** The sheep tries to eat all 4 bushes scattered across the map using A\* pathfinding. The sheepdog (controlled by the mouse) must herd the sheep into the pen before it finishes. Whoever completes their goal first wins.

- **Sheep wins** — eats all 4 bushes before being herded
- **Sheepdog wins** — drives the sheep into the pen

---

## System Requirements

- Python 3.9+
- Pygame 2.x

```bash
pip install pygame
```

---

## Running the Project

```bash
cd personalHW
python main.py
```

> **Note:** Make sure your keyboard is in English input mode before using hotkeys.

---

## Controls

| Key | Action |
|-----|--------|
| `3` | **Navigation mode** (HW2 main mode — sheep uses pathfinding) |
| `2` | Steering mode (HW1 compatible) |
| `1` | Kinematic mode (HW1 compatible) |
| `P` | Cycle pathfinding algorithm: A\* Euclidean → A\* Clearance → Dijkstra |
| `F` | Toggle path following strategy: Look-ahead ↔ Nearest-node |
| `G` | Toggle Grid overlay (red = blocked cell) |
| `C` | Toggle Clearance heatmap (blue intensity = distance to nearest wall) |
| `W` | Toggle Waypoint Graph overlay |
| `D` | Toggle debug info (path lines, state labels, panic radius) |
| `R` | Reset scene |

---

## Game Modes

### Navigation Mode (HW2 — Press `3`)

The sheep runs a state machine driven by pathfinding:

```
CHOOSE_BUSH → PATHFINDING → FOLLOWING → EATING → (next bush)
                                ↑            ↓
                             FLEEING ←── (dog too close)
```

- **CHOOSE\_BUSH** — randomly selects an uneaten bush
- **PATHFINDING** — runs A\* or Dijkstra to compute a path; if the goal cell is blocked, BFS finds the nearest open cell as a fallback
- **FOLLOWING** — steers along the path using Look-ahead or Nearest-node strategy
- **EATING** — stops at the bush for 2 seconds (progress bar shown); bush disappears when done
- **FLEEING** — briefly flees the dog for 1.2 seconds, then replans to a new bush
- **WIN** — all bushes eaten, sheep wins

Switch algorithm mid-game with `P`. Switch following strategy with `F`.

### Steering Mode (HW1 — Press `2`)

Sheep uses `SteeringBehaviors.arrive` to seek bushes and `flee` when the dog approaches. Obstacle avoidance is reactive (local forces only) — the sheep may get stuck against the central fence walls, which is expected behavior demonstrating the limitation of local steering vs. global planning.

Eating logic: sheep stops near a bush for 2 seconds, bush disappears, then moves to the next uneaten bush. All 4 bushes eaten = sheep wins.

### Kinematic Mode (HW1 — Press `1`)

Sheep uses `KinematicBehaviors.seek` to move toward bushes and `flee` from the dog. Same eating logic as Steering mode.

---

## Map Design

The map is 1024 × 768 px, divided into a 16 × 12 grid (64 px per cell).

```
Obstacles:
  Left stone cluster  — creates upper/lower route split
  Central fence wall  — 3 segments with 3 passable gaps (top / middle / bottom)
  Right sheep pen     — U-shaped fence with a left-facing opening

Bush positions (4 quadrants):
  A  (128, 160)  — top-left
  B  (864, 160)  — top-right
  C  (416, 416)  — center
  D  (352, 608)  — bottom-center
```

Bush positions are intentionally placed so that at least one obstacle must be bypassed to reach each one, making pathfinding non-trivial.

---

## Project Structure

```
personalHW/
├── main.py                      # Entry point, event loop, key bindings
├── simulation.py                # Core simulation: update, draw, win/loss logic
│
├── navigation/                  # HW2: Navigation system (new)
│   ├── __init__.py
│   ├── grid_map.py              # 64px grid, obstacle marking (AABB), clearance BFS
│   ├── dijkstra.py              # Dijkstra's algorithm
│   ├── astar.py                 # A* with Euclidean and Clearance heuristics
│   ├── path_follower.py         # Nearest-node and Look-ahead path following
│   └── waypoint_graph.py        # Manual waypoint graph (15 nodes, comparison use)
│
├── behaviors/                   # HW1: Steering behaviors (unchanged)
│   ├── avoidance.py
│   ├── blender.py
│   ├── kinematic.py
│   └── steering.py
│
├── entity/
│   ├── base_agent.py            # Unchanged
│   ├── bush.py                  # Unchanged
│   ├── obstacle.py              # Unchanged
│   ├── sheepdog.py              # Unchanged
│   └── sheep.py                 # HW2: full state machine + navigation integration
│
├── data/
│   └── metrics_logger.py
├── utils/
│   ├── debugger.py
│   └── vector_math.py
├── imgs/
└── README.md
```

---

## Navigation System Details

### Map Representation

Two representations are implemented for comparison:

**Grid (primary)**
- 16 × 12 cells, 64 px each → 192 nodes total
- Obstacle marking uses AABB overlap (handles thin fences narrower than one cell)
- Clearance precomputed via multi-source BFS: each cell stores its distance to the nearest blocked cell
- 4-directional movement (up/down/left/right)

**Waypoint Graph (comparison)**
- 15 manually placed nodes at intersections, bush locations, and pen entrance
- Edges weighted by Euclidean distance
- Far fewer nodes → faster search, but coarser paths

### Pathfinding Algorithms

| Algorithm | Heuristic | Explored nodes (typical) | Notes |
|-----------|-----------|--------------------------|-------|
| Dijkstra | None | ~150 | Guaranteed optimal; broad search |
| A\* Euclidean | Euclidean distance | ~80 | Admissible; faster than Dijkstra |
| A\* Clearance | Euclidean + wall penalty | ~85 | Not admissible; prefers open corridors |

**Clearance heuristic formula:**
```
h(n) = euclidean_distance(n, goal) + weight / (clearance(n) + 1)
```
Cells close to walls receive a higher penalty, guiding the sheep to prefer open corridors over tight passages.

### Path Following Strategies

**Nearest-node** — advances to the next waypoint once within arrival radius (28 px). Simple but can cause slight overshooting at sharp corners.

**Look-ahead** — projects a target point 96 px ahead along the path segments. Smoother cornering; the sheep begins turning before reaching each node.

---

## Win / Loss Conditions

| Mode | Sheep wins | Sheepdog wins |
|------|-----------|---------------|
| Navigation | All 4 bushes eaten (state machine) | Sheep enters pen area (X: 770–930, Y: 260–440) |
| Steering | All 4 bushes eaten (timer-based) | Same |
| Kinematic | All 4 bushes eaten (timer-based) | Same |

Press `R` to reset at any time, including after the game ends.

---

## Known Behavior Notes

- **Steering/Kinematic vs. central fence** — the sheep may oscillate or get stuck against the vertical fence segments. This is intentional: local steering forces cannot plan around obstacles globally. It serves as a demonstration of the gap between reactive behaviors and pathfinding-based navigation.
- **Clearance heuristic paths** — slightly longer than Euclidean paths on average, but keep the sheep further from walls, producing more natural-looking movement.

---

## Development Environment

- Python 3.9
- Pygame 2.6.1
- Supports Windows / macOS / Linux

---

## Author

Ingrid Miao

---

## Changelog

- **v2.0** — HW2: Added full navigation system (Grid, Waypoint Graph, Dijkstra, A\*, path following), sheep state machine, updated map, win/loss conditions for all modes
- **v1.0** — HW1: Kinematic, Steering, and Behavior Blending simulation
