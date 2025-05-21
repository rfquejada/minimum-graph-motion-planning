# Minimum Graph Motion Planning in C (Dijkstra + DFS)

This project simulates **Minimum Graph Motion Planning** using **Dijkstra’s Algorithm** and **Depth-First Search (DFS)** in C. It reads a graph from a file, includes configurable obstacles (unreachable nodes), and finds a path from a start vertex to a goal vertex while avoiding obstacles.

---

## Files

- `main.c` — Main driver program for graph creation, pathfinding, and output.
- `dijkstras.h` / `dijkstras.c` — Dijkstra’s shortest path algorithm implementation.
- `dfs_pathfinder.h` / `dfs_pathfinder.c` — DFS-based pathfinding with obstacle handling.
- `config.in` — Input configuration for graph, obstacles, start, and goal.

---

## Features

- Create and display adjacency list for undirected graphs.
- Supports:
  - **Dijkstra’s Algorithm** (shortest path based on steps)
  - **DFS Pathfinding** (not guaranteed to be shortest)
- Obstacle-aware pathfinding (blocked nodes avoided).
- Configurable start and goal nodes via `config.in`.
- Displays path and runtime.

---

## `config.in` Format
<num_vertices>
<num_edges>
<source1> <dest1>
<source2> <dest2>
...
<num_obstacles>
<obstacle_vertex_1> <obstacle_vertex_2> ... <obstacle_vertex_x>
<start_vertex>
<goal_vertex>


### Example
7
8
0 1
0 2
1 2
1 3
2 4
3 4
4 5
5 6
2
1 3
0
5

## ⚙️ Compilation & Execution

```bash
gcc main.c dijkstras.c dfs_pathfinder.c
./a.out
```

To switch between algorithms, change the IS_DIJKSTRAS flag in main.c:
int IS_DIJKSTRAS = 1; // 1 for Dijkstra, 0 for DFS

## Authors
Deoduco, Janry S. (jsdeoduco@up.edu.ph)
Quejada, Roche F. (rfquejada@up.edu.ph)
2025 • For academic or educational use