#ifndef DFS_PATHFINDER_H
#define DFS_PATHFINDER_H

struct Graph* createGraph(int vertices);
void addEdge(struct Graph* graph, int src, int dest);
void findShortestPathDFS(struct Graph* graph, int start, int end, int* obstacles, int obstacleSize);

#endif
