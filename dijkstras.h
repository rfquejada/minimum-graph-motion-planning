#ifndef DIJKSTRAS_H
#define DIJKSTRAS_H

typedef struct node graph; // Forward declaration of graph from main.c

// Structure to return path and its length
typedef struct {
    int *path;
    int length;
} path_result;

path_result dijkstra(graph **g, int v, int start, int goal, int *obstacles, int num_obstacles);

#endif