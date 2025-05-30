#include<stdio.h>
#include<stdlib.h>
#include "dijkstras.h"
#include "dfs_pathfinder.h"
#include <time.h>

int IS_DIJKSTRAS = 1;

//Structure for the graph
typedef struct node{
    int x;
    struct node *next;
}graph;

graph **createAdjList(int *, int *);
void viewList(graph **, int);
void deleteGraph(graph **, int);
int *readObstacles(int, int *, int *, int *);
int *readObstaclesDFS(int, int *, int *, int *);

void printObstacles(int *, int);
void printStartGoal(int, int);
struct timespec time_before, time_after;


int main() {
    graph **g;
    int v, e, num_obstacles, start, goal;
    float time_before, time_after, time_elapsed;
    srand(time(NULL)); 
  
    g = createAdjList(&v, &e);
    int *obstacles = readObstacles(v, &num_obstacles, &start, &goal);
    

    viewList(g, v);
    printObstacles(obstacles, v);
    printStartGoal(start, goal);

    time_before = clock();

    if (IS_DIJKSTRAS){
        path_result result = dijkstra(g, v, start, goal, obstacles, num_obstacles);
        if (result.path) {
            printf("Path found: ");
            for (int i = 0; i < result.length; i++) {
                printf("%d ", result.path[i]);
            }
            printf("\nNumber of steps: %d\n", result.length - 1);
            free(result.path);
        } else {
            printf("No path found\n");
        }
    } else {
        struct Graph* dfsGraph = createGraph(v);
        FILE *fp = fopen("config.in", "r");
        if (fp == NULL) {
            printf("Error opening config.in\n");
            exit(1);
        }

        fscanf(fp, "%d", &v);  // Number of vertices
        fscanf(fp, "%d", &e);  // Number of edges

        for (int i = 0; i < e; i++) {
            int src, dest;
            fscanf(fp, "%d %d", &src, &dest);
            addEdge(dfsGraph, src, dest);
        }

        fclose(fp);



        printf("\n--- DFS Pathfinding ---\n");
        obstacles = readObstaclesDFS(v, &num_obstacles, &start, &goal);
        findShortestPathDFS(dfsGraph, start, goal, obstacles, num_obstacles);
    }

  
    time_after = clock();
    time_elapsed = (float)(time_after - time_before) / CLOCKS_PER_SEC;
    printf("Time taken: %f seconds\n", time_elapsed);


    deleteGraph(g, v);
    free(obstacles);
    return 0;
}

//Reads the graph from a file and creates an adjacency list
graph **createAdjList(int *v, int *e){
    FILE *fp = fopen("config.in", "r");
    if (fp == NULL) {
        printf("Error opening config.in\n");
        exit(1);
    }

    fscanf(fp, "%d", v);
    fscanf(fp, "%d", e);
    
    graph** G = malloc(sizeof(graph*) * (*v));
    for(int i = 0; i < *v; i++){
        G[i] = NULL;
    }

    for (int i = 0; i < *e; i++){
        int dest, source;
        fscanf(fp, "%d %d", &dest, &source);

        graph* temp1 = malloc(sizeof(graph));
        temp1->x = source;
        temp1->next = G[dest];
        G[dest] = temp1;

        graph* temp2 = malloc(sizeof(graph));
        temp2->x = dest;
        temp2->next = G[source];
        G[source] = temp2;
    }

    fclose(fp);
    return G;
}

//Displays the adjacency list
void viewList(graph **g, int v){
    int i;
    graph *p;
    
    for(i = 0; i < v; i++){
        p = g[i];
        printf("%d: ", i);
        while(p != NULL){
            printf("%3d", p->x);
            p = p->next;
        }
        printf("\n");
    }
}

//Frees the memory allocated for the graph
void deleteGraph(graph **g, int v){
    int i;
    graph *p;
    for(i = 0; i < v; i++){
        while(g[i] != NULL){
            p = g[i];
            g[i] = g[i]->next;
            free(p);
        }
    }
    free(g);
}

int *readObstacles(int v, int *num_obstacles, int *start, int *goal) {
    FILE *fp = fopen("config.in", "r");
    if (fp == NULL) {
        printf("Error opening config.in\n");
        exit(1);
    }

    int vertices, edges;
    fscanf(fp, "%d", &vertices);
    fscanf(fp, "%d", &edges);

    // Skip edges
    for (int i = 0; i < edges; i++) {
        int a, b;
        fscanf(fp, "%d %d", &a, &b);
    }

    // Now read obstacles
    fscanf(fp, "%d", num_obstacles);
    int *obstacles = calloc(v, sizeof(int));
    for (int i = 0; i < *num_obstacles; i++) {
        int obs;
        fscanf(fp, "%d", &obs);
        obstacles[obs] = 1;
    }

    // Read start and goal
    fscanf(fp, "%d", start);
    fscanf(fp, "%d", goal);

    fclose(fp);
    return obstacles;
}

int *readObstaclesDFS(int v, int *num_obstacles, int *start, int *goal) {
    FILE *fp = fopen("config.in", "r");
    if (fp == NULL) {
        printf("Error opening config.in\n");
        exit(1);
    }

    int vertices, edges;
    fscanf(fp, "%d", &vertices);
    fscanf(fp, "%d", &edges);

    // Skip edges
    for (int i = 0; i < edges; i++) {
        int a, b;
        fscanf(fp, "%d %d", &a, &b);
    }

    // Now read obstacles
    fscanf(fp, "%d", num_obstacles);
    int *obstacles = malloc((*num_obstacles) * sizeof(int)); // Allocate space for the obstacle list
    for (int i = 0; i < *num_obstacles; i++) {
        fscanf(fp, "%d", &obstacles[i]); // Directly store the vertex index
    }

    // Read start and goal
    fscanf(fp, "%d", start);
    fscanf(fp, "%d", goal);

    fclose(fp);
    return obstacles;
}


// Print the list of obstacle vertices
void printObstacles(int *obstacles, int v) {
    printf("Obstacles at: ");
    int found = 0;
    for (int i = 0; i < v; i++) {
        if (obstacles[i]) {
            printf("%d ", i);
            found = 1;
        }
    }
    if (!found) {
        printf("None");
    }
    printf("\n");
}

// Print the start and goal vertices
void printStartGoal(int start, int goal) {
    printf("Start vertex: %d\n", start);
    printf("Goal vertex: %d\n", goal);
}