// SOURCE: https://www.geeksforgeeks.org/c-program-for-depth-first-search-or-dfs-for-a-graph/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

// Structure for a node in the adjacency list
struct Node {
    int data;
    struct Node* next;
};

// Structure for the adjacency list
struct List {
    struct Node* head;
};

// Structure for the graph
struct Graph {
    int vertices;
    struct List* array;
};

// Function to create a new node
struct Node* createNode(int data) {
    struct Node* newNode = (struct Node*)malloc(sizeof(struct Node));
    newNode->data = data;
    newNode->next = NULL;
    return newNode;
}

// Function to create a graph with a given number of vertices
struct Graph* createGraph(int vertices) {
    struct Graph* graph = (struct Graph*)malloc(sizeof(struct Graph));
    graph->vertices = vertices;
    graph->array = (struct List*)malloc(vertices * sizeof(struct List));

    for (int i = 0; i < vertices; i++) {
        graph->array[i].head = NULL;
    }

    return graph;
}

// Function to add an edge to the graph
void addEdge(struct Graph* graph, int src, int dest) {
    struct Node* newNode = createNode(dest);
    newNode->next = graph->array[src].head;
    graph->array[src].head = newNode;

    printf("Added edge from %d to %d\n", src, dest);

    // Uncomment the following to make it undirected
    
    newNode = createNode(src);
    newNode->next = graph->array[dest].head;
    graph->array[dest].head = newNode;
    printf("Added edge from %d to %d (undirected)\n", dest, src);
    
}

// Original DFS
void DFS(struct Graph* graph, int vertex, bool visited[]) {
    visited[vertex] = true;
    printf("Visited %d\n", vertex);

    struct Node* currentNode = graph->array[vertex].head;
    while (currentNode) {
        int adjacentVertex = currentNode->data;
        if (!visited[adjacentVertex]) {
            printf("Going deeper from %d to %d\n", vertex, adjacentVertex);
            DFS(graph, adjacentVertex, visited);
        }
        currentNode = currentNode->next;
    }
}

// DFS Traversal in specified order
void DFSTraversal(struct Graph* graph, int* order, int orderSize) {
    bool* visited = (bool*)malloc(graph->vertices * sizeof(bool));
    for (int i = 0; i < graph->vertices; i++) {
        visited[i] = false;
    }

    printf("\nStarting DFS Traversal in order:\n");
    for (int i = 0; i < orderSize; i++) {
        printf("Checking vertex %d\n", order[i]);
        if (!visited[order[i]]) {
            DFS(graph, order[i], visited);
        }
    }

    free(visited);
}






// Modified isObstacle
bool isObstacle(int vertex, int* obstacle, int obstacleSize, bool shouldPrint) {
    for (int i = 0; i < obstacleSize; i++) {
        if (obstacle[i] == vertex) {
            if (shouldPrint) printf("Blocked by obstacle at %d!\n", vertex);
            return true;
        }
    }
    return false;
}

void DFSShortestPath(struct Graph* graph, int current, int destination, bool visited[], int path[], int pathIndex,
                     int* shortestPath, int* shortestLength, int* obstacle, int obstacleSize,
                     int steps, int* minSteps) {
    visited[current] = true;
    path[pathIndex++] = current;

    printf("\nCurrent path: ");
    for (int i = 0; i < pathIndex; i++) {
        printf("%d ", path[i]);
    }
    printf("\nSteps so far: %d\n", steps);

    // Backup obstacle positions
    int prevObstacles[obstacleSize];
    for (int i = 0; i < obstacleSize; i++) {
        prevObstacles[i] = obstacle[i];
    }

    //Checking if the goal was already reached
    if (current == destination) {
        if (steps < *minSteps) {
            *minSteps = steps;
            *shortestLength = pathIndex;
            for (int i = 0; i < pathIndex; i++) {
                shortestPath[i] = path[i];
            }
            printf("[PATH FOUND] New shortest path found with total steps %d and path length %d\n", *minSteps, *shortestLength - 1);
        }
    } else {
        bool loop = true;
        while (loop) {
            loop = false;
            int moveCase = rand() % 2; // 0 = robot only, 1 = obstacle only
            printf("Move case: %d (%s)\n", moveCase,
                moveCase == 0 ? "robot only" :"obstacle only");

            bool robotMoved = false;

            // Move obstacles if needed
            if (moveCase == 1 ) {
                loop = true; // obstacles moved, may try again
                int randomObstacleIndex = rand() % obstacleSize;
                int original = obstacle[randomObstacleIndex];
                struct Node* neighbor = graph->array[original].head;

                while (neighbor) {
                    int candidate = neighbor->data;
                    if (!isObstacle(candidate, obstacle, obstacleSize, false) && candidate != current) {
                        printf("[OBSTACLE] Obstacle %d moves to %d\n", original, candidate);
                        obstacle[randomObstacleIndex] = candidate;
                        break;
                    }
                    if (candidate == current) {
                        printf("Obstacle movement blocked by candidate!\n");
                    }
                    neighbor = neighbor->next;
                }

                // Count obstacle move as one step (even if it doesn't move)
                steps++;
                
            }

            // Move robot if allowed
            if (moveCase == 0 ) {
                struct Node* temp = graph->array[current].head;
                while (temp) {
                    int adj = temp->data;
                    if (!visited[adj] && !isObstacle(adj, obstacle, obstacleSize, true)) {
                        printf("[ROBOT] Robot exploring from %d to %d\n", current, adj);
                        // Robot move counts as a step
                        DFSShortestPath(graph, adj, destination, visited, path, pathIndex,
                                        shortestPath, shortestLength, obstacle, obstacleSize,
                                        steps + 1, minSteps);
                        robotMoved = true;
                    }
                    temp = temp->next;
                }
            }

            // If robot did not move, wait and try again
            if (!robotMoved && (moveCase == 0)) {
                printf("⏳ Robot is blocked at %d. Waiting for path to clear...\n", current);

                // Try to move obstacles again
                for (int i = 0; i < obstacleSize; i++) {
                    int original = obstacle[i];
                    struct Node* neighbor = graph->array[original].head;
                    while (neighbor) {
                        int candidate = neighbor->data;
                        if (!isObstacle(candidate, obstacle, obstacleSize, false) && candidate != current) {
                            printf("[OBSTACLE] Obstacle %d (during wait) moves to %d\n", original, candidate);
                            obstacle[i] = candidate;
                            break;
                        }
                        neighbor = neighbor->next;
                    }
                }

                // Count obstacle move as a step for waiting
                steps++;

                // Retry robot movement after obstacle movement
                struct Node* temp = graph->array[current].head;
                while (temp) {
                    int adj = temp->data;
                    if (!visited[adj] && !isObstacle(adj, obstacle, obstacleSize, true)) {
                        printf("[ROBOT] Robot retrying from %d to %d\n", current, adj);
                        DFSShortestPath(graph, adj, destination, visited, path, pathIndex,
                                        shortestPath, shortestLength, obstacle, obstacleSize,
                                        steps + 1, minSteps);
                    }
                    temp = temp->next;
                }
            }
        }
    }

    visited[current] = false;
    printf("[BACK] Backtracking robot from %d\n", current);
    // Backtrack obstacle positions
    for (int i = 0; i < obstacleSize; i++) {
        if (obstacle[i] != prevObstacles[i]) {
            printf("[BACK] Obstacle backtracked from %d to %d\n", obstacle[i], prevObstacles[i]);
            obstacle[i] = prevObstacles[i];
        }
    }
}






// Wrapper to find and print shortest path using DFS
void findShortestPathDFS(struct Graph* graph, int start, int end, int* obstacles, int obstacleSize) {
    bool* visited = (bool*)calloc(graph->vertices, sizeof(bool));
    int* path = (int*)malloc(graph->vertices * sizeof(int));
    int* shortestPath = (int*)malloc(graph->vertices * sizeof(int));
    int shortestLength = graph->vertices + 1;
    int minSteps = 1000000; // large initial value

    printf("\nFinding shortest path from %d to %d using DFS...\n", start, end);
    DFSShortestPath(graph, start, end, visited, path, 0, shortestPath, &shortestLength, obstacles, obstacleSize, 0, &minSteps);

    if (shortestLength <= graph->vertices) {
        printf("\nShortest path from %d to %d using DFS: ", start, end);
        for (int i = 0; i < shortestLength; i++) {
            printf("%d ", shortestPath[i]);
        }
        printf("\nPath length (robot moves): %d\n", shortestLength - 1);
        printf("Total steps (robot + obstacles): %d\n", minSteps);
    } else {
        printf("\nNo path found from %d to %d\n", start, end);
    }

    free(visited);
    free(path);
    free(shortestPath);
}


// Main function
// int main() {
//     int vertices = 4;
//     srand(time(NULL));

//     struct Graph* graph = createGraph(vertices);
    
//     // Adding edges
//     addEdge(graph, 2, 0);
//     addEdge(graph, 1, 2);
//     addEdge(graph, 0, 1);
//     addEdge(graph, 2, 3);
//     addEdge(graph, 1, 3);   

//     int obstacleSize = 2;
//     int* obstacles = (int*)malloc(obstacleSize * sizeof(int));
//     obstacles[0] = 1;
//     obstacles[1] = 0;
    

//     // DFS Traversal
//     // int order[] = {2, 0, 1, 3};
//     // int orderSize = sizeof(order) / sizeof(order[0]);

//     // printf("\n--- DFS TRAVERSAL OUTPUT ---\n");
//     // DFSTraversal(graph, order, orderSize);
    
//     // Shortest path
//     int start = 2;
//     int end = 3;
//     printf("\n--- SHORTEST PATH OUTPUT ---\n");
//     findShortestPathDFS(graph, start, end, obstacles, obstacleSize);

//     return 0;
// }
