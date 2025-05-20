#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <time.h>

// Structure for graph nodes
typedef struct node {
    int x;
    struct node *next;
} graph;

// Structure for priority queue nodes
typedef struct {
    int vertex;
    int dist;
    int *obstacle_config; // Current obstacle positions
} pq_node;

// Structure for priority queue
typedef struct {
    pq_node *nodes;
    int size;
    int capacity;
} priority_queue;

// Structure to return path and its length
typedef struct {
    int *path;
    int length;
} path_result;

// Function prototypes
path_result dijkstra(graph **g, int v, int start, int goal, int *obstacles, int num_obstacles);
priority_queue *create_pq(int capacity);
void pq_push(priority_queue *pq, int vertex, int dist, int *obstacle_config, int v);
pq_node pq_pop(priority_queue *pq);
int is_empty_pq(priority_queue *pq);
void free_pq(priority_queue *pq);
int *copy_obstacle_config(int *config, int v);
int config_equal(int *config1, int *config2, int v);
int is_valid_move(graph **g, int vertex, int *obstacle_config, int v);
int get_random_valid_neighbor(graph **g, int obstacle_vertex, int *obstacle_config, int curr_vertex, int v);

// Modified Dijkstra's algorithm
path_result dijkstra(graph **g, int v, int start, int goal, int *obstacles, int num_obstacles) {
    path_result result = {NULL, 0};

    // Seed random number generator
    srand(time(NULL));

    // Initialize distances and visited arrays
    int ***dist = malloc(v * sizeof(int **));
    int ***visited = malloc(v * sizeof(int **));
    for (int i = 0; i < v; i++) {
        dist[i] = malloc(v * sizeof(int *));
        visited[i] = malloc(v * sizeof(int *));
        for (int j = 0; j < v; j++) {
            dist[i][j] = malloc((1 << v) * sizeof(int));
            visited[i][j] = malloc((1 << v) * sizeof(int));
            for (int k = 0; k < (1 << v); k++) {
                dist[i][j][k] = INT_MAX;
                visited[i][j][k] = 0;
            }
        }
    }

    // Create priority queue
    priority_queue *pq = create_pq(v * v * (1 << v));
    
    // Initialize starting state
    int *initial_config = copy_obstacle_config(obstacles, v);
    dist[start][0][0] = 0;
    pq_push(pq, start, 0, initial_config, v);
    
    // Store parent information for path reconstruction
    int ****parent = malloc(v * sizeof(int ***));
    for (int i = 0; i < v; i++) {
        parent[i] = malloc(v * sizeof(int **));
        for (int j = 0; j < v; j++) {
            parent[i][j] = malloc((1 << v) * sizeof(int *));
            for (int k = 0; k < (1 << v); k++) {
                parent[i][j][k] = malloc(2 * sizeof(int));
                parent[i][j][k][0] = -1; // Vertex
                parent[i][j][k][1] = -1; // Obstacle config
            }
        }
    }

    while (!is_empty_pq(pq)) {
        pq_node current = pq_pop(pq);
        int curr_vertex = current.vertex;
        int curr_dist = current.dist;
        int *curr_config = current.obstacle_config;
        
        // Convert config to bit mask for indexing
        int config_mask = 0;
        for (int i = 0; i < v; i++) {
            if (curr_config[i]) config_mask |= (1 << i);
        }
        
        if (visited[curr_vertex][0][config_mask]) {
            free(curr_config);
            continue;
        }
        
        visited[curr_vertex][0][config_mask] = 1;
        
        // Check if goal reached
        if (curr_vertex == goal) {
            // Reconstruct path
            int path_len = curr_dist + 1;
            result.path = malloc(path_len * sizeof(int));
            result.length = path_len;
            result.path[path_len - 1] = goal;
            
            int v = goal, steps = curr_dist - 1;
            int curr_mask = config_mask;
            while (steps >= 0) {
                result.path[steps] = parent[v][0][curr_mask][0];
                curr_mask = parent[v][0][curr_mask][1];
                v = result.path[steps];
                steps--;
            }
            
            free(curr_config);
            break;
        }
        
        // Randomly choose robot or obstacle move (50% probability)
        int move_type = rand() % 2; // 0 for robot, 1 for obstacle
        
        if (move_type == 0) { // Robot move
            graph *neighbor = g[curr_vertex];
            while (neighbor != NULL) {
                int next_vertex = neighbor->x;
                if (!curr_config[next_vertex]) { // Check if next vertex is free
                    int new_dist = curr_dist + 1;
                    int next_mask = config_mask;
                    
                    if (new_dist < dist[next_vertex][0][next_mask]) {
                        dist[next_vertex][0][next_mask] = new_dist;
                        int *new_config = copy_obstacle_config(curr_config, v);
                        pq_push(pq, next_vertex, new_dist, new_config, v);
                        
                        parent[next_vertex][0][next_mask][0] = curr_vertex;
                        parent[next_vertex][0][next_mask][1] = config_mask;
                    }
                }
                neighbor = neighbor->next;
            }
        } else { // Obstacle move
            // Select one obstacle randomly
            int *obstacle_vertices = malloc(num_obstacles * sizeof(int));
            int obstacle_count = 0;
            for (int i = 0; i < v; i++) {
                if (curr_config[i]) {
                    obstacle_vertices[obstacle_count++] = i;
                }
            }
            
            if (obstacle_count > 0) {
                // Randomly select one obstacle
                int selected_obstacle_idx = rand() % obstacle_count;
                int selected_obstacle = obstacle_vertices[selected_obstacle_idx];
                
                // Get a random valid neighbor for the selected obstacle
                int next_pos = get_random_valid_neighbor(g, selected_obstacle, curr_config, curr_vertex, v);
                
                if (next_pos != -1) { // If a valid move exists
                    int new_dist = curr_dist + 1;
                    int *new_config = copy_obstacle_config(curr_config, v);
                    new_config[selected_obstacle] = 0;
                    new_config[next_pos] = 1;
                    
                    int new_mask = 0;
                    for (int j = 0; j < v; j++) {
                        if (new_config[j]) new_mask |= (1 << j);
                    }
                    
                    if (new_dist < dist[curr_vertex][0][new_mask]) {
                        dist[curr_vertex][0][new_mask] = new_dist;
                        pq_push(pq, curr_vertex, new_dist, new_config, v);
                        
                        parent[curr_vertex][0][new_mask][0] = curr_vertex;
                        parent[curr_vertex][0][new_mask][1] = config_mask;
                    } else {
                        free(new_config);
                    }
                }
            }
            
            free(obstacle_vertices);
        }
        
        free(curr_config);
    }
    
    // Cleanup
    free_pq(pq);
    for (int i = 0; i < v; i++) {
        for (int j = 0; j < v; j++) {
            for (int k = 0; k < (1 << v); k++) {
                free(parent[i][j][k]);
            }
            free(parent[i][j]);
            free(dist[i][j]);
            free(visited[i][j]);
        }
        free(parent[i]);
        free(dist[i]);
        free(visited[i]);
    }
    free(parent);
    free(dist);
    free(visited);
    
    return result;
}

// Priority queue implementation
priority_queue *create_pq(int capacity) {
    priority_queue *pq = malloc(sizeof(priority_queue));
    pq->nodes = malloc(capacity * sizeof(pq_node));
    pq->size = 0;
    pq->capacity = capacity;
    return pq;
}

void pq_push(priority_queue *pq, int vertex, int dist, int *obstacle_config, int v) {
    pq_node node = {vertex, dist, obstacle_config};
    
    int i = pq->size;
    pq->nodes[i] = node;
    pq->size++;
    
    // Heapify up
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->nodes[parent].dist > pq->nodes[i].dist) {
            pq_node temp = pq->nodes[parent];
            pq->nodes[parent] = pq->nodes[i];
            pq->nodes[i] = temp;
            i = parent;
        } else {
            break;
        }
    }
}

pq_node pq_pop(priority_queue *pq) {
    pq_node result = pq->nodes[0];
    
    pq->nodes[0] = pq->nodes[pq->size - 1];
    pq->size--;
    
    // Heapify down
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        
        if (left < pq->size && pq->nodes[left].dist < pq->nodes[smallest].dist) {
            smallest = left;
        }
        if (right < pq->size && pq->nodes[right].dist < pq->nodes[smallest].dist) {
            smallest = right;
        }
        
        if (smallest != i) {
            pq_node temp = pq->nodes[i];
            pq->nodes[i] = pq->nodes[smallest];
            pq->nodes[smallest] = temp;
            i = smallest;
        } else {
            break;
        }
    }
    
    return result;
}

int is_empty_pq(priority_queue *pq) {
    return pq->size == 0;
}

void free_pq(priority_queue *pq) {
    for (int i = 0; i < pq->size; i++) {
        free(pq->nodes[i].obstacle_config);
    }
    free(pq->nodes);
    free(pq);
}

int *copy_obstacle_config(int *config, int v) {
    int *new_config = malloc(v * sizeof(int));
    memcpy(new_config, config, v * sizeof(int));
    return new_config;
}

int config_equal(int *config1, int *config2, int v) {
    for (int i = 0; i < v; i++) {
        if (config1[i] != config2[i]) return 0;
    }
    return 1;
}

int is_valid_move(graph **g, int vertex, int *obstacle_config, int v) {
    return vertex >= 0 && vertex < v && !obstacle_config[vertex];
}

// New helper function to get a random valid neighbor
int get_random_valid_neighbor(graph **g, int obstacle_vertex, int *obstacle_config, int curr_vertex, int v) {
    // Count valid neighbors
    graph *neighbor = g[obstacle_vertex];
    int valid_neighbors = 0;
    while (neighbor != NULL) {
        if (!obstacle_config[neighbor->x] && neighbor->x != curr_vertex) {
            valid_neighbors++;
        }
        neighbor = neighbor->next;
    }
    
    if (valid_neighbors == 0) return -1; // No valid neighbors
    
    // Select a random valid neighbor
    int target_index = rand() % valid_neighbors;
    neighbor = g[obstacle_vertex];
    int count = 0;
    while (neighbor != NULL) {
        if (!obstacle_config[neighbor->x] && neighbor->x != curr_vertex) {
            if (count == target_index) {
                return neighbor->x;
            }
            count++;
        }
        neighbor = neighbor->next;
    }
    
    return -1; // Should not reach here if valid_neighbors > 0
}