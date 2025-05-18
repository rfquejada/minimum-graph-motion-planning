#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

// Graph structure (same as previous code)
typedef struct node {
    int x;
    struct node *next;
} graph;

// Min-heap node for priority queue
typedef struct {
    int vertex;
    int f_score;
} heap_node;

// Min-heap structure
typedef struct {
    heap_node *nodes;
    int *positions; // Track position of each vertex in heap
    int size;
    int capacity;
} min_heap;

// Function prototypes
min_heap *create_min_heap(int capacity);
void heap_swap(min_heap *heap, int i, int j);
void heapify_up(min_heap *heap, int index);
void heapify_down(min_heap *heap, int index);
void heap_insert(min_heap *heap, int vertex, int f_score);
int heap_extract_min(min_heap *heap, int *vertex);
void heap_decrease_key(min_heap *heap, int vertex, int f_score);
int heap_is_empty(min_heap *heap);
void free_min_heap(min_heap *heap);
int *a_star(graph **G, int v, int *obstacles, int start, int goal, int *path_length);
int *reconstruct_path(int *parent, int start, int goal, int *path_length);
int heuristic(int current, int goal);

// Create a min-heap
min_heap *create_min_heap(int capacity) {
    min_heap *heap = malloc(sizeof(min_heap));
    heap->nodes = malloc(sizeof(heap_node) * capacity);
    heap->positions = malloc(sizeof(int) * capacity);
    heap->size = 0;
    heap->capacity = capacity;
    for (int i = 0; i < capacity; i++) {
        heap->positions[i] = -1; // -1 means not in heap
    }
    return heap;
}

// Swap two nodes in the heap
void heap_swap(min_heap *heap, int i, int j) {
    heap_node temp = heap->nodes[i];
    heap->nodes[i] = heap->nodes[j];
    heap->nodes[j] = temp;
    heap->positions[heap->nodes[i].vertex] = i;
    heap->positions[heap->nodes[j].vertex] = j;
}

// Heapify up after insertion
void heapify_up(min_heap *heap, int index) {
    while (index > 0) {
        int parent = (index - 1) / 2;
        if (heap->nodes[index].f_score < heap->nodes[parent].f_score) {
            heap_swap(heap, index, parent);
            index = parent;
        } else {
            break;
        }
    }
}

// Heapify down after extraction
void heapify_down(min_heap *heap, int index) {
    int min = index;
    int left = 2 * index + 1;
    int right = 2 * index + 2;

    if (left < heap->size && heap->nodes[left].f_score < heap->nodes[min].f_score) {
        min = left;
    }
    if (right < heap->size && heap->nodes[right].f_score < heap->nodes[min].f_score) {
        min = right;
    }

    if (min != index) {
        heap_swap(heap, index, min);
        heapify_down(heap, min);
    }
}

// Insert a vertex into the heap
void heap_insert(min_heap *heap, int vertex, int f_score) {
    if (heap->size == heap->capacity) return;

    int index = heap->size++;
    heap->nodes[index].vertex = vertex;
    heap->nodes[index].f_score = f_score;
    heap->positions[vertex] = index;

    heapify_up(heap, index);
}

// Extract the vertex with minimum f_score
int heap_extract_min(min_heap *heap, int *vertex) {
    if (heap->size == 0) return 0;

    *vertex = heap->nodes[0].vertex;
    heap->positions[*vertex] = -1;

    heap->nodes[0] = heap->nodes[--heap->size];
    if (heap->size > 0) {
        heap->positions[heap->nodes[0].vertex] = 0;
        heapify_down(heap, 0);
    }

    return 1;
}

// Decrease the f_score of a vertex
void heap_decrease_key(min_heap *heap, int vertex, int f_score) {
    int index = heap->positions[vertex];
    if (index == -1 || heap->nodes[index].f_score <= f_score) return;

    heap->nodes[index].f_score = f_score;
    heapify_up(heap, index);
}

// Check if heap is empty
int heap_is_empty(min_heap *heap) {
    return heap->size == 0;
}

// Free the heap
void free_min_heap(min_heap *heap) {
    free(heap->nodes);
    free(heap->positions);
    free(heap);
}

// Heuristic function (admissible)
int heuristic(int current, int goal) {
    return 0; // Simplest admissible heuristic for unit weights
}

// A* algorithm for minimum graph motion planning
int *a_star(graph **G, int v, int *obstacles, int start, int goal, int *path_length) {
    // Initialize arrays
    int *g_score = malloc(sizeof(int) * v);
    int *f_score = malloc(sizeof(int) * v);
    int *parent = malloc(sizeof(int) * v);
    int *closed = calloc(v, sizeof(int));
    min_heap *open = create_min_heap(v);

    for (int i = 0; i < v; i++) {
        g_score[i] = INT_MAX;
        f_score[i] = INT_MAX;
        parent[i] = -1;
    }

    // Initialize start vertex
    g_score[start] = 0;
    f_score[start] = heuristic(start, goal);
    heap_insert(open, start, f_score[start]);

    while (!heap_is_empty(open)) {
        int current;
        if (!heap_extract_min(open, &current)) break;

        // Skip if obstacle or already closed
        if (obstacles[current] || closed[current]) continue;

        closed[current] = 1;

        // Goal reached
        if (current == goal) {
            int *path = reconstruct_path(parent, start, goal, path_length);
            free(g_score);
            free(f_score);
            free(parent);
            free(closed);
            free_min_heap(open);
            return path;
        }

        // Explore neighbors
        graph *neighbor = G[current];
        while (neighbor != NULL) {
            int next = neighbor->x;
            if (obstacles[next] || closed[next]) {
                neighbor = neighbor->next;
                continue;
            }

            int tentative_g = g_score[current] + 1; // Unit weight

            if (tentative_g < g_score[next]) {
                parent[next] = current;
                g_score[next] = tentative_g;
                f_score[next] = g_score[next] + heuristic(next, goal);
                heap_decrease_key(open, next, f_score[next]);
                if (open->positions[next] == -1) {
                    heap_insert(open, next, f_score[next]);
                }
            }
            neighbor = neighbor->next;
        }
    }

    // No path found
    *path_length = 0;
    free(g_score);
    free(f_score);
    free(parent);
    free(closed);
    free_min_heap(open);
    return NULL;
}

// Reconstruct the path from parent array
int *reconstruct_path(int *parent, int start, int goal, int *path_length) {
    // Count path length
    *path_length = 0;
    int current = goal;
    while (current != -1) {
        (*path_length)++;
        current = parent[current];
    }

    // Allocate and fill path
    int *path = malloc(sizeof(int) * (*path_length));
    current = goal;
    for (int i = *path_length - 1; i >= 0; i--) {
        path[i] = current;
        current = parent[current];
    }

    return path;
}