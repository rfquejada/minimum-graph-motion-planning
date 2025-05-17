#include<stdio.h>
#include<stdlib.h>

//Structure for the graph
typedef struct node{
    int x;
    struct node *next;
}graph;

graph **createAdjList(int *, int *);
void viewList(graph **, int);
void deleteGraph(graph **, int);
int *readObstacles(int, int *);
void printObstacles(int *, int);

int main() {
    graph **g;
    int v, e, num_obstacles;

    g = createAdjList(&v, &e);
    int *obstacles = readObstacles(v, &num_obstacles);

    viewList(g, v);
    printObstacles(obstacles, v);

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

int *readObstacles(int v, int *num_obstacles) {
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

