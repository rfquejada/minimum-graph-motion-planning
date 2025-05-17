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

int main() {
    graph **g;
    int v, e;

    g = createAdjList(&v, &e);
    viewList(g, v);

    deleteGraph(g, v);
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