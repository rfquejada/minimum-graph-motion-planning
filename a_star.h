#ifndef A_STAR_H
#define A_STAR_H

typedef struct node {
    int x;
    struct node *next;
} graph;

int *a_star(graph **G, int v, int *obstacles, int start, int goal, int *path_length); 

#endif
