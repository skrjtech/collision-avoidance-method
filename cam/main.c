#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "common.h"
#include "random.h"
#include "Object.h"
#include "Array.h"

#define DEGREE(x) ((x) * M_PI / 180)

int main(void) {
    seed_set(12345);
    Object Robot;
    Robot = ObjectCreate(
        0., 0., 0.,
        0., 0., 0.,
        0., 0., 0.,
        0., 0., 0.,
        10.
    );

    for (int i = 0; i < 10; i++) {
        ObjectSetVXYZ(Robot, drand_uniform(), drand_uniform(), drand_uniform());
        ObjectMove(Robot);
        ObjectPrint(Robot);
        printf("\n");
    }

    ObjectDestroy(Robot);
    double **ARRAY = (double**)malloc(sizeof(double*) * ARRAY_HEIGHT);
    for (int X = 0; X < ARRAY_HEIGHT; X++) ARRAY[X] = (double*)malloc(sizeof(double) * ARRAY_WIDTH);

    for (int X = 0; X < ARRAY_HEIGHT; X++) {
        for (int Y = 0; Y < ARRAY_WIDTH; Y++) {
            ARRAY[X][Y] = 0.;
        }
    }

    unsigned int sec;
    int nsec;
    double d_sec;
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_REALTIME, &start_time);

    for (int X = 0; X < ARRAY_HEIGHT; X++) {
        for (int Y = 0; Y < ARRAY_WIDTH; Y++) {
            // printf("X: %lf, Y: %lf\n", (float)DEGREE(X), (float)DEGREE(Y));
            ARRAY[X][Y] = X + Y;
        }
    }
    clock_gettime(CLOCK_REALTIME, &end_time);
    
    sec = end_time.tv_sec - start_time.tv_sec;
    nsec = end_time.tv_nsec - start_time.tv_nsec;
    
    d_sec = (double)sec + (double)nsec / (1000 * 1000 * 1000);

    printf("time:%f\n", d_sec);

    for (int X = 0; X < ARRAY_HEIGHT; X++) free(ARRAY[X]);
    free(ARRAY);

    Array array1 = ArrayCreate(ARRAY_HEIGHT, ARRAY_HEIGHT);
    Array array2 = ArrayCreate(ARRAY_HEIGHT, ARRAY_HEIGHT);
    ArrayFills(array2, drand_uniform());
    ArrayADD(array1, array2);
    ArrayDestroy(array1);
    ArrayDestroy(array2);

    return 0;
}