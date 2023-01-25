#include <stdio.h>
#include "random.h"
#include "Object.h"

void PrintRand(int);

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
    return 0;
}