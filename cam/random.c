#include <stdlib.h>
#include <math.h>
#include "random.h"

void seed_set(int seed) {
    srand((unsigned int)seed);
}

int irand(void) { return rand() / MAXRANDOMVALUE; }
float frand(void) { return (float)rand() / MAXRANDOMVALUE; }
double drand(void) { return (double)rand() / MAXRANDOMVALUE; }

int irand_minmax(int min, int max) {
    return min + (int)(irand() * (max - min + 1.0) / (1.0 + MAXRANDOMVALUE));
}
float frand_minmax(float min, float max) {
    return min + (float)(frand() * (max - min + 1.0) / (1.0 + MAXRANDOMVALUE));
}
double drand_minmax(double min, double max) {
    return min + (double)(drand() * (max - min + 1.0) / (1.0 + MAXRANDOMVALUE));
}
float frand_uniform(void) {
    return (frand() + 1.) / (MAXRANDOMVALUE + 2.0);
}
double drand_uniform(void) {
    return (drand() + 1.) / (MAXRANDOMVALUE + 2.0);
}
// double drand_normadist(double mu, double sigma) {
//     double normal = sqrt(-2. * log(drand()) * sin(2. * M_PI * drand()));
//     return mu + sigma * normal;
// }