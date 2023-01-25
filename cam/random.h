#ifndef INCLUDE_RANDOM_HEADER
#define INCLUDE_RANDOM_HEADER

#define MAXRANDOMVALUE 0x7fff

void seed_set(int seed);
// Normal Get Random value
int irand(void);
float frand(void);
double drand(void);
// Min Max Get Random Value
int irand_minmax(int, int);
float frand_minmax(float, float);
double drand_minmax(double, double);
// Uniform Random (0 < x < 1)
float frand_uniform(void);
double drand_uniform(void);
// Normal Distribution Random
// double drand_normadist(double, double);

#endif