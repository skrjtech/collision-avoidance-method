#include <stdlib.h>
#include "Array.h"

typedef struct _Array
{
    /* data */
    int row;
    int col;
    double **data;
} _Array;

static double *CreateVector(int num) {
    return (double*)malloc(sizeof(double) * num);
}
static double **CreateArray(int row, int col) {
    int i;
    double **array = (double**)malloc(sizeof(double*) * row);
    for (i = 0; i < row; i++) array[i] = (double*)malloc(sizeof(double) * col);
    return array;
}
static void Destroy(Array array) {
    int i;
    for (i = 0; i < array->row; i++) free(array->data[i]);
    free(array->data);
}

Array ArrayCreate(int row, int col) {
    Array array = (Array)malloc(sizeof(_Array) * 1);
    array->row = row;
    array->col = col;
    array->data = CreateArray(row, col);
    ArrayZeros(array);
    return array;
}
void ArrayZeros(Array array) {
    int i,j;
    for (i = 0; i < array->row; i++) 
        for (j = 0; j < array->col; j++) array->data[i][j] = 0.;
}
void ArrayOnes(Array array) {
    int i,j;
    for (i = 0; i < array->row; i++) 
        for (j = 0; j < array->col; j++) array->data[i][j] = 1.;
}
void ArrayFills(Array array, double val) {
    int i,j;
    for (i = 0; i < array->row; i++) 
        for (j = 0; j < array->col; j++) array->data[i][j] = val;
}
void ArrayADD(Array arrayA, Array arrayB) {
    int i,j;
    for (i = 0; i < arrayA->row; i++) 
        for (j = 0; j < arrayA->col; j++) arrayA->data[i][j] += arrayB->data[i][j];
}
void ArraySUB(Array arrayA, Array arrayB) {
    int i,j;
    for (i = 0; i < arrayA->row; i++) 
        for (j = 0; j < arrayA->col; j++) arrayA->data[i][j] -= arrayB->data[i][j];
}
void ArrayTIM(Array arrayA, Array arrayB) {
    int i,j;
    for (i = 0; i < arrayA->row; i++) 
        for (j = 0; j < arrayA->col; j++) arrayA->data[i][j] *= arrayB->data[i][j];
}
void ArrayDIV(Array arrayA, Array arrayB) {
    int i,j;
    for (i = 0; i < arrayA->row; i++) 
        for (j = 0; j < arrayA->col; j++) arrayA->data[i][j] /= arrayB->data[i][j];
}
double ArrayDOT(Array arrayA, Array arrayB) {
    int i;
    double sum = 0.;
    for (i = 0; i < arrayA->col; i++) sum += arrayA->data[0][i] * arrayB->data[0][i];
    return sum;
}
void ArrayPOW(Array arrayA, double x) {
    int i, j, k;
    for (i = 0; i < arrayA->row; i++) 
        for (j = 0; j < arrayA->col; j++) 
            for (k = 0; k < arrayA->col; k++) arrayA->data[i][j] *= arrayA->data[i][j];
}
void ArrayDestroy(Array array) {
    Destroy(array);
    free(array);
}
