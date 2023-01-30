#include <stdlib.h>
#include "Array.h"

typedef struct _Array
{
    /* data */
    int row;
    int col;
    double **data;
} _Array;

Array ArrayCreate(int row, int col) {
    Array array = (Array)malloc(sizeof(_Array) * 1);
    array->data = (double**)malloc(sizeof(double*) * row);
    int i;
    for (i = 0; i < col; i++) array->data[i] = (double*)malloc(sizeof(double) * row);
    array->row = row;
    array->col = col;
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
// void ArrayDOT(Array arrayA, Array arrayB) {
//     int i,j;
//     for (i = 0; i < arrayA->row; i++) 
//         for (j = 0; j < arrayA->col; j++) arrayA->data[i][j] /= arrayB->data[i][j];
// }
void ArrayDestroy(Array array) {
    int i;
    for (i = 0; i < array->col; i++) free(array->data[i]);
    free(array->data);
    free(array);
}
