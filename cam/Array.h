#ifndef INCLUDE_ARRAY_HEADER
#define INCLUDE_ARRAY_HEADER

typedef struct _Array *Array;

Array ArrayCreate(int row, int col);
void ArrayZeros(Array array);
void ArrayOnes(Array array);
void ArrayFills(Array array, double val);
void ArrayADD(Array arrayA, Array arrayB);
void ArraySUB(Array arrayA, Array arrayB);
void ArrayTIM(Array arrayA, Array arrayB);
void ArrayDIV(Array arrayA, Array arrayB);
double ArrayDOT(Array arrayA, Array arrayB);
void ArrayPOW(Array arrayA, double x);
void ArrayDestroy(Array array);

#endif