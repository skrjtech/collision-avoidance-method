#ifndef INCLUDE_ARRAY_HEADER
#define INCLUDE_ARRAY_HEADER

typedef struct _Array *Array;

Array ArrayCreate(int x, int y);
void ArrayZeros(Array array);
void ArrayOnes(Array array);
void ArrayFills(Array array, double val);
void ArrayADD(Array arrayA, Array arrayB);
void ArraySUB(Array arrayA, Array arrayB);
void ArrayTIM(Array arrayA, Array arrayB);
void ArrayDIV(Array arrayA, Array arrayB);
void ArrayDOT(Array arrayA, Array arrayB);
void ArrayPOW(Array arrayA);
void ArrayDestroy(Array array);

#endif