#include "File.h"

#include "stdio.h"

typedef struct _File
{
    /* data */
    FILE *fp;
    char* fileName;
} _File;

FILE_STATUS_CODE OpenFile(File fp, char* mode) {
    if ((fp->fp = fopen(fp->fileName, mode)) == NULL) return FILE_OPEN_NG;
    return FILE_OPEN_OK;
}

FILE_STATUS_CODE FileWriteCSV(File fp) {

    return FILE_WRITE_OK;
}

FILE_STATUS_CODE CloseFile(File fp) {
    if ((fp->fp = fclose(fp->fp)) != 0) return FILE_CLOSE_NG;
    return FILE_CLOSE_OK;
}
