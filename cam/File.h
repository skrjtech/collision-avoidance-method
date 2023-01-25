#ifndef INCLUDE_FILE_HEADER
#define INCLUDE_FILE_HEADER

typedef enum {
    FILE_OPEN_NG,
    FILE_OPEN_OK,
    FILE_WRITE_NG,
    FILE_WRITE_OK,
    FILE_READ_NG,
    FILE_READ_OK,
    FILE_CLOSE_NG,
    FILE_CLOSE_OK
} FILE_STATUS_CODE;

typedef struct _File *File;

FILE_STATUS_CODE OpenFile(File fp, char* mode);
FILE_STATUS_CODE FileWriteCSV(File fp);
FILE_STATUS_CODE CloseFile(File fp);

#endif