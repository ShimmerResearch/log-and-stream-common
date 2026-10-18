/*
 * Host-test stub for FatFs, the SD card file system.
 *
 * The two firmwares ship different FatFs releases, and the library tells them
 * apart by _FATFS (see SDCard/shimmer_sd.h):
 *   - Shimmer3:  R0.08b, shimmer3-firmware
 *                LogAndStream_Shimmer3/Shimmer_Driver/FatFs/ff.h. Its directory
 *                type is DIRS, and FIL carries the file size.
 *   - Shimmer3R: R0.12c, shimmer3r-firmware
 *                LogAndStream_Shimmer3R/Middlewares/Third_Party/FatFs/src/ff.h.
 *                The directory type is DIR, the file size comes from f_stat(),
 *                and with _FS_EXFAT set in its ffconf.h, FSIZE_t is 64-bit.
 *
 * Only what the code under host test touches is here. The revision IDs, the
 * FRESULT codes and the FA_ mode bits are the real headers' values: a module
 * compares against them, so a wrong one would make a green test meaningless.
 * The structs carry only the members the library reads; their layout is not the
 * real one, and nothing here depends on it.
 *
 * Declarations only. A suite that drives a file defines these itself, as a fake
 * card - see test_calibration.c.
 */
#ifndef HOST_TEST_STUB_FF_H
#define HOST_TEST_STUB_FF_H

#include <stdint.h>

typedef unsigned int UINT;
typedef unsigned char BYTE;
/* unsigned long on both MCUs, where it is 32 bits; a 64-bit host's is wider. */
typedef uint32_t DWORD;
typedef char TCHAR;

#if defined(SHIMMER3)

#define _FATFS 8237 /* Revision ID */

typedef struct
{
  DWORD fptr;  /* File read/write pointer (0 on file open) */
  DWORD fsize; /* File size */
} FIL;

typedef struct
{
  DWORD sclust; /* Table start cluster (0:Root dir) */
} DIRS;

typedef struct
{
  DWORD fsize; /* File size */
} FILINFO;

#elif defined(SHIMMER3R)

#define _FATFS     68300 /* Revision ID */

typedef uint64_t QWORD;
typedef QWORD FSIZE_t; /* _FS_EXFAT is 1 in the Shimmer3R ffconf.h */

#define _MAX_LFN   255 /* the Shimmer3R ffconf.h */

typedef struct
{
  FSIZE_t objsize; /* Object size */
} _FDID;

typedef struct
{
  _FDID obj;    /* Object identifier - f_size() reads its size */
  FSIZE_t fptr; /* File read/write pointer (Zeroed on file open) */
} FIL;

typedef struct
{
  DWORD dptr; /* Current read/write offset */
} DIR;

typedef struct
{
  FSIZE_t fsize;             /* File size */
  BYTE fattrib;              /* File attribute */
  TCHAR fname[_MAX_LFN + 1]; /* Primary file name (_USE_LFN 2) */
} FILINFO;

/* The file system object - only declared, as the SD module's extern. */
typedef struct
{
  BYTE fs_type; /* File system type (0:N/A) */
} FATFS;

#define f_size(fp) ((fp)->obj.objsize)
#define AM_DIR     0x10 /* Directory */

#endif

typedef enum
{
  FR_OK = 0,    /* (0) Succeeded */
  FR_DISK_ERR,  /* (1) A hard error occurred in the low level disk I/O layer */
  FR_INT_ERR,   /* (2) Assertion failed */
  FR_NOT_READY, /* (3) The physical drive cannot work */
  FR_NO_FILE,   /* (4) Could not find the file */
  FR_NO_PATH,   /* (5) Could not find the path */
  FR_INVALID_NAME, /* (6) The path name format is invalid */
  FR_DENIED, /* (7) Access denied due to prohibited access or directory full */
  FR_EXIST,  /* (8) Access denied due to prohibited access */
  FR_INVALID_OBJECT,  /* (9) The file/directory object is invalid */
  FR_WRITE_PROTECTED, /* (10) The physical drive is write protected */
  FR_INVALID_DRIVE,   /* (11) The logical drive number is invalid */
  FR_NOT_ENABLED,     /* (12) The volume has no work area */
  FR_NO_FILESYSTEM,   /* (13) There is no valid FAT volume */
  FR_MKFS_ABORTED,    /* (14) The f_mkfs() aborted */
  FR_TIMEOUT,         /* (15) Could not get a grant to access the volume */
  FR_LOCKED, /* (16) The operation is rejected by the file sharing policy */
  FR_NOT_ENOUGH_CORE,     /* (17) LFN working buffer could not be allocated */
  FR_TOO_MANY_OPEN_FILES, /* (18) Number of open files exceeds the limit */
#if defined(SHIMMER3R)
  FR_INVALID_PARAMETER /* (19) Given parameter is invalid - R0.12c only */
#endif
} FRESULT;

/* File access mode bits - the same values in both releases. */
#define FA_READ          0x01
#define FA_OPEN_EXISTING 0x00
#define FA_WRITE         0x02
#define FA_CREATE_ALWAYS 0x08
#define FA_CREATE_NEW    0x04

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode);
FRESULT f_close(FIL *fp);
FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br);
FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw);
#if defined(SHIMMER3)
FRESULT f_opendir(DIRS *dp, const TCHAR *path);
#elif defined(SHIMMER3R)
FRESULT f_opendir(DIR *dp, const TCHAR *path);
FRESULT f_closedir(DIR *dp);
FRESULT f_readdir(DIR *dp, FILINFO *fno);
FRESULT f_lseek(FIL *fp, FSIZE_t ofs);
FRESULT f_sync(FIL *fp);
#endif
FRESULT f_mkdir(const TCHAR *path);
FRESULT f_stat(const TCHAR *path, FILINFO *fno);

#endif /* HOST_TEST_STUB_FF_H */
