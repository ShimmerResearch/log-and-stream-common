/*
 * shimmer_sd_companion.c
 *
 * See shimmer_sd_companion.h.
 */

#include "shimmer_sd_companion.h"

#if defined(SHIMMER3R)

#include <string.h>

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"

#include "ff.h"

uint8_t ShimSdCompanion_path(const char *root, const char *ext, char *out, uint16_t len)
{
  const char *data = ShimSdDataFile_getDataFileName();
  size_t rootLen = strlen(root);
  size_t extLen = strlen(ext);

  /* data/<experiment>/<session>/<NNN> */
  if (strncmp(data, "data/", 5U) != 0 || rootLen + 1U + strlen(data + 5U) + extLen >= len)
  {
    return 0;
  }
  strcpy(out, root);
  strcat(out, "/");
  strcat(out, data + 5U);
  strcat(out, ext);
  return 1;
}

uint8_t ShimSdCompanion_makeDirs(char *path)
{
  char *start = path;
  char *slash;
  FRESULT res;

  while ((slash = strchr(start, '/')) != 0)
  {
    *slash = '\0';
    res = f_mkdir(path);
    *slash = '/';
    if (res != FR_OK && res != FR_EXIST)
    {
      return 0;
    }
    start = slash + 1;
  }
  return 1;
}

#endif /* SHIMMER3R */
