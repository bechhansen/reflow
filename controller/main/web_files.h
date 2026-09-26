#pragma once
/* Files embedded in the firmware at build time (tools/embed_web.py, from
   controller/web and controller/profiles), so an over-the-air update carries
   the web UI and the default profiles along with the code. */
#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char    *name;     /* web: URL path, e.g. "/index.html"; profiles: file name */
    const char    *mime;
    const uint8_t *data;     /* web: gzipped; profiles: as stored */
    size_t         len;
} embedded_file_t;

extern const embedded_file_t web_files[];
extern const size_t          web_files_count;
extern const embedded_file_t default_profiles[];
extern const size_t          default_profiles_count;
