#pragma once
/* Firmware updates over Wi-Fi.

   Two app slots (partitions.csv). An update is written into the idle slot
   while the running one is untouched, verified, then booted. The new firmware
   must confirm itself (ota_update_confirm_later) or the bootloader returns to
   the previous one on the next reset.

   Sources:
   - GitHub: the latest release of CONFIG_REFLOW_OTA_REPO, its asset
     reflow-controller.bin, only for a production tag (exactly vX.Y.Z; alpha,
     beta and rc tags are never downloaded). Checked 60 s after boot, every
     24 h, and on request. Installed only when asked.
   - Upload: a .bin posted from the browser (ota_update_upload), any build of
     this firmware including alpha/beta.

   Never while a reflow run is active; a run cannot start during an update.
   The heater is switched off (and that confirmed) before writing starts. */
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

typedef enum {
    OTA_IDLE,          /* nothing checked yet */
    OTA_CHECKING,
    OTA_UP_TO_DATE,
    OTA_AVAILABLE,     /* a newer release was found */
    OTA_INSTALLING,    /* writing the idle slot */
    OTA_REBOOTING,
    OTA_ERROR,         /* see message */
} ota_state_t;

typedef struct {
    ota_state_t state;
    char        version[32];     /* running */
    char        build_date[40];  /* running */
    char        latest[32];      /* newest release found, "" if none */
    int         progress;        /* 0..100 while installing */
    int         last_check_s;    /* seconds since the last check, -1 never */
    char        message[96];     /* error or notice (e.g. a rollback) */
} ota_status_t;

/* After Wi-Fi is up. Starts the check task and the confirmation timer. */
esp_err_t   ota_update_init(void);

void        ota_update_get_status(ota_status_t *out);
bool        ota_update_busy(void);            /* installing or rebooting */
const char *ota_state_str(ota_state_t s);

/* Ask for a check now (asynchronous). */
esp_err_t   ota_update_check(void);

/* Install the release found by the last check (asynchronous). Returns
   ESP_ERR_INVALID_STATE if a run is active or nothing newer is available. */
esp_err_t   ota_update_install(void);

/* Stream the request body (a firmware .bin) into the idle slot, then reboot.
   Runs in the HTTP handler; answers the request itself. */
esp_err_t   ota_update_upload(httpd_req_t *req);
