#include "dap_scope.h"

#include <stddef.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "bsp_dwt.h"

#ifndef DAP_SCOPE_TIME_US
#define DAP_SCOPE_TIME_US() ((uint32_t)DWT_GetTimeline_us())
#endif

#define DAP_SCOPE_MAGIC 0x31435344U

static char scope_buffer[DAP_SCOPE_BUF_SIZE];
static char names_cache[DAP_SCOPE_NAMES_MAX + 1U];
static uint8_t scope_initialized;
static uint16_t frame_count;

static void send_metadata(uint8_t field_count, const char *names)
{
    uint8_t frame[8U + DAP_SCOPE_NAMES_MAX];
    const size_t names_length = strlen(names);
    const uint32_t magic = DAP_SCOPE_MAGIC;

    memcpy(frame, &magic, sizeof(magic));
    frame[4] = field_count;
    frame[5] = (uint8_t)names_length;
    frame[6] = 0U;
    frame[7] = 0U;
    memcpy(frame + 8U, names, names_length);
    SEGGER_RTT_Write(DAP_SCOPE_RTT_CH, frame, (unsigned)(8U + names_length));
}

void dap_scope_plot(uint8_t field_count, const char *names, const float *values)
{
    uint8_t frame[sizeof(uint32_t) + DAP_SCOPE_MAX_FIELDS * sizeof(float)];
    uint32_t timestamp;
    size_t names_length;

    if ((field_count == 0U) || (field_count > DAP_SCOPE_MAX_FIELDS) ||
        (names == NULL) || (values == NULL))
    {
        return;
    }

    if (!scope_initialized)
    {
        if (SEGGER_RTT_ConfigUpBuffer(DAP_SCOPE_RTT_CH, "DAPScope", scope_buffer,
                                      sizeof(scope_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP) < 0)
        {
            return;
        }
        scope_initialized = 1U;
    }

    if (strncmp(names_cache, names, DAP_SCOPE_NAMES_MAX) != 0)
    {
        names_length = strlen(names);
        if (names_length > DAP_SCOPE_NAMES_MAX)
        {
            return;
        }
        memcpy(names_cache, names, names_length + 1U);
        send_metadata(field_count, names_cache);
        frame_count = 0U;
    }
    else if (++frame_count >= DAP_SCOPE_META_INTERVAL)
    {
        send_metadata(field_count, names_cache);
        frame_count = 0U;
    }

    timestamp = DAP_SCOPE_TIME_US();
    memcpy(frame, &timestamp, sizeof(timestamp));
    memcpy(frame + sizeof(timestamp), values, field_count * sizeof(float));
    SEGGER_RTT_Write(DAP_SCOPE_RTT_CH, frame,
                     (unsigned)(sizeof(timestamp) + field_count * sizeof(float)));
}
