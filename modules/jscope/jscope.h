#ifndef __JSCOPE_H__
#define __JSCOPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JSCOPE_CHANNEL_COUNT 4U
#define JSCOPE_RTT_BUFFER_INDEX 2U

/**
 * @brief Initialize the RTT up-buffer used by SEGGER J-Scope.
 * @return 0 on success, negative value if the buffer is unavailable.
 */
int JScopeInit(void);

/**
 * @brief Write one sample frame. Unspecified channels are sent as zero.
 * @return Number of bytes written (16) on success, otherwise 0.
 */
int JScopeWrite(int32_t ch0, int32_t ch1, int32_t ch2, int32_t ch3);

/**
 * @brief Write one sample frame using printf-style format specifiers.
 * @note Supported conversions are d, i, u, x, X and c. At most four
 *       conversions are accepted; missing channels are padded with zero.
 * @return Number of bytes written (16) on success, otherwise 0.
 */
int JScopePrintf(const char *format, ...);

/**
 * @brief Write multiple sample frames.
 * @param samples Sample-major array: samples[sample * channel_count + channel].
 * @param sample_count Number of sample frames.
 * @param channel_count Number of channels per frame, from 1 to 4.
 * @return Number of bytes written, or 0 if the complete batch was not written.
 * @note The complete batch must fit in the RTT buffer's current free space.
 */
int JScopeWriteSamples(const int32_t *samples, uint32_t sample_count,
                       uint8_t channel_count);

#ifdef __cplusplus
}
#endif

#endif
