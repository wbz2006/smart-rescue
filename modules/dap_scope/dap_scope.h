#ifndef DAP_SCOPE_H
#define DAP_SCOPE_H

#include <stdint.h>

#define DAP_SCOPE_RTT_CH 1U
#define DAP_SCOPE_BUF_SIZE 1024U
#define DAP_SCOPE_META_INTERVAL 250U
#define DAP_SCOPE_MAX_FIELDS 8U
#define DAP_SCOPE_NAMES_MAX 200U

void dap_scope_plot(uint8_t field_count, const char *names, const float *values);

#define DAP_SCOPE_CAT_(a, b) a##b
#define DAP_SCOPE_CAT(a, b) DAP_SCOPE_CAT_(a, b)
#define DAP_SCOPE_NARG(...) DAP_SCOPE_NARG_(__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1)
#define DAP_SCOPE_NARG_(_1, _2, _3, _4, _5, _6, _7, _8, N, ...) N

#define SCOPE_PLOT(...) \
    DAP_SCOPE_CAT(DAP_SCOPE_PLOT_, DAP_SCOPE_NARG(__VA_ARGS__))(__VA_ARGS__)

#define DAP_SCOPE_PLOT_1(a) \
    do { const float values_[1] = {(float)(a)}; dap_scope_plot(1U, #a, values_); } while (0)
#define DAP_SCOPE_PLOT_2(a, b) \
    do { const float values_[2] = {(float)(a), (float)(b)}; dap_scope_plot(2U, #a "," #b, values_); } while (0)
#define DAP_SCOPE_PLOT_3(a, b, c) \
    do { const float values_[3] = {(float)(a), (float)(b), (float)(c)}; dap_scope_plot(3U, #a "," #b "," #c, values_); } while (0)
#define DAP_SCOPE_PLOT_4(a, b, c, d) \
    do { const float values_[4] = {(float)(a), (float)(b), (float)(c), (float)(d)}; dap_scope_plot(4U, #a "," #b "," #c "," #d, values_); } while (0)
#define DAP_SCOPE_PLOT_5(a, b, c, d, e) \
    do { const float values_[5] = {(float)(a), (float)(b), (float)(c), (float)(d), (float)(e)}; dap_scope_plot(5U, #a "," #b "," #c "," #d "," #e, values_); } while (0)
#define DAP_SCOPE_PLOT_6(a, b, c, d, e, f) \
    do { const float values_[6] = {(float)(a), (float)(b), (float)(c), (float)(d), (float)(e), (float)(f)}; dap_scope_plot(6U, #a "," #b "," #c "," #d "," #e "," #f, values_); } while (0)
#define DAP_SCOPE_PLOT_7(a, b, c, d, e, f, g) \
    do { const float values_[7] = {(float)(a), (float)(b), (float)(c), (float)(d), (float)(e), (float)(f), (float)(g)}; dap_scope_plot(7U, #a "," #b "," #c "," #d "," #e "," #f "," #g, values_); } while (0)
#define DAP_SCOPE_PLOT_8(a, b, c, d, e, f, g, h) \
    do { const float values_[8] = {(float)(a), (float)(b), (float)(c), (float)(d), (float)(e), (float)(f), (float)(g), (float)(h)}; dap_scope_plot(8U, #a "," #b "," #c "," #d "," #e "," #f "," #g "," #h, values_); } while (0)

#endif
