# DAPScope on STM32H723

`dap_scope` sends self-describing float samples through RTT up-channel 1. Up-channel 0 remains the framework log channel; the existing J-Scope module retains channel 2.

The H723 link script places `.bss` in D1 SRAM at `0x24000000` with a size of `128 KB`. The RTT control block and DAPScope buffer are statically allocated there, so the host can scan that range directly. The host target name is `stm32h723xx`.

```c
#include "dap_scope.h"

SCOPE_PLOT(left_ref, left_measure, right_ref, right_measure);
```

`ChassisTask()` emits the two differential drive motors' speed PID reference and measured values at the robot task rate (200 Hz). To disable it for a production build, compile with `CHASSIS_SCOPE_ENABLE=0`.

The board clock is derived from its 24 MHz HSE. `BSPInit()` passes CMSIS `SystemCoreClock` to `DWT_Init()`, which supplies the microsecond timestamps. The current clock tree runs the H723 core at 240 MHz; this avoids reusing the F407 clock value from the original integration.

Install the host dependencies, then connect without resetting the running target:

```powershell
python -m daplink_jscope -t stm32h723xx --search-range 0x24000000,0x20000
```

The `--search-range` option can be repeated to include additional RAM regions. It should match the RAM used by the active link script.
