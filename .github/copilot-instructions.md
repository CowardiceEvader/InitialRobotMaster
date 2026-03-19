# Copilot Instructions for This Firmware Repo

## Project Shape
- This is STM32F407 + FreeRTOS robot firmware, generated from STM32CubeMX and extended with DJI-style application modules.
- Startup flow is: HAL/Cube init in `Src/main.c`, then custom init (`can_filter_init`, `delay_init`, `cali_param_init`, `remote_control_init`, `aim_assistant_control_init`), then RTOS scheduler start.
- Real behavior lives in `application/` (control, comms, calibration), hardware drivers in `bsp/boards/`, algorithms/devices in `components/`, MCU/HAL glue in `Src/` and `Inc/`.

## Runtime Architecture (Read Before Editing)
- Task creation is centralized in `Src/freertos.c`; update this file when adding/removing runtime features.
- Fast loops:
  - `INS_task` runs IMU/AHRS update at high priority and feeds Euler angles.
  - `gimbal_task` consumes RC + IMU + motor feedback and sends gimbal currents through CAN.
  - `chassis_task` consumes RC + gimbal yaw relation + motor feedback and sends chassis currents through CAN.
- Control/data path is: UART SBUS/vision/referee input -> decoded structs (`remote_control.c`, `aim_assist_control.c`, `referee_usart_task.c`) -> behavior/state machines (`*_behaviour.c`, `*_task.c`) -> `CAN_receive.c` transmit helpers.

## Safety and Fault Pattern
- Online/offline health uses TOE hooks (`detect_hook`) and checks (`toe_is_error`) from `application/detect_task.c`.
- Important nuance: `detect_task` thread creation is currently commented out in `Src/freertos.c`; do not assume periodic fault evaluation is active unless you re-enable it.
- Motors are typically zeroed on control loss (example: DBUS offline branch in `application/chassis_task.c`). Preserve this fail-safe behavior.

## Code Conventions in This Repo
- Keep STM32Cube-generated files merge-safe: modify inside `/* USER CODE BEGIN */` blocks when touching Cube files (`Src/main.c`, `Src/freertos.c`, etc.).
- Task entry signatures use `void xxx_task(void const *pvParameters)` and rely on `vTaskDelay/osDelay` for deterministic loop periods.
- Cross-module access uses global getter pointers (examples: `get_remote_control_point`, `get_INS_angle_point`, `get_yaw_motor_point`). Prefer this pattern over new global extern structs.
- New sensors/devices should be wired through both:
  - detect framework (`errorList`, `detect_init` timings, `detect_hook` call sites), and
  - calibration framework (`application/calibrate_task.c` sensor table + flash serialization).

## Integration Points You Must Respect
- CAN receive/transmit IDs and motor index mapping are in `application/CAN_receive.c`; keep ID-to-index assumptions consistent.
- UART + DMA + IDLE interrupt parsing is core here:
  - RC SBUS on USART3 (`application/remote_control.c`, `bsp/boards/bsp_rc.c`)
  - Referee on USART6 (`application/referee_usart_task.c`)
  - Aim assist/vision bridge on USART1 (`application/aim_assist_control.c`)
- USB CDC telemetry/debug output is in `application/usb_task.c`.

## Build and Debug Workflow
- Primary build/debug environment is Keil uVision project `MDK-ARM/standard_robot.uvprojx` (target `standard_robot`, output under `MDK-ARM/standard_tpye_c/`).
- This repo does not include a native CMake/Ninja/Linux build pipeline; do not invent one in routine fixes.
- Cleanup helper script is `keilkilll.bat` (Windows-oriented artifact cleanup).
- There is no unit test harness in-tree; validate by compiling in Keil and smoke-testing key tasks/interrupt paths on target hardware.