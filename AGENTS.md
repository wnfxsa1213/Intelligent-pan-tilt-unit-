# Repository Guidelines

## Project Structure & Module Organization
Firmware sources live in `Core/Src`, with matching headers in `Core/Inc`; keep module-facing declarations in headers and limit cross-module coupling. Vendor and HAL dependencies are under `Drivers/STM32F1xx_HAL_Driver`, while CMSIS startup assets sit in `Drivers/CMSIS` and the root linker script `STM32F103XX_FLASH.ld`. Shared documentation, including coding standards, is collected in `docs/`, and the CubeMX project file `IMU.ioc` is the single source of truth for pinouts and clock trees. Place any new board assets or calibration data under `docs/` or a dedicated `assets/` subdirectory so they stay versioned.

## Build, Test, and Development Commands
- `make all` — builds `build/IMU.elf`, `.hex`, and `.bin` using the arm-none-eabi toolchain; pass `DEBUG=0` for release-grade optimization.
- `make clean` — removes the `build/` directory prior to a fresh build.
- `arm-none-eabi-gdb build/IMU.elf` — attach to a running target for on-device inspection; pair with your preferred ST-Link/OpenOCD setup.
- `make BUILD_DIR=build_debug` — stage parallel configuration builds without clobbering artifacts.

## Coding Style & Naming Conventions
Adhere to the STM32 C rules codified in `docs/编程规范.md`: 4-space indentation, no tabs, and lower_snake_case for variables and functions. Reserve UpperCamelCase for public typedefs and ALL_CAPS for macros or register masks. Keep HAL includes ordered (`stm32f1xx_hal.h` before module headers) and annotate non-obvious hardware assumptions with concise comments right where they apply. When regenerating code through CubeMX, immediately normalize formatting before committing.

## Testing Guidelines
Automated unit tests are not yet wired in; treat `make all` as the minimum gate before any merge. Validate sensor pipelines on hardware with a serial session (`Core/Src/usart.c`) watching for saturation or timing drift, and capture UART logs for regressions. When introducing critical math, add lightweight assert-style checks that compile out under `DEBUG=0`. Document manual verification steps in the pull request body so others can reproduce them.

## Commit & Pull Request Guidelines
Follow the Conventional Commits format already in history (`type(scope): subject`), keeping the scope aligned with top-level modules such as `imu`, `servo`, or `attitude`. Bundle related changes only, and keep bodies bulletized when explaining multi-file updates. Pull requests must include: a clear summary, linked task IDs or issues, build/test evidence (`make all` output or hardware log excerpts), and screenshots or oscillograms when UI or timing behavior changes. Request review from a maintainer who owns the touched subsystem before merging.

## Hardware & Configuration Tips
Treat `IMU.ioc` as canonical; regenerate code and review the diff whenever clock, DMA, or GPIO settings change. Update `Core/Src/system_stm32f1xx.c` if CubeMX alters system clocks, and verify flash/stack sizes still fit the linker script. Check in any tuned calibration constants with comments referencing firmware revisions so future builds remain reproducible.
