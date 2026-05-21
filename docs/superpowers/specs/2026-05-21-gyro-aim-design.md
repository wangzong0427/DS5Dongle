# Gyro Aim Design

## Goal

Add optional gyro aiming for PC games that do not natively support DualSense motion input. The firmware will convert DualSense gyro motion into right-stick movement while preserving the existing USB DualSense device shape.

## Approved Direction

- Output mode: emulate right-stick movement, not mouse movement.
- Activation: enabled only while L2 is held past a configurable threshold.
- Axis mapping: yaw maps to right-stick X, pitch maps to right-stick Y.
- Composition: gyro output is added to the original right-stick value, then clamped.
- Configuration: expose a firmware configuration switch and tuning parameters through the existing HID config path so the web GUI can disable the feature for games with native gyro support.

## Architecture

Create a focused `src/gyro_aim.h/.cpp` module. It processes the 63-byte DualSense input report before `main.cpp` copies it to `interrupt_in_data` and before TinyUSB sends report ID `0x01` to the PC host.

The data flow becomes:

```text
DualSense BT 0x31 report
-> on_bt_data()
-> gyro_aim_process_report(report[63])
-> interrupt_in_data
-> tud_hid_report(0x01, interrupt_in_data, 63)
-> PC game sees normal DS5 right stick movement
```

`main.cpp` will only call the module. Bluetooth connection handling, USB descriptors, haptics, audio, and feature-report forwarding remain unchanged.

## Configuration

Extend `Config_body` with these fields:

```cpp
uint8_t gyro_aim_enabled;
float gyro_aim_sens_x;
float gyro_aim_sens_y;
uint8_t gyro_aim_deadzone;
uint8_t gyro_aim_smoothing;
uint8_t gyro_aim_max_output;
uint8_t gyro_aim_l2_threshold;
uint8_t gyro_aim_invert_x;
uint8_t gyro_aim_invert_y;
```

Defaults are conservative. `gyro_aim_enabled` defaults to `0`, so games with native gyro support are not affected unless the user enables the feature in the web GUI. Config validation will clamp invalid values and initialize new fields when old flash data is loaded.

## Algorithm

The processor reads the report fields documented in `utils.h`:

- `report[2]`: right-stick X
- `report[3]`: right-stick Y
- `report[4]`: L2 trigger
- `report[15..16]`: angular velocity X, used as pitch
- `report[19..20]`: angular velocity Y, used as yaw

Processing steps:

1. Return unchanged when `gyro_aim_enabled == 0`.
2. Return unchanged and reset smoothing when `L2 < gyro_aim_l2_threshold`.
3. Read yaw and pitch as signed little-endian 16-bit values.
4. Apply a gyro deadzone around zero.
5. Multiply by X/Y sensitivity.
6. Apply X/Y inversion flags.
7. Smooth the offset with a retained filtered value.
8. Clamp each axis to `[-gyro_aim_max_output, +gyro_aim_max_output]`.
9. Add each offset to the original right-stick byte and clamp to `[0, 255]`.

The module resets smoothing state when L2 is released so stale motion does not carry into the next aim activation.

## Risks

- Axis direction may need real-controller calibration. The first version includes X/Y inversion flags to compensate.
- Game-specific right-stick deadzones can swallow small output. Sensitivity and max-output are configurable for tuning.
- The web GUI must be updated to display the new config fields. Firmware remains safe because the feature defaults off.
- DualSense Edge may need axis-specific adjustment later. The first version uses the same mapping for DS5 and DSE.

## Verification

- Add host-buildable unit tests for the pure gyro aim processor behavior.
- Cover disabled mode, L2 gating, deadzone, inversion, max-output clamp, right-stick addition, and smoothing reset.
- Run the host tests locally.
- Run `dart analyze` because it is an approved existing check in this environment.
- Attempt firmware build if dependencies are available; if submodules/toolchain are missing, report the blocker clearly.
