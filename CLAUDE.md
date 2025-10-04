# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **N-Car (inverted pendulum car)** control system for the **Infineon TC264D microcontroller**. It implements a self-balancing two-wheeled vehicle using:
- **Momentum wheel** for balance control (动量轮)
- **Drive wheel** for forward/backward motion (行进轮)
- IMU sensor (imu660rb) for attitude measurement
- Cascaded PID control loops for stabilization

The codebase is based on the **SEEKFREE TC264 V3.4.1** open-source library.

## Build System

This project uses the **AURIX Development Studio (ADS)** IDE with the **TASKING compiler** for TC26x microcontrollers.

### Build Commands
```bash
# Build the project (in ADS IDE)
# Project -> Build Project

# Clean the project
# Project -> Clean

# Flash to hardware
# Use AURIX Development Studio's flash tool
```

The project is configured for:
- **Target**: TC26xB (tc26xb)
- **Package**: BGA292
- **Build configuration**: Debug
- **Compiler**: TASKING C/C++ Compiler

Build outputs are in the `Debug/` directory. The generated binary is flashed to the TC264D microcontroller.

## Code Architecture

### Directory Structure

```
code/              - Application code (user-written modules)
├── imu.c/h        - IMU sensor interface (imu660rb)
├── motor.c/h      - Motor control (momentum wheel + drive wheel)
├── servo.c/h      - Servo control
├── pid.c/h        - Cascaded PID controllers (3-loop control system)
├── menu.c/h       - Parameter tuning menu system (IPS114 display)
├── param_save.c/h - Flash parameter persistence (auto-save/load)

user/              - System entry points and interrupts
├── cpu0_main.c    - Main program entry (CPU0)
├── isr.c          - Interrupt service routines (PIT, UART, DMA, EXTI)

libraries/
├── infineon_libraries/ - Infineon iLLD low-level drivers (DO NOT MODIFY)
├── zf_driver/          - SEEKFREE hardware abstraction layer (PWM, GPIO, UART, etc.)
├── zf_device/          - SEEKFREE device drivers (IMU, display, sensors)
├── zf_common/          - Common utilities and type definitions
└── zf_components/      - Additional components (seekfree assistant, etc.)
```

### Control System Architecture

The N-Car uses a **cascaded triple-loop PID controller** for balance:

1. **Speed Loop** (outermost) → outputs desired angle offset
   - Input: encoder speed vs target speed
   - Output: angle offset for angle loop

2. **Angle Loop** (middle) → outputs desired angular velocity
   - Input: pitch angle (from IMU) + speed loop output
   - Output: target gyro rate for gyro loop

3. **Gyro Loop** (innermost) → outputs motor PWM
   - Input: gyro Y-axis rate vs target rate
   - Output: PWM value for momentum wheel motor

**Separate controller** for the drive wheel:
- Single-loop speed PID controlling drive wheel motor

All PID controllers are in [pid.c](code/pid.c) and [pid.h](code/pid.h).

### Key Control Flow

1. **Initialization** ([cpu0_main.c:56-74](user/cpu0_main.c#L56)):
   - Hardware initialization (clock, debug UART)
   - Peripheral initialization (IPS114 display, IMU, motors, encoders)
   - PID controller initialization
   - Timer initialization (1ms PID control, 20ms key scanning)
   - Load saved parameters from flash
   - Menu system initialization

2. **Main Loop** ([cpu0_main.c:81-90](user/cpu0_main.c#L81)):
   - Menu display and updates
   - Motor control outputs
   - 10ms delay to reduce CPU usage

3. **Interrupt-based Control** ([isr.c](user/isr.c)):
   - **CCU60_CH0 (1ms)**: Reserved for PID control (currently disabled)
   - **CCU60_CH1 (20ms)**: Key scanning with long-press support
   - **UART interrupts**: Debug, camera, wireless module, GNSS
   - **DMA interrupts**: Camera data acquisition
   - **EXTI interrupts**: Camera vsync, ToF sensor

### Parameter System

Parameters are managed through a **menu system** displayed on IPS114 LCD:
- Interactive parameter tuning via 4 buttons (UP/DOWN/OK/BACK)
- Automatic save/load to/from flash memory ([param_save.c](code/param_save.c))
- No code modification needed when adding new parameters - just update menu structure

Menu pages are defined in [menu.c](code/menu.c):
- Servo parameters
- Gyro PID parameters
- Angle PID parameters
- Speed PID parameters
- Drive wheel speed PID parameters

**Adding new parameters:**
1. Add parameter page in [menu.c](code/menu.c)
2. Add page pointer to `param_pages[]` array in [param_save.c:24-32](code/param_save.c#L24)
3. Parameters auto-save to flash when changed via menu

### IMU and Attitude Estimation

IMU sensor: **imu660rb** (6-axis accelerometer + gyroscope)

Attitude calculation uses **first-order complementary filter** ([imu.c:86-98](code/imu.c#L86)):
- Combines gyro integration with accelerometer feedback
- Only pitch angle is calculated (roll/yaw set to 0)
- Gyro zero-bias calibration supported
- Machine center angle offset configurable

## Common Development Tasks

### Modifying PID Parameters

PID parameters can be adjusted in two ways:

1. **Runtime tuning** (recommended):
   - Use the menu system on IPS114 display
   - Press buttons to navigate and adjust values
   - Parameters automatically saved to flash

2. **Code modification**:
   - Edit initial values in [pid.c:7-39](code/pid.c#L7)
   - Rebuild and flash

### Adding New Control Features

When adding new control logic:
- Control loops should run in timer interrupts (modify [isr.c:42-48](user/isr.c#L42))
- Do NOT call blocking functions in interrupts
- Use `interrupt_global_enable(0)` if interrupt nesting is required
- Update encoder readings via `motor_encoder_update()` before using encoder values

### Working with Motors

**Momentum wheel** (balance control):
```c
momentum_wheel_control(pwm_value);  // Range: -10000 to 10000
```

**Drive wheel** (forward/backward):
```c
drive_wheel_control(pwm_value);     // Range: -10000 to 10000
```

Both functions handle:
- PWM range limiting
- Direction pin control
- Sign handling (positive = forward, negative = reverse)

### Flash Parameter Storage

The parameter system automatically handles save/load:
- Parameters saved to flash when modified via menu
- Loaded at startup in `Param_Load_All()` ([cpu0_main.c:71](user/cpu0_main.c#L71))
- Uses hash-based identification for version safety

**Important**: Call `Param_Load_All()` BEFORE `Menu_Init()` in initialization sequence.

## Hardware Pin Assignments

Key pin definitions are in header files under [code/](code/):

**Motor control** ([motor.h](code/motor.h)):
- Momentum wheel: PWM_CH1, DIR_CH1, ENCODER1
- Drive wheel: PWM_CH2, DIR_CH2, ENCODER2

**Display and buttons** ([menu.h](code/menu.h)):
- IPS114 LCD (SPI interface)
- 4 buttons: KEY1-KEY4 (active low with pull-up)

**IMU sensor**:
- imu660rb (SPI interface, configured in zf_device)

Refer to Chinese text files for complete pin recommendations:
- [推荐IO分配.txt](推荐IO分配.txt)
- [尽量不要使用的引脚.txt](尽量不要使用的引脚.txt)

## Important Notes

- **DO NOT modify files** in `libraries/infineon_libraries/` - these are Infineon's official iLLD drivers
- The project uses GPL3.0 license - maintain SEEKFREE copyright notices when modifying
- Interrupt nesting is disabled by default on TC264 - must explicitly enable with `interrupt_global_enable(0)`
- The control system requires IMU to be initialized successfully or it will loop indefinitely at startup ([imu.c:33-49](code/imu.c#L33))
- Memory sections can be assigned to specific CPUs using `#pragma section` directives (project uses dual-core TC264)
