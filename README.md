# CNC-Brain

**CNC-Brain** is a modern, open-source CNC controller firmware designed as a flexible, extensible, and more maintainable alternative to legacy controllers like [GRBL](https://github.com/gnea/grbl). This project is currently focused on supporting the [Langmuir MR-1 CNC Mill](https://www.langmuirsystems.com/mr1) and is tailored for microcontrollers like the Raspberry Pi RP2040 and RP2350, making it ideal for low-cost, high-performance motion control systems.

## Goals and Philosophy

- **Modern Architecture**:  
  Leverage Rust’s strong type system, memory safety, and concurrency features for more reliable real-time control and easier long-term maintenance.
  
- **Extendability**:  
  Provide a cleaner, modular codebase that can be easily extended to support new hardware, kinematic configurations, and additional G-code commands.
  
- **Better Error Handling & Debugging**:  
  Offer comprehensive error reporting and robust debugging tools, improving the user experience during setup, calibration, and day-to-day operation.

- **Performance & Precision**:  
  Optimize for stable, smooth motion and high-accuracy CNC operations on embedded MCUs, ensuring that even low-cost hardware can deliver professional results.

- **Community-Friendly**:  
  Encourage contributions, experimentation, and adaptation for a wide variety of machines and workflows.

## Current Status

- **G-Code Parsing**:  
  A functional G-code parser has been implemented in Rust. It:
  - Supports common motion commands (G0, G1, G2, G3).
  - Handles modal commands for setting distance mode (G90/G91), units (G20/G21), feed rate modes (G93/G94), and more.
  - Interprets tool and spindle commands (M3, M4, M5, T#) as well as coolant and override modes.
  - Recognizes coordinate system setting commands (G10, G54–G59) and basic offset commands (G92 series).

- **Machine State Management**:  
  A preliminary machine state structure is in place. It:
  - Tracks position, feed rate, spindle speed, active work coordinate system (WCS), and other modal states.
  - Applies incremental or absolute moves as instructed by G-code.
  - Updates WCS offsets when `G10` commands are processed.

- **Serial Communication Mocking**:  
  A simple mock serial port implementation is included for testing the parser and state logic without actual hardware.

## Near-Term Future Work

- **Real-Time Motion Control**:  
  Implement the real-time motion planner and step generation routines required for controlling steppers or servos. This includes acceleration profiles, jerk-limited motion, and synchronization with input signals.

- **Hardware Integration**:  
  Add HAL (Hardware Abstraction Layer) code for interfacing with GPIO pins, motor drivers, encoders, limit switches, and spindle relays on RP2040/RP2350 microcontrollers.

- **Error Recovery & Stalling Protection**:  
  Handle runtime exceptions like limit switch triggers, spindle or coolant failures, and unexpected user interrupts gracefully.

- **Tool Length and Probe Routines**:  
  Implement standard tool setting, tool offset commands (G43 series), and probe commands (G38.x) to support automated tool measurement and workpiece probing.

- **UI and Control Interface**:  
  Provide hooks for a user interface (e.g., a touch panel, web interface, or pendant) and integrate communication over USB/serial/Wi-Fi.

## Long-Term Vision

- **Modular Codebase**:  
  Ensure that adding new G/M codes or machine operations is as simple as dropping in a new module or implementing a defined trait.

- **Robust Simulation and Dry-Run Modes**:  
  Integrate simulation capabilities for verifying toolpaths before cutting material, reducing the risk of crashes or wasted stock.

---

**CNC-Brain** is still in the early stages but has a clear roadmap and a strong technical foundation. We invite interested developers, makers, and machinists to get involved, test the code, suggest improvements, and help shape a modern CNC control solution for the future.
