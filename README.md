# Rotating-Hex

This project is a low-level ARMv7-A assembly implementation of an interrupt-driven message rotator on the DE1-SoC FPGA board on hardware-mapped FPGA peripherals. It uses direct register manipulation and branching logic to coordinate between multiple I/O devices—HEX displays, slider switches, pushbuttons, and LEDs—through memory-mapped addresses (0xFF200000, 0xFFFEC100, etc..).

The core functionality involves rotating a byte-encoded message across six 7-segment HEX displays. The message is assembled at runtime based on the state of the slider switches, with conditional branching and bitmasking used to copy strings from .rodata into a buffer. Pushbuttons serve as interrupt triggers, controlling system state (pause, reverse, speed adjustment), with the behavior managed via branching in the IRQ handler.

Timing is driven by the ARM private timer configured at 0xFFFEC600, and all interrupts are routed through the GIC, which is manually configured to map interrupt sources (IDs 29 for timer, 73 for pushbuttons) to the CPU. Interrupt service routines clear pending flags and update global flags, which the main loop polls and acts on with control flow logic. System mode transitions and stack pointer setup for IRQ and SVC modes are performed at startup using CPSR instructions.

Challenges included handling wraparound rotation logic, race conditions between poll and interrupt timing, and ensuring proper state restoration after nested subroutine calls. All control flow—message assembly, rotation, and device updates—is governed by manual register-based branching and inlined subroutines, with no reliance on high-level constructs or runtime environments.
