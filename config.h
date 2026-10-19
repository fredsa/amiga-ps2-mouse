#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// ESP32-S3 Dev Module Pin Assignments
// ============================================================================
// Note: Avoid GPIO 0, 3, 45, 46 (strapping pins), GPIO 19, 20 (USB D-/D+),
// and GPIO 26-32 (SPI flash/PSRAM).
//
// PS/2 Mouse lines are 5V open-collector. Because the ESP32-S3 is 3.3V logic,
// use a bidirectional logic level converter (e.g., BSS138 4-channel module)
// between the PS/2 mouse and these GPIOs, or pull PS/2 DATA/CLK to 3.3V if the
// mouse supports 3.3V CMOS thresholds.
#define PIN_PS2_CLK             4   // PS/2 Mini-DIN-6 Pin 5 (Clock)
#define PIN_PS2_DATA            5   // PS/2 Mini-DIN-6 Pin 1 (Data)

// Amiga 1000 DE-9 (DB9) Mouse Port Connections:
// Amiga 1000 uses a 74LS157 (TTL input thresholds: V_IL <= 0.8V, V_IH >= 2.0V)
// feeding the Denise (8362) chip for quadrature, and CIA-A / Paula with
// internal pull-ups for mouse buttons.
#define PIN_AMIGA_V_PULSE       6   // Amiga DB9 Pin 1: V-pulse  (Vertical Y1)
#define PIN_AMIGA_H_PULSE       7   // Amiga DB9 Pin 2: H-pulse  (Horizontal X1)
#define PIN_AMIGA_VQ_PULSE      15  // Amiga DB9 Pin 3: VQ-pulse (Vertical Y2)
#define PIN_AMIGA_HQ_PULSE      16  // Amiga DB9 Pin 4: HQ-pulse (Horizontal X2)

#define PIN_AMIGA_MMB           17  // Amiga DB9 Pin 5: Middle Mouse Button (Active LOW)
#define PIN_AMIGA_LMB           18  // Amiga DB9 Pin 6: Left Mouse Button   (Active LOW)
#define PIN_AMIGA_RMB           8   // Amiga DB9 Pin 9: Right Mouse Button  (Active LOW)

// Onboard status LED (GPIO 2 or GPIO 48 on many ESP32-S3 DevKitC-1 boards)
#define PIN_STATUS_LED          PIN_RGB_LED

// Target ESP32-S3 CPU clock frequency in MHz (40 MHz minimizes current draw on
// Amiga 500 DB9 Pin 7 +5V rail across the 4.7 ohm R401 protection resistor)
#define ESP32_CPU_FREQ_MHZ      40

// ============================================================================
// PS/2 Mouse Configuration
// ============================================================================
// Sample rate in reports/second (valid PS/2 values: 10, 20, 40, 60, 80, 100, 200)
#define PS2_DEFAULT_SAMPLE_RATE 100

// Resolution code: 0 = 1 count/mm, 1 = 2 counts/mm, 2 = 4 counts/mm, 3 = 8 counts/mm
#define PS2_DEFAULT_RESOLUTION  2

// Minimum valid PS/2 clock period (microseconds): PS/2 clock is 10-16.7 kHz
// (60-100 us period). Edges closer than 35 us are bounce or crosstalk glitches.
#define PS2_MIN_CLOCK_PERIOD_US 35

// Intra-byte bit timeout (microseconds): if clock edges stop for longer than
// this within a single 11-bit byte, reset the bit state machine before the next
// byte in the packet arrives (inter-byte gap is typically 150-400 us).
#define PS2_FRAME_TIMEOUT_US    250

// Inter-packet byte timeout (microseconds): bytes within a 3/4-byte packet arrive
// ~1 ms apart, whereas packets arrive 10 ms apart (at 100 Hz). Any gap > 3.5 ms
// indicates the start of a new packet.
#define PS2_PACKET_TIMEOUT_US   3500

// Periodic watchdog interval (milliseconds): check if the PS/2 lines are stuck
// or if a mouse was hot-plugged.
#define PS2_WATCHDOG_MS         1000

// ============================================================================
// Amiga Quadrature & Timing Configuration
// ============================================================================
// Amiga's Denise chip maintains an 8-bit signed counter per axis in JOY0DAT
// (-128 to +127), which Intuition/gameport.device reads once per Vertical Blank
// (50 Hz PAL = 20 ms, 60 Hz NTSC = 16.67 ms).
//
// Real Amiga optomechanical mice produce quadrature transitions spaced by
// ~1-2+ ms. Pacing transitions between 1000 us (1.0 ms fastest) and 2500 us
// (2.5 ms slow) matches real mouse waveforms for the 74LS157/Denise inputs
// while keeping each 20 ms PAL frame well below the 127-count JOY0DAT limit.
#define AMIGA_MIN_STEP_INTERVAL_US  1000  // Fastest step interval (1.0 ms / 1,000 steps/s)
#define AMIGA_MAX_STEP_INTERVAL_US  2500  // Base step interval for slow movements (2.5 ms)

// Maximum queued delta counts per axis to prevent runaway lag after a huge flick
#define AMIGA_MAX_ACCUMULATED_DELTA 512

// Axis inversion flags (set to 1 to invert axis direction if desired)
#define AMIGA_INVERT_X          0
#define AMIGA_INVERT_Y          0

// Default sensitivity preset index (0-based: 0=#1 50%, 1=#2 75%, 2=#3 100%, 3=#4 125%, 4=#5 150%, 5=#6 200%)
#define AMIGA_DEFAULT_PRESET_INDEX  3

// Swap physical Left and Right mouse buttons by default (1 = swapped, 0 = normal)
#define AMIGA_DEFAULT_SWAP_BUTTONS  1

// Sensitivity feedback cursor wiggle parameters:
// Each Up/Down or Left/Right pair sends WIGGLE_STEP_DELTA WIGGLE_STEPS_PER_DIRECTION
// times in the first direction, then WIGGLE_STEPS_PER_DIRECTION times in the opposite
// direction (spaced by WIGGLE_LEG_DURATION_MS >= 1 VBlank), with WIGGLE_PAIR_PAUSE_MS
// pause between consecutive pairs.
#define WIGGLE_STEP_DELTA           16
#define WIGGLE_STEPS_PER_DIRECTION  2
#define WIGGLE_LEG_DURATION_MS      25
#define WIGGLE_PAIR_PAUSE_MS        250

// Enable serial diagnostic output at 115200 baud
#define ENABLE_SERIAL_DEBUG         1

// Enable real-time verbose logging of mouse buttons and movement packets over USB Serial
#define ENABLE_USB_VERBOSE_DEBUG    1

#endif // CONFIG_H
