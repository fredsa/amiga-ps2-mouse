# Amiga 1000 PS/2 Mouse Adapter (`ESP32S3 Dev Module`)

An Arduino project for the **ESP32S3 Dev Module** that connects a standard PS/2 mouse to an **Amiga 1000** DE-9 (DB9) mouse port. The microcontroller handles bidirectional PS/2 host communication (initialization, IntelliMouse detection, hot-plug recovery, and movement/button packet decoding) and translates reports into Amiga quadrature signals (`V`, `H`, `VQ`, `HQ`) and active-LOW open-drain button signals (`LMB`, `RMB`, `MMB`).

This project was authored by AI with human prompting and feedback.

---

## Project Structure

- [`amiga-ps2-mouse.ino`](amiga-ps2-mouse.ino) — Main Arduino sketch (`setup()` and `loop()`), sensitivity preset switching, and serial diagnostics.
- [`config.h`](config.h) — GPIO pin mapping for the `ESP32S3 Dev Module`, PS/2 sample rate/resolution settings, and Amiga quadrature timing constants.
- [`ps2_mouse.h`](ps2_mouse.h) / [`ps2_mouse.cpp`](ps2_mouse.cpp) — Interrupt-driven PS/2 host controller supporting Host-to-Device commands, 11-bit frame reception, sync validation, and hot-plug recovery.
- [`amiga_mouse.h`](amiga_mouse.h) / [`amiga_mouse.cpp`](amiga_mouse.cpp) — Non-blocking microsecond-paced 2-bit Gray-code quadrature generator and open-drain button driver tailored to the Amiga 1000 (`Denise` 8362 `JOY0DAT` & `CIA-A`/`Paula`).

---

## Hardware Wiring

### 1. PS/2 Mouse (6-Pin Mini-DIN Female) to `ESP32S3 Dev Module`

> **Important (5V vs 3.3V Logic):** Standard PS/2 mice require **+5V DC** power (`VBUS` / `5V`) and use 5V open-collector signaling on `CLK` and `DATA`. Because ESP32-S3 GPIOs are **3.3V only**, connect a **bidirectional logic level shifter** (such as a 4-channel BSS138 module) between the PS/2 `CLK`/`DATA` lines and the ESP32-S3 GPIOs, with 4.7 kΩ–10 kΩ pull-up resistors to 5V on the PS/2 side.

| PS/2 Mini-DIN-6 Pin | Signal | Level Shifter (HV 5V -> LV 3.3V) | ESP32-S3 GPIO |
| :---: | :--- | :---: | :---: |
| **Pin 1** | `DATA` | `HV1` <-> `LV1` | **GPIO 5** |
| **Pin 2** | *Not Connected* | — | — |
| **Pin 3** | `GND` | `GND` | **GND** |
| **Pin 4** | `+5V Vcc` | `HV` (+5V) | **5V / VBUS** |
| **Pin 5** | `CLK` (Clock) | `HV2` <-> `LV2` | **GPIO 4** |
| **Pin 6** | *Not Connected* | — | — |

---

### 2. `ESP32S3 Dev Module` to Amiga 1000 Mouse Port (DE-9 / DB9 Female Plug)

The Amiga 1000 Mouse Port (Port 1) uses a standard DE-9 male connector on the computer:
- **Quadrature lines (Pins 1–4)** feed a `74LS157` TTL multiplexer (`V_IL <= 0.8V`, `V_IH >= 2.0V`) connected to the `Denise` (8362) chip. The ESP32-S3's 3.3V CMOS outputs cleanly satisfy 5V TTL input thresholds, or you may optionally buffer them with a 74HCT245 / level shifter.
- **Button lines (Pins 5, 6, 9)** are pulled up internally inside the Amiga (`CIA-A` for Left button, `Paula` `POTGO` for Right/Middle buttons) and are driven as active-LOW open-drain outputs.

| Amiga 1000 DE-9 Pin | Amiga Signal | Direction | ESP32-S3 GPIO | Notes |
| :---: | :--- | :---: | :---: | :--- |
| **Pin 1** | `V-pulse` (Vertical Y1) | ESP32 -> Amiga | **GPIO 6** | Quadrature Y phase A |
| **Pin 2** | `H-pulse` (Horizontal X1) | ESP32 -> Amiga | **GPIO 7** | Quadrature X phase A |
| **Pin 3** | `VQ-pulse` (Vertical Y2) | ESP32 -> Amiga | **GPIO 15** | Quadrature Y phase B |
| **Pin 4** | `HQ-pulse` (Horizontal X2) | ESP32 -> Amiga | **GPIO 16** | Quadrature X phase B |
| **Pin 5** | `BUTTON 3` (`MMB` / `POTY`) | ESP32 -> Amiga | **GPIO 17** | Active LOW (Open-Drain) |
| **Pin 6** | `BUTTON 1` (`LMB` / `FIRE0`) | ESP32 -> Amiga | **GPIO 18** | Active LOW (Open-Drain) |
| **Pin 7** | `+5V DC` (max 50 mA) | Amiga -> ESP32 | *(Optional 5V)* | Power ESP32 via USB or regulate carefully |
| **Pin 8** | `GND` (Signal Ground) | Common | **GND** | **Must** be connected to ESP32-S3 GND |
| **Pin 9** | `BUTTON 2` (`RMB` / `POTX`) | ESP32 -> Amiga | **GPIO 8** | Active LOW (Open-Drain) |

---

## Building and Flashing (Arduino IDE)

1. Install **Arduino IDE 2.x** and the **esp32 by Espressif Systems** board package via Boards Manager.
2. Open [`amiga-ps2-mouse.ino`](amiga-ps2-mouse.ino).
3. Select **Tools -> Board -> esp32 -> ESP32S3 Dev Module**.
4. Recommended Board Settings:
   - **USB CDC On Boot**: `Enabled` (for Serial diagnostics over native USB)
   - **CPU Frequency**: `240MHz (WiFi)`
   - **Upload Mode**: `UART0 / Hardware CDC`
5. Click **Upload**.

---

## Features & Operation

- **Anti-Overflow Quadrature Pacing:** The Amiga `Denise` chip uses an 8-bit signed counter per axis (`JOY0DAT`, `-128` to `+127`) sampled once per 50 Hz PAL (20 ms) or 60 Hz NTSC (16.7 ms) vertical blanking interrupt. The driver adaptively paces quadrature transitions down to a minimum interval of `200 µs` (max 100 counts per 20 ms frame), preventing pointer reversal on fast mouse flicks.
- **Configuration Mode (Sensitivity & Left/Right Button Swap):**
  - Physical Left and Right buttons are **swapped by default** (`AMIGA_DEFAULT_SWAP_BUTTONS = 1` in [`config.h`](config.h)).
  - Hold **Left + Middle + Right** mouse buttons simultaneously for **1 second** to enter **Configuration Mode** (acknowledged by **1 Up/Down cursor jiggle**). While in configuration mode, mouse movement and clicks are not passed to the Amiga.
  - Release all 3 buttons, then click a single physical button (always physical, regardless of swap mode):
    - **Left Click**: Decrease sensitivity by 1 level (minimum `#1`), acknowledged by `N` **Up/Down** jiggles (`1`–`6`).
    - **Right Click**: Increase sensitivity by 1 level (maximum `#6`), acknowledged by `N` **Up/Down** jiggles (`1`–`6`).
    - **Middle Click**: Toggle the **Swap Left/Right Buttons** flag, acknowledged by **1 Left/Right** wiggle.
  - **Sensitivity Presets (`SCALING_PRESETS` in [`amiga_mouse.cpp`](amiga_mouse.cpp)):**

    | Preset | Speed | Scale Ratio | Feedback (Up/Down Jiggles) | Notes |
    | :---: | :---: | :---: | :---: | :--- |
    | **#1** | `50%` | `1 : 2` | 1× Up/Down | Fine precision |
    | **#2** | `75%` | `3 : 4` | 2× Up/Down | — |
    | **#3** | `100%` | `1 : 1` | 3× Up/Down | 1:1 native counts |
    | **#4** | `125%` | `5 : 4` | 4× Up/Down | **Default** |
    | **#5** | `150%` | `3 : 2` | 5× Up/Down | — |
    | **#6** | `200%` | `2 : 1` | 6× Up/Down | Fast |

  - Real-time USB Serial packet and button logging is enabled by default via `ENABLE_USB_VERBOSE_DEBUG` in [`config.h`](config.h).
  - Send keys `1`–`6` (sensitivity), `b` (toggle button swap), `v` (toggle verbose logging), `s` (status), or `r` (reset mouse) over the 115200 baud USB Serial Monitor.
- **Hot-Plug Support:** Automatically detects the PS/2 `0xAA 0x00` Basic Assurance Test sequence if the mouse is unplugged and reconnected.
