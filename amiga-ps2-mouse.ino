/*
 * Amiga 1000 PS/2 Mouse Adapter for ESP32-S3 Dev Module
 *
 * Translates bidirectional PS/2 mouse protocol packets (movement + 3 buttons)
 * into Amiga 1000 DE-9 (DB9) quadrature pulses (V, H, VQ, HQ) and active-LOW
 * open-drain button signals (LMB, RMB, MMB).
 *
 * Target Board: ESP32S3 Dev Module (Arduino-ESP32 core)
 */

#include <Arduino.h>
#include "config.h"
#include "ps2_mouse.h"
#include "amiga_mouse.h"

static PS2Mouse ps2Mouse(PIN_PS2_CLK, PIN_PS2_DATA);

static AmigaMouse amigaMouse(
    PIN_AMIGA_V_PULSE,
    PIN_AMIGA_H_PULSE,
    PIN_AMIGA_VQ_PULSE,
    PIN_AMIGA_HQ_PULSE,
    PIN_AMIGA_LMB,
    PIN_AMIGA_RMB,
    PIN_AMIGA_MMB
);

// Configuration mode state machine triggered by holding physical LMB + RMB for 1 second,
// followed by two physical clicks:
//   - Left then Left   -> Decrease sensitivity
//   - Right then Right -> Increase sensitivity
//   - Left then Right (or Right then Left) -> Toggle Left/Right button swap
enum ConfigState : uint8_t {
    CFG_IDLE = 0,
    CFG_WAIT_RELEASE_ALL,   // Waiting for both buttons to be released after entering config mode
    CFG_WAIT_TWO_CLICKS,    // Waiting for two-click config selection (L+L, R+R, or L+R / R+L)
    CFG_WAIT_ACK_COMPLETE   // Waiting for 2nd clicked button release AND full acknowledgment jiggle completion
};

enum ConfigClick : uint8_t {
    CFG_CLICK_NONE = 0,
    CFG_CLICK_LEFT,
    CFG_CLICK_RIGHT,
    CFG_CLICK_SWAP
};

static ConfigState s_configState           = CFG_IDLE;
static ConfigClick s_cfgFirstClick         = CFG_CLICK_NONE;
static bool        s_cfgFirstClickReleased = false;
static ConfigClick s_cfgPendingAction      = CFG_CLICK_NONE;
static bool        s_cfgClickReleased      = false;
static uint32_t    s_chordStartMs          = 0;
static bool        s_bothButtonsHeld       = false;
#if ENABLE_SERIAL_DEBUG
static bool        s_verboseDebug     = (ENABLE_USB_VERBOSE_DEBUG != 0);
static bool        s_prevLeft         = false;
static bool        s_prevRight        = false;
static bool        s_prevMiddle       = false;
static bool        s_prevBtn4         = false;
static bool        s_prevBtn5         = false;
#endif

#if ENABLE_STATUS_LED
// Non-blocking status LED blink state machine
static uint8_t  s_ledBlinksRemaining = 0;
static bool     s_ledState           = false;
static uint32_t s_ledLastToggleMs    = 0;
static constexpr uint16_t LED_BLINK_INTERVAL_MS = 80;
#endif

// Non-blocking artificial cursor jiggle state machine (N x Up/Down or Left/Right pairs)
static uint8_t  s_wiggleLegIndex     = 0;
static uint8_t  s_wiggleTotalLegs    = 0;     // pairs * 2 * WIGGLE_STEPS_PER_DIRECTION
static bool     s_wiggleVertical     = true;  // true = Up/Down, false = Left/Right
static bool     s_wiggleWaitingPause = false; // true once current leg's pulses have finished
static uint32_t s_wigglePauseStartUs = 0;     // micros() timestamp when current leg finished

static void startStatusLedBlink(uint8_t count) {
#if ENABLE_STATUS_LED
    s_ledBlinksRemaining = count * 2; // ON + OFF transitions
    s_ledState = true;
    s_ledLastToggleMs = millis();
    digitalWrite(PIN_STATUS_LED, HIGH);
#else
    (void)count;
#endif
}

static void updateStatusLedBlink() {
#if ENABLE_STATUS_LED
    if (s_ledBlinksRemaining == 0) {
        return;
    }
    uint32_t nowMs = millis();
    if ((nowMs - s_ledLastToggleMs) >= LED_BLINK_INTERVAL_MS) {
        s_ledLastToggleMs = nowMs;
        s_ledBlinksRemaining--;
        if (s_ledBlinksRemaining == 0) {
            // Restore LED to steady ON if mouse is connected
            digitalWrite(PIN_STATUS_LED, ps2Mouse.isConnected() ? HIGH : LOW);
        } else {
            s_ledState = !s_ledState;
            digitalWrite(PIN_STATUS_LED, s_ledState ? HIGH : LOW);
        }
    }
#endif
}

static void startCursorJiggle(uint8_t pairs, bool vertical) {
    if (pairs == 0) {
        return;
    }
    amigaMouse.clearPendingSteps();
    s_wiggleTotalLegs    = pairs * 2 * WIGGLE_STEPS_PER_DIRECTION;
    s_wiggleLegIndex     = 0;
    s_wiggleVertical     = vertical;
    s_wiggleWaitingPause = false;

    // First step: UP (dy = -WIGGLE_STEP_DELTA) if vertical, or LEFT (dx = -WIGGLE_STEP_DELTA) if horizontal
    if (vertical) {
        amigaMouse.queueRawSteps(0, -WIGGLE_STEP_DELTA);
    } else {
        amigaMouse.queueRawSteps(-WIGGLE_STEP_DELTA, 0);
    }
#if ENABLE_SERIAL_DEBUG
    if (s_verboseDebug) {
        Serial.printf("[AmigaMouse] Artificial jiggle pair 1/%u step 1/%u %s (%ld steps)\n",
                      pairs,
                      static_cast<unsigned>(WIGGLE_STEPS_PER_DIRECTION),
                      vertical ? "UP" : "LEFT",
                      static_cast<long>(-WIGGLE_STEP_DELTA));
    }
#endif
}

static void updateCursorJiggle() {
    if (s_wiggleLegIndex >= s_wiggleTotalLegs) {
        return;
    }

    // 1. Wait until all quadrature pulses for the current leg have been emitted
    if (amigaMouse.getPendingX() != 0 || amigaMouse.getPendingY() != 0) {
        return;
    }

    // 2. Start the inter-train pause timer the instant the pulse train finishes
    uint32_t nowUs = micros();
    if (!s_wiggleWaitingPause) {
        s_wiggleWaitingPause = true;
        s_wigglePauseStartUs = nowUs;
        return;
    }

    static constexpr uint8_t LEGS_PER_PAIR = 2 * WIGGLE_STEPS_PER_DIRECTION;
    uint8_t legInPair = s_wiggleLegIndex % LEGS_PER_PAIR;
    bool isEndOfPair  = (legInPair == (LEGS_PER_PAIR - 1));

    // Wait WIGGLE_LEG_DURATION_MS after each pulse train within a pair,
    // and WIGGLE_PAIR_PAUSE_MS after the last pulse train of a pair.
    uint32_t pauseUs = (isEndOfPair ? static_cast<uint32_t>(WIGGLE_PAIR_PAUSE_MS)
                                    : static_cast<uint32_t>(WIGGLE_LEG_DURATION_MS)) * 1000UL;

    if ((nowUs - s_wigglePauseStartUs) >= pauseUs) {
        s_wiggleWaitingPause = false;
        s_wiggleLegIndex++;
        if (s_wiggleLegIndex < s_wiggleTotalLegs) {
            uint8_t nextLegInPair = s_wiggleLegIndex % LEGS_PER_PAIR;
            bool firstDirection   = (nextLegInPair < WIGGLE_STEPS_PER_DIRECTION);
            int32_t delta         = firstDirection ? -WIGGLE_STEP_DELTA : +WIGGLE_STEP_DELTA;
            if (s_wiggleVertical) {
                amigaMouse.queueRawSteps(0, delta);
            } else {
                amigaMouse.queueRawSteps(delta, 0);
            }
#if ENABLE_SERIAL_DEBUG
            if (s_verboseDebug) {
                const char *dirName = s_wiggleVertical ? (firstDirection ? "UP" : "DOWN")
                                                       : (firstDirection ? "LEFT" : "RIGHT");
                uint8_t stepInDir   = (nextLegInPair % WIGGLE_STEPS_PER_DIRECTION) + 1;
                Serial.printf("[AmigaMouse] Artificial jiggle pair %u/%u step %u/%u %s (%+ld steps)\n",
                              (s_wiggleLegIndex / LEGS_PER_PAIR) + 1,
                              s_wiggleTotalLegs / LEGS_PER_PAIR,
                              stepInDir,
                              static_cast<unsigned>(WIGGLE_STEPS_PER_DIRECTION),
                              dirName,
                              static_cast<long>(delta));
            }
#endif
        }
    }
}

static void handleSerialCommands() {
#if ENABLE_SERIAL_DEBUG
    while (Serial.available() > 0) {
        char ch = static_cast<char>(Serial.read());
        switch (ch) {
            case '1':
                amigaMouse.setSensitivityPresetIndex(0);
                Serial.println(F("[AmigaMouse] Sensitivity set to #1: 50% (1:2)"));
                startCursorJiggle(1, true);
                startStatusLedBlink(1);
                break;
            case '2':
                amigaMouse.setSensitivityPresetIndex(1);
                Serial.println(F("[AmigaMouse] Sensitivity set to #2: 75% (3:4)"));
                startCursorJiggle(2, true);
                startStatusLedBlink(2);
                break;
            case '3':
                amigaMouse.setSensitivityPresetIndex(2);
                Serial.println(F("[AmigaMouse] Sensitivity set to #3: 100% (1:1)"));
                startCursorJiggle(3, true);
                startStatusLedBlink(3);
                break;
            case '4':
                amigaMouse.setSensitivityPresetIndex(3);
                Serial.println(F("[AmigaMouse] Sensitivity set to #4: 125% (5:4)"));
                startCursorJiggle(4, true);
                startStatusLedBlink(4);
                break;
            case '5':
                amigaMouse.setSensitivityPresetIndex(4);
                Serial.println(F("[AmigaMouse] Sensitivity set to #5: 150% (3:2)"));
                startCursorJiggle(5, true);
                startStatusLedBlink(5);
                break;
            case '6':
                amigaMouse.setSensitivityPresetIndex(5);
                Serial.println(F("[AmigaMouse] Sensitivity set to #6: 200% (2:1)"));
                startCursorJiggle(6, true);
                startStatusLedBlink(6);
                break;
            case 'b':
            case 'B': {
                bool swapped = amigaMouse.toggleLeftRightSwapped();
                Serial.printf("[AmigaMouse] Left/Right buttons swap %s\n", swapped ? "ENABLED" : "DISABLED");
                startCursorJiggle(1, false);
                startStatusLedBlink(1);
                break;
            }
            case 'v':
            case 'V':
            case 'p':
            case 'P':
                s_verboseDebug = !s_verboseDebug;
                Serial.printf("[Debug] USB verbose packet logging %s\n", s_verboseDebug ? "ENABLED" : "DISABLED");
                break;
            case 'r':
            case 'R':
                Serial.println(F("[PS2Mouse] Resetting PS/2 mouse..."));
                if (ps2Mouse.begin()) {
                    Serial.printf("[PS2Mouse] Reset & initialization succeeded. Device ID=0x%02X (%s)\n",
                                  ps2Mouse.getDeviceId(),
                                  ps2Mouse.getDeviceId() == 0x04 ? "IntelliMouse Explorer 4-byte (5-btn + scroll)" :
                                  (ps2Mouse.hasScrollWheel() ? "IntelliMouse 4-byte (scroll wheel/nub)" : "Standard PS/2 3-byte"));
                } else {
                    Serial.println(F("[PS2Mouse] Reset failed (check wiring/power)."));
                }
                break;
            case 's':
            case 'S':
                Serial.printf(
                    "[Status] Connected=%d | DevID=0x%02X | Preset=#%u | SwapLR=%d | Packets=%lu | Errors=%lu | Pending=(%ld,%ld) | Verbose=%d\n",
                    ps2Mouse.isConnected() ? 1 : 0,
                    ps2Mouse.getDeviceId(),
                    amigaMouse.getSensitivityPresetIndex() + 1,
                    amigaMouse.isLeftRightSwapped() ? 1 : 0,
                    static_cast<unsigned long>(ps2Mouse.getPacketCount()),
                    static_cast<unsigned long>(ps2Mouse.getErrorCount()),
                    static_cast<long>(amigaMouse.getPendingX()),
                    static_cast<long>(amigaMouse.getPendingY()),
                    s_verboseDebug ? 1 : 0
                );
                break;
            default:
                break;
        }
    }
#endif
}

void setup() {
    // Use less power, lower clock frequency.
    setCpuFrequencyMhz(ESP32_CPU_FREQ_MHZ);

    // Use less power, disable bluetooth.
    btStop();

    // Always drive status LED LOW at boot so it cannot draw power when disabled
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    // Initialize Amiga 1000 mouse port signals first so lines are clean at boot
    amigaMouse.begin();

#if ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
    uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 500) {
        delay(10);
    }
    Serial.println();
    Serial.println(F("=========================================================="));
    Serial.println(F(" Amiga 1000 PS/2 Mouse Adapter - ESP32-S3 Dev Module"));
    Serial.println(F("=========================================================="));
    Serial.printf(" CPU Freq   : %lu MHz\n", static_cast<unsigned long>(getCpuFrequencyMhz()));
    Serial.printf(" PS/2 Pins  : CLK=GPIO%d, DATA=GPIO%d\n", PIN_PS2_CLK, PIN_PS2_DATA);
    Serial.printf(" Amiga Quad : V=GPIO%d, H=GPIO%d, VQ=GPIO%d, HQ=GPIO%d\n",
                  PIN_AMIGA_V_PULSE, PIN_AMIGA_H_PULSE, PIN_AMIGA_VQ_PULSE, PIN_AMIGA_HQ_PULSE);
    Serial.printf(" Amiga Btns : LMB=GPIO%d, RMB=GPIO%d, MMB=GPIO%d (SwapLR=%s)\n",
                  PIN_AMIGA_LMB, PIN_AMIGA_RMB, PIN_AMIGA_MMB,
                  amigaMouse.isLeftRightSwapped() ? "ON" : "OFF");
    Serial.printf(" USB Verbose: %s (press 'v' in Serial Monitor to toggle)\n",
                  s_verboseDebug ? "ENABLED" : "DISABLED");
#endif

    bool ok = ps2Mouse.begin();
    if (ok) {
#if ENABLE_STATUS_LED
        digitalWrite(PIN_STATUS_LED, HIGH);
#endif
#if ENABLE_SERIAL_DEBUG
        Serial.printf("[PS2Mouse] Initialized! Device ID=0x%02X (%s)\n",
                      ps2Mouse.getDeviceId(),
                      ps2Mouse.getDeviceId() == 0x04 ? "IntelliMouse Explorer 4-byte (5-btn + scroll)" :
                      (ps2Mouse.hasScrollWheel() ? "IntelliMouse 4-byte (scroll wheel/nub)" : "Standard PS/2 3-byte"));
#endif
    } else {
#if ENABLE_SERIAL_DEBUG
        Serial.println(F("[PS2Mouse] Mouse not detected at boot; waiting for hot-plug..."));
#endif
    }
}

void loop() {
    // 1. Poll for complete PS/2 movement & button packets
    PS2MousePacket packet;
    if (ps2Mouse.poll(packet)) {
#if ENABLE_SERIAL_DEBUG
        if (s_verboseDebug) {
            bool btnChanged = (packet.leftButton != s_prevLeft) ||
                              (packet.rightButton != s_prevRight) ||
                              (packet.middleButton != s_prevMiddle) ||
                              (packet.button4 != s_prevBtn4) ||
                              (packet.button5 != s_prevBtn5);
            s_prevLeft   = packet.leftButton;
            s_prevRight  = packet.rightButton;
            s_prevMiddle = packet.middleButton;
            s_prevBtn4   = packet.button4;
            s_prevBtn5   = packet.button5;

            Serial.printf("[PS2 #%lu] dx=%4d dy=%4d dz=%3d | PhysBtns: L=%d R=%d M=%d B4=%d B5=%d%s%s | Cfg=%u | AmigaPending=(%4ld,%4ld) | Err=%lu\n",
                          static_cast<unsigned long>(ps2Mouse.getPacketCount()),
                          packet.dx,
                          packet.dy,
                          packet.dz,
                          packet.leftButton ? 1 : 0,
                          packet.rightButton ? 1 : 0,
                          packet.middleButton ? 1 : 0,
                          packet.button4 ? 1 : 0,
                          packet.button5 ? 1 : 0,
                          btnChanged ? " [BTN_CHG]" : "",
                          (packet.dz != 0) ? " [SCROLL]" : "",
                          static_cast<unsigned>(s_configState),
                          static_cast<long>(amigaMouse.getPendingX()),
                          static_cast<long>(amigaMouse.getPendingY()),
                          static_cast<unsigned long>(ps2Mouse.getErrorCount()));
        }
#endif

        if (s_configState == CFG_IDLE) {
            // While an acknowledgment jiggle is active (e.g. triggered via Serial),
            // ignore mouse movement so the full jiggle is clearly seen.
            if (s_wiggleLegIndex >= s_wiggleTotalLegs) {
                amigaMouse.processPs2Packet(packet);
            }

            // Track configuration mode chord: hold both physical Left + Right buttons for 1 second
            bool bothHeld = packet.leftButton && packet.rightButton && !packet.middleButton;
            if (bothHeld) {
                if (abs(packet.dx) > 4 || abs(packet.dy) > 4) {
                    s_chordStartMs = millis();
                } else if (!s_bothButtonsHeld) {
                    s_chordStartMs = millis();
                }
                s_bothButtonsHeld = true;
            } else {
                s_bothButtonsHeld = false;
                s_chordStartMs = 0;
            }
        } else if (s_configState == CFG_WAIT_RELEASE_ALL) {
            // Ignore dx/dy and wait for all physical buttons to be released
            if (!packet.leftButton && !packet.rightButton && !packet.middleButton) {
                s_cfgFirstClick         = CFG_CLICK_NONE;
                s_cfgFirstClickReleased = false;
                s_cfgPendingAction      = CFG_CLICK_NONE;
                s_cfgClickReleased      = false;
                s_configState           = CFG_WAIT_TWO_CLICKS;
#if ENABLE_SERIAL_DEBUG
                Serial.println(F("[ConfigMode] Both buttons released. Waiting for two clicks (L+L=-Sens, R+R=+Sens, L+R or R+L=Toggle Swap)..."));
#endif
            }
        } else if (s_configState == CFG_WAIT_TWO_CLICKS) {
            // Ignore dx/dy; latch physical two-click sequence (even if the entry jiggle
            // or its post-jiggle pause is still finishing):
            //   Left + Left   -> Decrease sensitivity (CFG_CLICK_LEFT)
            //   Right + Right -> Increase sensitivity (CFG_CLICK_RIGHT)
            //   Left + Right or Right + Left -> Toggle L/R swap (CFG_CLICK_SWAP)
            if (s_cfgFirstClick == CFG_CLICK_NONE) {
                if (packet.leftButton && !packet.rightButton && !packet.middleButton) {
                    s_cfgFirstClick         = CFG_CLICK_LEFT;
                    s_cfgFirstClickReleased = false;
#if ENABLE_SERIAL_DEBUG
                    Serial.println(F("[ConfigMode] 1st click: LEFT (waiting for release + 2nd click)..."));
#endif
                } else if (packet.rightButton && !packet.leftButton && !packet.middleButton) {
                    s_cfgFirstClick         = CFG_CLICK_RIGHT;
                    s_cfgFirstClickReleased = false;
#if ENABLE_SERIAL_DEBUG
                    Serial.println(F("[ConfigMode] 1st click: RIGHT (waiting for release + 2nd click)..."));
#endif
                }
            } else if (!s_cfgFirstClickReleased) {
                if (!packet.leftButton && !packet.rightButton && !packet.middleButton) {
                    s_cfgFirstClickReleased = true;
                }
            } else if (s_cfgPendingAction == CFG_CLICK_NONE) {
                ConfigClick secondClick = CFG_CLICK_NONE;
                if (packet.leftButton && !packet.rightButton && !packet.middleButton) {
                    secondClick = CFG_CLICK_LEFT;
                } else if (packet.rightButton && !packet.leftButton && !packet.middleButton) {
                    secondClick = CFG_CLICK_RIGHT;
                }

                if (secondClick != CFG_CLICK_NONE) {
                    if (s_cfgFirstClick == CFG_CLICK_LEFT && secondClick == CFG_CLICK_LEFT) {
                        s_cfgPendingAction = CFG_CLICK_LEFT;  // 2x Left = decrease sensitivity
                    } else if (s_cfgFirstClick == CFG_CLICK_RIGHT && secondClick == CFG_CLICK_RIGHT) {
                        s_cfgPendingAction = CFG_CLICK_RIGHT; // 2x Right = increase sensitivity
                    } else {
                        s_cfgPendingAction = CFG_CLICK_SWAP;  // L->R or R->L = toggle L/R swap
                    }
                    s_cfgClickReleased = false;
                }
            } else if (!packet.leftButton && !packet.rightButton && !packet.middleButton) {
                // User already released the 2nd clicked button while entry jiggle was finishing
                s_cfgClickReleased = true;
            }
        } else if (s_configState == CFG_WAIT_ACK_COMPLETE) {
            // Keep mouse movement disabled while acknowledgment jiggle plays;
            // record when the clicked button has been released.
            if (!packet.leftButton && !packet.rightButton && !packet.middleButton) {
                s_cfgClickReleased = true;
            }
        }
    }

    // Check 1-second 2-button hold timer continuously in loop() because a PS/2
    // mouse in Stream Mode sends no packets while stationary with buttons held.
    if (s_configState == CFG_IDLE && s_bothButtonsHeld && (millis() - s_chordStartMs) >= 1000) {
        s_configState           = CFG_WAIT_RELEASE_ALL;
        s_cfgFirstClick         = CFG_CLICK_NONE;
        s_cfgFirstClickReleased = false;
        s_cfgPendingAction      = CFG_CLICK_NONE;
        s_cfgClickReleased      = false;
        s_bothButtonsHeld       = false;
        s_chordStartMs          = 0;

        // Release Amiga buttons and clear any queued movement while in config mode
        amigaMouse.setButtons(false, false, false);
        amigaMouse.clearPendingSteps();

#if ENABLE_SERIAL_DEBUG
        Serial.println(F("[ConfigMode] Entered configuration mode! Release both buttons, then click twice (L+L, R+R, or L+R/R+L)."));
#endif
        // Acknowledge entering configuration mode with 1 Up/Down jiggle
        startCursorJiggle(1, true);
        startStatusLedBlink(1);
    }

    // Advance non-blocking feedback animations
    updateCursorJiggle();
    updateStatusLedBlink();

    // Execute the latched config action once the entry jiggle (including its pause) has finished
    if (s_configState == CFG_WAIT_TWO_CLICKS &&
        s_cfgPendingAction != CFG_CLICK_NONE &&
        s_wiggleLegIndex >= s_wiggleTotalLegs &&
        amigaMouse.getPendingX() == 0 &&
        amigaMouse.getPendingY() == 0) {
        if (s_cfgPendingAction == CFG_CLICK_LEFT) {
            amigaMouse.decreaseSensitivityPreset();
            uint8_t presetNum = amigaMouse.getSensitivityPresetIndex() + 1;
#if ENABLE_SERIAL_DEBUG
            Serial.printf("[ConfigMode] 2x LEFT click -> Sensitivity preset #%u\n", presetNum);
#endif
            startCursorJiggle(presetNum, true); // Acknowledge with presetNum Up/Down pairs
            startStatusLedBlink(presetNum);
        } else if (s_cfgPendingAction == CFG_CLICK_RIGHT) {
            amigaMouse.increaseSensitivityPreset();
            uint8_t presetNum = amigaMouse.getSensitivityPresetIndex() + 1;
#if ENABLE_SERIAL_DEBUG
            Serial.printf("[ConfigMode] 2x RIGHT click -> Sensitivity preset #%u\n", presetNum);
#endif
            startCursorJiggle(presetNum, true); // Acknowledge with presetNum Up/Down pairs
            startStatusLedBlink(presetNum);
        } else if (s_cfgPendingAction == CFG_CLICK_SWAP) {
            bool swapped = amigaMouse.toggleLeftRightSwapped();
#if ENABLE_SERIAL_DEBUG
            Serial.printf("[ConfigMode] LEFT+RIGHT click -> Swap Left/Right buttons = %s\n",
                          swapped ? "ENABLED" : "DISABLED");
#endif
            startCursorJiggle(1, false); // Acknowledge with 1 Left/Right wiggle
            startStatusLedBlink(1);
        }
        s_cfgPendingAction = CFG_CLICK_NONE;
        s_configState      = CFG_WAIT_ACK_COMPLETE;
    }

    // Exit configuration mode only after BOTH the 2nd clicked button is released
    // AND the entire acknowledgment jiggle has finished playing on screen.
    if (s_configState == CFG_WAIT_ACK_COMPLETE &&
        s_cfgClickReleased &&
        s_wiggleLegIndex >= s_wiggleTotalLegs &&
        amigaMouse.getPendingX() == 0 &&
        amigaMouse.getPendingY() == 0) {
        s_configState = CFG_IDLE;
        s_bothButtonsHeld = false;
        s_chordStartMs = 0;
#if ENABLE_SERIAL_DEBUG
        Serial.println(F("[ConfigMode] Acknowledgment complete. Exited configuration mode."));
#endif
    }

    // 2. Step the Amiga quadrature state machine at microsecond intervals
    amigaMouse.update();

    // 3. Run PS/2 watchdog & hot-plug detection
    ps2Mouse.update();

    // 4. Process optional USB serial diagnostic commands
    handleSerialCommands();
}
