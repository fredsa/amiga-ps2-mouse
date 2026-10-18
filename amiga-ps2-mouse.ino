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

// Configuration mode state machine triggered by holding LMB + RMB + MMB for 1 second
enum ConfigState : uint8_t {
    CFG_IDLE = 0,
    CFG_WAIT_RELEASE_ALL,   // Waiting for all 3 buttons to be released after entering config mode
    CFG_WAIT_SINGLE_CLICK,  // Waiting for Left (decrease), Right (increase), or Middle (swap L/R) click
    CFG_WAIT_ACK_COMPLETE   // Waiting for clicked button release AND full acknowledgment jiggle completion
};

static ConfigState s_configState      = CFG_IDLE;
static bool        s_cfgAllReleased   = false;
static bool        s_cfgClickReleased = false;
static uint32_t    s_chordStartMs     = 0;
static bool        s_allButtonsHeld   = false;
static bool        s_verboseDebug     = (ENABLE_USB_VERBOSE_DEBUG != 0);
static bool        s_prevLeft         = false;
static bool        s_prevRight        = false;
static bool        s_prevMiddle       = false;

// Non-blocking status LED blink state machine
static uint8_t  s_ledBlinksRemaining = 0;
static bool     s_ledState           = false;
static uint32_t s_ledLastToggleMs    = 0;
static constexpr uint16_t LED_BLINK_INTERVAL_MS = 80;

// Non-blocking artificial cursor jiggle state machine (N x Up/Down or Left/Right pairs)
static uint8_t  s_wiggleLegIndex   = 0;
static uint8_t  s_wiggleTotalLegs  = 0; // pairs * 2 * WIGGLE_STEPS_PER_DIRECTION
static bool     s_wiggleVertical   = true; // true = Up/Down, false = Left/Right
static uint32_t s_wiggleLegStartMs = 0;

static void startStatusLedBlink(uint8_t count) {
    s_ledBlinksRemaining = count * 2; // ON + OFF transitions
    s_ledState = true;
    s_ledLastToggleMs = millis();
    digitalWrite(PIN_STATUS_LED, HIGH);
}

static void updateStatusLedBlink() {
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
}

static void startCursorJiggle(uint8_t pairs, bool vertical) {
    if (pairs == 0) {
        return;
    }
    amigaMouse.clearPendingSteps();
    s_wiggleTotalLegs  = pairs * 2 * WIGGLE_STEPS_PER_DIRECTION;
    s_wiggleLegIndex   = 0;
    s_wiggleVertical   = vertical;
    s_wiggleLegStartMs = millis();

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
    static constexpr uint8_t LEGS_PER_PAIR = 2 * WIGGLE_STEPS_PER_DIRECTION;
    uint8_t legInPair = s_wiggleLegIndex % LEGS_PER_PAIR;
    bool isEndOfPair  = (legInPair == (LEGS_PER_PAIR - 1));

    // Wait WIGGLE_LEG_DURATION_MS (>= 1 VBlank) after each step within a pair,
    // and WIGGLE_LEG_DURATION_MS + WIGGLE_PAIR_PAUSE_MS after the last step of a pair.
    uint32_t waitMs = isEndOfPair ? (WIGGLE_LEG_DURATION_MS + WIGGLE_PAIR_PAUSE_MS)
                                  : WIGGLE_LEG_DURATION_MS;

    uint32_t nowMs = millis();
    if ((nowMs - s_wiggleLegStartMs) >= waitMs &&
        amigaMouse.getPendingX() == 0 &&
        amigaMouse.getPendingY() == 0) {
        s_wiggleLegIndex++;
        if (s_wiggleLegIndex < s_wiggleTotalLegs) {
            uint8_t nextLegInPair = s_wiggleLegIndex % LEGS_PER_PAIR;
            bool firstDirection   = (nextLegInPair < WIGGLE_STEPS_PER_DIRECTION);
            int32_t delta         = firstDirection ? -WIGGLE_STEP_DELTA : +WIGGLE_STEP_DELTA;
            s_wiggleLegStartMs    = nowMs;
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
                    Serial.println(F("[PS2Mouse] Reset & initialization succeeded."));
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
        digitalWrite(PIN_STATUS_LED, HIGH);
#if ENABLE_SERIAL_DEBUG
        Serial.printf("[PS2Mouse] Initialized! Device ID=0x%02X (%s)\n",
                      ps2Mouse.getDeviceId(),
                      ps2Mouse.hasScrollWheel() ? "IntelliMouse 4-byte" : "Standard PS/2 3-byte");
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
                              (packet.middleButton != s_prevMiddle);
            s_prevLeft   = packet.leftButton;
            s_prevRight  = packet.rightButton;
            s_prevMiddle = packet.middleButton;

            Serial.printf("[PS2 #%lu] dx=%4d dy=%4d | PhysBtns: L=%d R=%d M=%d%s | Cfg=%u | AmigaPending=(%4ld,%4ld) | Err=%lu\n",
                          static_cast<unsigned long>(ps2Mouse.getPacketCount()),
                          packet.dx,
                          packet.dy,
                          packet.leftButton ? 1 : 0,
                          packet.rightButton ? 1 : 0,
                          packet.middleButton ? 1 : 0,
                          btnChanged ? " [BTN_CHG]" : "",
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

            // Track 3-button configuration mode chord (physical LMB + RMB + MMB)
            bool allThree = packet.leftButton && packet.rightButton && packet.middleButton;
            if (allThree) {
                if (abs(packet.dx) > 4 || abs(packet.dy) > 4) {
                    s_chordStartMs = millis();
                } else if (!s_allButtonsHeld) {
                    s_chordStartMs = millis();
                }
                s_allButtonsHeld = true;
            } else {
                s_allButtonsHeld = false;
                s_chordStartMs = 0;
            }
        } else if (s_configState == CFG_WAIT_RELEASE_ALL) {
            // Ignore dx/dy and track whether all 3 physical buttons are released
            s_cfgAllReleased = (!packet.leftButton && !packet.rightButton && !packet.middleButton);
        } else if (s_configState == CFG_WAIT_SINGLE_CLICK) {
            // Ignore dx/dy; check physical buttons directly (independent of swap mode)
            if (packet.leftButton && !packet.rightButton && !packet.middleButton) {
                amigaMouse.decreaseSensitivityPreset();
                uint8_t presetNum = amigaMouse.getSensitivityPresetIndex() + 1;
#if ENABLE_SERIAL_DEBUG
                Serial.printf("[ConfigMode] Physical LEFT click -> Sensitivity preset #%u\n", presetNum);
#endif
                startCursorJiggle(presetNum, true); // Acknowledge with presetNum Up/Down pairs
                startStatusLedBlink(presetNum);
                s_cfgClickReleased = false;
                s_configState = CFG_WAIT_ACK_COMPLETE;
            } else if (packet.rightButton && !packet.leftButton && !packet.middleButton) {
                amigaMouse.increaseSensitivityPreset();
                uint8_t presetNum = amigaMouse.getSensitivityPresetIndex() + 1;
#if ENABLE_SERIAL_DEBUG
                Serial.printf("[ConfigMode] Physical RIGHT click -> Sensitivity preset #%u\n", presetNum);
#endif
                startCursorJiggle(presetNum, true); // Acknowledge with presetNum Up/Down pairs
                startStatusLedBlink(presetNum);
                s_cfgClickReleased = false;
                s_configState = CFG_WAIT_ACK_COMPLETE;
            } else if (packet.middleButton && !packet.leftButton && !packet.rightButton) {
                bool swapped = amigaMouse.toggleLeftRightSwapped();
#if ENABLE_SERIAL_DEBUG
                Serial.printf("[ConfigMode] Physical MIDDLE click -> Swap Left/Right buttons = %s\n",
                              swapped ? "ENABLED" : "DISABLED");
#endif
                startCursorJiggle(1, false); // Acknowledge with 1 Left/Right wiggle
                startStatusLedBlink(1);
                s_cfgClickReleased = false;
                s_configState = CFG_WAIT_ACK_COMPLETE;
            }
        } else if (s_configState == CFG_WAIT_ACK_COMPLETE) {
            // Keep mouse movement disabled while acknowledgment jiggle plays;
            // record when the clicked button has been released.
            if (!packet.leftButton && !packet.rightButton && !packet.middleButton) {
                s_cfgClickReleased = true;
            }
        }
    }

    // Check 1-second 3-button hold timer continuously in loop() because a PS/2
    // mouse in Stream Mode sends no packets while stationary with buttons held.
    if (s_configState == CFG_IDLE && s_allButtonsHeld && (millis() - s_chordStartMs) >= 1000) {
        s_configState = CFG_WAIT_RELEASE_ALL;
        s_cfgAllReleased = false;
        s_allButtonsHeld = false;
        s_chordStartMs = 0;

        // Release Amiga buttons and clear any queued movement while in config mode
        amigaMouse.setButtons(false, false, false);
        amigaMouse.clearPendingSteps();

#if ENABLE_SERIAL_DEBUG
        Serial.println(F("[ConfigMode] Entered configuration mode! Release all 3 buttons, then click Left/Right/Middle."));
#endif
        // Acknowledge entering configuration mode with 1 Up/Down jiggle
        startCursorJiggle(1, true);
        startStatusLedBlink(1);
    }

    // Advance non-blocking feedback animations
    updateCursorJiggle();
    updateStatusLedBlink();

    // Advance from CFG_WAIT_RELEASE_ALL to CFG_WAIT_SINGLE_CLICK only after BOTH
    // all 3 buttons have been released AND the entry jiggle has finished playing.
    if (s_configState == CFG_WAIT_RELEASE_ALL &&
        s_cfgAllReleased &&
        s_wiggleLegIndex >= s_wiggleTotalLegs &&
        amigaMouse.getPendingX() == 0 &&
        amigaMouse.getPendingY() == 0) {
        s_configState = CFG_WAIT_SINGLE_CLICK;
#if ENABLE_SERIAL_DEBUG
        Serial.println(F("[ConfigMode] All buttons released & entry jiggle complete. Waiting for single click (Left=-Sens, Right=+Sens, Middle=Toggle Swap)..."));
#endif
    }

    // Exit configuration mode only after BOTH the clicked button is released
    // AND the entire acknowledgment jiggle has finished playing on screen.
    if (s_configState == CFG_WAIT_ACK_COMPLETE &&
        s_cfgClickReleased &&
        s_wiggleLegIndex >= s_wiggleTotalLegs &&
        amigaMouse.getPendingX() == 0 &&
        amigaMouse.getPendingY() == 0) {
        s_configState = CFG_IDLE;
        s_allButtonsHeld = false;
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
