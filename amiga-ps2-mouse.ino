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

// Button chord detection for on-the-fly DPI / sensitivity switching
static uint32_t s_chordStartMs   = 0;
static bool     s_chordTriggered = false;
static bool     s_allButtonsHeld = false;
static bool     s_verboseDebug   = (ENABLE_USB_VERBOSE_DEBUG != 0);
static bool     s_prevLeft       = false;
static bool     s_prevRight      = false;
static bool     s_prevMiddle     = false;

// Non-blocking status LED blink state machine
static uint8_t  s_ledBlinksRemaining = 0;
static bool     s_ledState           = false;
static uint32_t s_ledLastToggleMs    = 0;
static constexpr uint16_t LED_BLINK_INTERVAL_MS = 80;

// Non-blocking artificial cursor wiggle state machine (N x Left+Right pairs)
static uint8_t  s_wiggleLegIndex   = 0;
static uint8_t  s_wiggleTotalLegs  = 0; // 2 * presetNum (Left + Right per pair)
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

static void startCursorWiggle(uint8_t pairs) {
    if (pairs == 0) {
        return;
    }
    s_wiggleTotalLegs = pairs * 2;
    s_wiggleLegIndex = 0;
    s_wiggleLegStartMs = millis();

    amigaMouse.queueRawSteps(-WIGGLE_STEP_DELTA, 0);
#if ENABLE_SERIAL_DEBUG
    if (s_verboseDebug) {
        Serial.printf("[AmigaMouse] Artificial wiggle pair 1/%u LEFT (%ld steps)\n",
                      pairs,
                      static_cast<long>(-WIGGLE_STEP_DELTA));
    }
#endif
}

static void updateCursorWiggle() {
    if (s_wiggleLegIndex >= s_wiggleTotalLegs) {
        return;
    }
    // Wait WIGGLE_LEG_DURATION_MS (>= 1 VBlank) after Left leg,
    // and WIGGLE_LEG_DURATION_MS + WIGGLE_PAIR_PAUSE_MS after Right leg before next pair.
    bool isRightLeg = ((s_wiggleLegIndex & 0x01) != 0);
    uint32_t waitMs = isRightLeg ? (WIGGLE_LEG_DURATION_MS + WIGGLE_PAIR_PAUSE_MS)
                                 : WIGGLE_LEG_DURATION_MS;

    uint32_t nowMs = millis();
    if ((nowMs - s_wiggleLegStartMs) >= waitMs && amigaMouse.getPendingX() == 0) {
        s_wiggleLegIndex++;
        if (s_wiggleLegIndex < s_wiggleTotalLegs) {
            int32_t dx = ((s_wiggleLegIndex & 0x01) == 0) ? -WIGGLE_STEP_DELTA : +WIGGLE_STEP_DELTA;
            s_wiggleLegStartMs = nowMs;
            amigaMouse.queueRawSteps(dx, 0);
#if ENABLE_SERIAL_DEBUG
            if (s_verboseDebug) {
                Serial.printf("[AmigaMouse] Artificial wiggle pair %u/%u %s (%+ld steps)\n",
                              (s_wiggleLegIndex / 2) + 1,
                              s_wiggleTotalLegs / 2,
                              (dx < 0) ? "LEFT" : "RIGHT",
                              static_cast<long>(dx));
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
                amigaMouse.setScaling(1, 2);
                Serial.println(F("[AmigaMouse] Sensitivity set to #1: 50% (1:2)"));
                startCursorWiggle(1);
                startStatusLedBlink(1);
                break;
            case '2':
                amigaMouse.setScaling(3, 4);
                Serial.println(F("[AmigaMouse] Sensitivity set to #2: 75% (3:4)"));
                startCursorWiggle(2);
                startStatusLedBlink(2);
                break;
            case '3':
                amigaMouse.setScaling(1, 1);
                Serial.println(F("[AmigaMouse] Sensitivity set to #3: 100% (1:1)"));
                startCursorWiggle(3);
                startStatusLedBlink(3);
                break;
            case '4':
                amigaMouse.setScaling(3, 2);
                Serial.println(F("[AmigaMouse] Sensitivity set to #4: 150% (3:2)"));
                startCursorWiggle(4);
                startStatusLedBlink(4);
                break;
            case '5':
                amigaMouse.setScaling(2, 1);
                Serial.println(F("[AmigaMouse] Sensitivity set to #5: 200% (2:1)"));
                startCursorWiggle(5);
                startStatusLedBlink(5);
                break;
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
                    "[Status] Connected=%d | DevID=0x%02X | Wheel=%d | Packets=%lu | Errors=%lu | Pending=(%ld,%ld) | Verbose=%d\n",
                    ps2Mouse.isConnected() ? 1 : 0,
                    ps2Mouse.getDeviceId(),
                    ps2Mouse.hasScrollWheel() ? 1 : 0,
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
    Serial.printf(" Amiga Btns : LMB=GPIO%d, RMB=GPIO%d, MMB=GPIO%d\n",
                  PIN_AMIGA_LMB, PIN_AMIGA_RMB, PIN_AMIGA_MMB);
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
        // If user moves the mouse while the artificial wiggle is still playing,
        // cancel the remaining wiggle legs immediately so real movement is crisp.
        if (s_wiggleLegIndex < s_wiggleTotalLegs && (abs(packet.dx) > 2 || abs(packet.dy) > 2)) {
            s_wiggleLegIndex = s_wiggleTotalLegs;
        }

        amigaMouse.processPs2Packet(packet);

#if ENABLE_SERIAL_DEBUG
        if (s_verboseDebug) {
            bool btnChanged = (packet.leftButton != s_prevLeft) ||
                              (packet.rightButton != s_prevRight) ||
                              (packet.middleButton != s_prevMiddle);
            s_prevLeft   = packet.leftButton;
            s_prevRight  = packet.rightButton;
            s_prevMiddle = packet.middleButton;

            Serial.printf("[PS2 #%lu] dx=%4d dy=%4d | Btns: L=%d R=%d M=%d%s | AmigaPending=(%4ld,%4ld) | Err=%lu\n",
                          static_cast<unsigned long>(ps2Mouse.getPacketCount()),
                          packet.dx,
                          packet.dy,
                          packet.leftButton ? 1 : 0,
                          packet.rightButton ? 1 : 0,
                          packet.middleButton ? 1 : 0,
                          btnChanged ? " [BTN_CHG]" : "",
                          static_cast<long>(amigaMouse.getPendingX()),
                          static_cast<long>(amigaMouse.getPendingY()),
                          static_cast<unsigned long>(ps2Mouse.getErrorCount()));
        }
#endif

        // Track 3-button sensitivity preset chord state (LMB + RMB + MMB)
        bool allThree = packet.leftButton && packet.rightButton && packet.middleButton;
        if (allThree) {
            // Reset hold timer if mouse is actively dragged (> 4 counts) while holding buttons
            if (abs(packet.dx) > 4 || abs(packet.dy) > 4) {
                s_chordStartMs = millis();
            } else if (!s_allButtonsHeld) {
                s_chordStartMs = millis();
                s_chordTriggered = false;
            }
            s_allButtonsHeld = true;
        } else {
            s_allButtonsHeld = false;
            s_chordStartMs = 0;
            s_chordTriggered = false;
        }
    }

    // Check 1-second 3-button hold timer continuously in loop() because a PS/2
    // mouse in Stream Mode sends no packets while stationary with buttons held.
    if (s_allButtonsHeld && !s_chordTriggered && (millis() - s_chordStartMs) >= 1000) {
        s_chordTriggered = true;
        amigaMouse.cycleSensitivityPreset();
        uint8_t presetNum = amigaMouse.getSensitivityPresetIndex() + 1;
#if ENABLE_SERIAL_DEBUG
        Serial.printf("[AmigaMouse] Sensitivity preset cycled to #%u\n", presetNum);
#endif
        startCursorWiggle(presetNum);
        startStatusLedBlink(presetNum);
    }

    // Advance non-blocking feedback animations
    updateCursorWiggle();
    updateStatusLedBlink();

    // 2. Step the Amiga quadrature state machine at microsecond intervals
    amigaMouse.update();

    // 3. Run PS/2 watchdog & hot-plug detection
    ps2Mouse.update();

    // 4. Process optional USB serial diagnostic commands
    handleSerialCommands();
}
