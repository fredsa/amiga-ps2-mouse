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
static uint32_t s_chordStartMs = 0;
static bool     s_chordTriggered = false;

static void blinkStatusLed(uint8_t count, uint16_t onMs = 80, uint16_t offMs = 80) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        uint32_t t0 = millis();
        while ((millis() - t0) < onMs) {
            amigaMouse.update();
        }
        digitalWrite(PIN_STATUS_LED, LOW);
        t0 = millis();
        while ((millis() - t0) < offMs) {
            amigaMouse.update();
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
                Serial.println(F("[AmigaMouse] Sensitivity set to 50% (1:2)"));
                break;
            case '2':
                amigaMouse.setScaling(3, 4);
                Serial.println(F("[AmigaMouse] Sensitivity set to 75% (3:4)"));
                break;
            case '3':
                amigaMouse.setScaling(1, 1);
                Serial.println(F("[AmigaMouse] Sensitivity set to 100% (1:1)"));
                break;
            case '4':
                amigaMouse.setScaling(3, 2);
                Serial.println(F("[AmigaMouse] Sensitivity set to 150% (3:2)"));
                break;
            case '5':
                amigaMouse.setScaling(2, 1);
                Serial.println(F("[AmigaMouse] Sensitivity set to 200% (2:1)"));
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
                    "[Status] Connected=%d | DevID=0x%02X | Wheel=%d | Packets=%lu | Errors=%lu | Pending=(%ld,%ld)\n",
                    ps2Mouse.isConnected() ? 1 : 0,
                    ps2Mouse.getDeviceId(),
                    ps2Mouse.hasScrollWheel() ? 1 : 0,
                    static_cast<unsigned long>(ps2Mouse.getPacketCount()),
                    static_cast<unsigned long>(ps2Mouse.getErrorCount()),
                    static_cast<long>(amigaMouse.getPendingX()),
                    static_cast<long>(amigaMouse.getPendingY())
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
        amigaMouse.processPs2Packet(packet);

        // Check for 3-button sensitivity preset chord (LMB + RMB + MMB held for 1 second)
        if (packet.leftButton && packet.rightButton && packet.middleButton &&
            packet.dx == 0 && packet.dy == 0) {
            if (s_chordStartMs == 0) {
                s_chordStartMs = millis();
                s_chordTriggered = false;
            } else if (!s_chordTriggered && (millis() - s_chordStartMs) >= 1000) {
                s_chordTriggered = true;
                amigaMouse.cycleSensitivityPreset();
                uint8_t presetNum = amigaMouse.getSensitivityPresetIndex() + 1;
#if ENABLE_SERIAL_DEBUG
                Serial.printf("[AmigaMouse] Sensitivity preset cycled to #%u\n", presetNum);
#endif
                blinkStatusLed(presetNum);
            }
        } else {
            s_chordStartMs = 0;
            s_chordTriggered = false;
        }
    }

    // 2. Step the Amiga quadrature state machine at microsecond intervals
    amigaMouse.update();

    // 3. Run PS/2 watchdog & hot-plug detection
    ps2Mouse.update();

    // 4. Process optional USB serial diagnostic commands
    handleSerialCommands();
}
