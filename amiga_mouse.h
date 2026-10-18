#ifndef AMIGA_MOUSE_H
#define AMIGA_MOUSE_H

#include <Arduino.h>
#include "config.h"
#include "ps2_mouse.h"

class AmigaMouse {
public:
    AmigaMouse(uint8_t vPin,
               uint8_t hPin,
               uint8_t vqPin,
               uint8_t hqPin,
               uint8_t lmbPin,
               uint8_t rmbPin,
               uint8_t mmbPin);

    // Initialize Amiga DB9 port pins to idle states
    void begin();

    // Queue a decoded PS/2 mouse report for translation to Amiga signaling
    void processPs2Packet(const PS2MousePacket &packet);

    // Directly queue unscaled quadrature steps (e.g., for visual cursor feedback)
    void queueRawSteps(int32_t stepsX, int32_t stepsY);

    // Non-blocking microsecond state machine that outputs quadrature transitions
    // at a controlled rate safe for the Amiga 1000 Denise (8362) JOY0DAT counters.
    // Call this as frequently as possible from loop().
    void update();

    // Directly set button states (true = pressed / active LOW on Amiga port)
    void setButtons(bool left, bool right, bool middle);

    // Configure movement scaling factor (numerator / denominator)
    void setScaling(int16_t numerator, int16_t denominator);
    void cycleSensitivityPreset();
    uint8_t getSensitivityPresetIndex() const { return _presetIndex; }

    // Diagnostics accessors
    int32_t getPendingX() const { return _accumX; }
    int32_t getPendingY() const { return _accumY; }
    bool isMoving() const { return (_accumX != 0) || (_accumY != 0); }

private:
    uint8_t _vPin;
    uint8_t _hPin;
    uint8_t _vqPin;
    uint8_t _hqPin;
    uint8_t _lmbPin;
    uint8_t _rmbPin;
    uint8_t _mmbPin;

    // Current 2-bit quadrature phase index (0..3) for each axis
    uint8_t _phaseX;
    uint8_t _phaseY;

    // Pending quadrature steps to emit
    int32_t _accumX;
    int32_t _accumY;

    // Fractional scaling remainders so slow sub-step movements are preserved
    int32_t _remX;
    int32_t _remY;
    int16_t _scaleNum;
    int16_t _scaleDen;
    uint8_t _presetIndex;

    // Timestamps (microseconds) of last quadrature transition per axis
    uint32_t _lastStepXUs;
    uint32_t _lastStepYUs;

    // Current button state cache
    bool _leftPressed;
    bool _rightPressed;
    bool _middlePressed;

    // Output helpers
    void writeQuadratureX();
    void writeQuadratureY();
    void writeOpenDrainButton(uint8_t pin, bool pressed);
    uint32_t computeStepIntervalUs(int32_t pendingSteps) const;
};

#endif // AMIGA_MOUSE_H
