#include "amiga_mouse.h"

// ============================================================================
// Amiga 2-Bit Gray Code Quadrature Phase Table
// ============================================================================
// Standard Amiga mouse quadrature sequence for Denise (8362) JOY0DAT:
//   Phase 0: Pulse = 0, QPulse = 0
//   Phase 1: Pulse = 1, QPulse = 0
//   Phase 2: Pulse = 1, QPulse = 1
//   Phase 3: Pulse = 0, QPulse = 1
//
// Stepping forward (0 -> 1 -> 2 -> 3 -> 0):
//   Horizontal (H leads HQ): Moves Amiga pointer RIGHT (+X)
//   Vertical   (V leads VQ): Moves Amiga pointer DOWN  (+Y)
//
// Stepping backward (0 -> 3 -> 2 -> 1 -> 0):
//   Horizontal (HQ leads H): Moves Amiga pointer LEFT  (-X)
//   Vertical   (VQ leads V): Moves Amiga pointer UP    (-Y)
static constexpr uint8_t QUAD_PULSE_TABLE[4]  = { LOW, HIGH, HIGH, LOW };
static constexpr uint8_t QUAD_QPULSE_TABLE[4] = { LOW, LOW,  HIGH, HIGH };

AmigaMouse::AmigaMouse(uint8_t vPin,
                       uint8_t hPin,
                       uint8_t vqPin,
                       uint8_t hqPin,
                       uint8_t lmbPin,
                       uint8_t rmbPin,
                       uint8_t mmbPin)
    : _vPin(vPin),
      _hPin(hPin),
      _vqPin(vqPin),
      _hqPin(hqPin),
      _lmbPin(lmbPin),
      _rmbPin(rmbPin),
      _mmbPin(mmbPin),
      _phaseX(0),
      _phaseY(0),
      _accumX(0),
      _accumY(0),
      _lastStepXUs(0),
      _lastStepYUs(0),
      _leftPressed(false),
      _rightPressed(false),
      _middlePressed(false),
      _swapLeftRight(AMIGA_DEFAULT_SWAP_BUTTONS != 0) {}

void AmigaMouse::begin() {
    // Configure quadrature lines as push-pull outputs driving TTL levels
    pinMode(_vPin, OUTPUT);
    pinMode(_hPin, OUTPUT);
    pinMode(_vqPin, OUTPUT);
    pinMode(_hqPin, OUTPUT);

    _phaseX = 0;
    _phaseY = 0;
    writeQuadratureX();
    writeQuadratureY();

    // Configure button lines as open-drain (released = high-Z input, pressed = LOW)
    // Amiga 1000 provides internal pull-ups on LMB (CIA-A) and RMB/MMB (Paula POTGO).
    writeOpenDrainButton(_lmbPin, false);
    writeOpenDrainButton(_rmbPin, false);
    writeOpenDrainButton(_mmbPin, false);

    uint32_t nowUs = micros();
    _lastStepXUs = nowUs;
    _lastStepYUs = nowUs;
}

void AmigaMouse::writeQuadratureX() {
    digitalWrite(_hPin,  QUAD_PULSE_TABLE[_phaseX & 0x03]);
    digitalWrite(_hqPin, QUAD_QPULSE_TABLE[_phaseX & 0x03]);
}

void AmigaMouse::writeQuadratureY() {
    digitalWrite(_vPin,  QUAD_PULSE_TABLE[_phaseY & 0x03]);
    digitalWrite(_vqPin, QUAD_QPULSE_TABLE[_phaseY & 0x03]);
}

void AmigaMouse::writeOpenDrainButton(uint8_t pin, bool pressed) {
    if (pressed) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    } else {
        // High-impedance when released so the Amiga's pull-up holds the line HIGH
        pinMode(pin, INPUT);
    }
}

void AmigaMouse::setButtons(bool left, bool right, bool middle) {
    if (left != _leftPressed) {
        _leftPressed = left;
        writeOpenDrainButton(_lmbPin, left);
    }
    if (right != _rightPressed) {
        _rightPressed = right;
        writeOpenDrainButton(_rmbPin, right);
    }
    if (middle != _middlePressed) {
        _middlePressed = middle;
        writeOpenDrainButton(_mmbPin, middle);
    }
}

void AmigaMouse::processPs2Packet(const PS2MousePacket &packet) {
    // Update Left, Right, and Middle mouse buttons immediately (applying L/R swap if enabled)
    bool outLeft  = _swapLeftRight ? packet.rightButton : packet.leftButton;
    bool outRight = _swapLeftRight ? packet.leftButton  : packet.rightButton;
    setButtons(outLeft, outRight, packet.middleButton);

    // Convert PS/2 coordinate deltas 1:1 to Amiga screen deltas:
    //   PS/2:  +X = Right, +Y = Up
    //   Amiga: +X = Right, +Y = Down
    int32_t stepsX = packet.dx;
    int32_t stepsY = -packet.dy;

#if AMIGA_INVERT_X
    stepsX = -stepsX;
#endif
#if AMIGA_INVERT_Y
    stepsY = -stepsY;
#endif

    // Accumulate clamped 1:1 deltas so large flicks don't cause multi-second lag
    _accumX = constrain(_accumX + stepsX, -AMIGA_MAX_ACCUMULATED_DELTA, AMIGA_MAX_ACCUMULATED_DELTA);
    _accumY = constrain(_accumY + stepsY, -AMIGA_MAX_ACCUMULATED_DELTA, AMIGA_MAX_ACCUMULATED_DELTA);
}

void AmigaMouse::queueRawSteps(int32_t stepsX, int32_t stepsY) {
    _accumX = constrain(_accumX + stepsX, -AMIGA_MAX_ACCUMULATED_DELTA, AMIGA_MAX_ACCUMULATED_DELTA);
    _accumY = constrain(_accumY + stepsY, -AMIGA_MAX_ACCUMULATED_DELTA, AMIGA_MAX_ACCUMULATED_DELTA);
}

void AmigaMouse::clearPendingSteps() {
    _accumX = 0;
    _accumY = 0;
}

uint32_t AmigaMouse::computeStepIntervalUs(int32_t pendingSteps) const {
    int32_t mag = abs(pendingSteps);
    if (mag >= 16) {
        return AMIGA_MIN_STEP_INTERVAL_US;
    }
    // Linearly interpolate between MAX_STEP_INTERVAL_US (slow motion) and
    // MIN_STEP_INTERVAL_US (fast motion) based on backlog depth (1..16 steps)
    uint32_t span = AMIGA_MAX_STEP_INTERVAL_US - AMIGA_MIN_STEP_INTERVAL_US;
    return AMIGA_MIN_STEP_INTERVAL_US + ((span * (16 - mag)) / 16);
}

void AmigaMouse::update() {
    uint32_t nowUs = micros();

    // Service Horizontal (X) quadrature transitions
    if (_accumX != 0) {
        uint32_t intervalX = computeStepIntervalUs(_accumX);
        if ((nowUs - _lastStepXUs) >= intervalX) {
            _lastStepXUs = nowUs;
            if (_accumX > 0) {
                _phaseX = (_phaseX + 1) & 0x03;
                _accumX--;
            } else {
                _phaseX = (_phaseX - 1) & 0x03;
                _accumX++;
            }
            writeQuadratureX();
        }
    }

    // Service Vertical (Y) quadrature transitions
    if (_accumY != 0) {
        uint32_t intervalY = computeStepIntervalUs(_accumY);
        if ((nowUs - _lastStepYUs) >= intervalY) {
            _lastStepYUs = nowUs;
            if (_accumY > 0) {
                _phaseY = (_phaseY + 1) & 0x03;
                _accumY--;
            } else {
                _phaseY = (_phaseY - 1) & 0x03;
                _accumY++;
            }
            writeQuadratureY();
        }
    }
}
