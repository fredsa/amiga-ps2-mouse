#include "ps2_mouse.h"

// Static instance pointer for the GPIO falling-edge ISR trampoline
static PS2Mouse *s_ps2Instance = nullptr;

static void IRAM_ATTR ps2ClockIsrTrampoline() {
    if (s_ps2Instance != nullptr) {
        s_ps2Instance->handleClockInterrupt();
    }
}

// PS/2 Command Constants
static constexpr uint8_t PS2_CMD_SET_SCALING_1_1   = 0xE6;
static constexpr uint8_t PS2_CMD_SET_RESOLUTION    = 0xE8;
static constexpr uint8_t PS2_CMD_GET_DEVICE_ID     = 0xF2;
static constexpr uint8_t PS2_CMD_SET_SAMPLE_RATE   = 0xF3;
static constexpr uint8_t PS2_CMD_ENABLE_REPORTING  = 0xF4;
static constexpr uint8_t PS2_CMD_DISABLE_REPORTING = 0xF5;
static constexpr uint8_t PS2_CMD_SET_DEFAULTS      = 0xF6;
static constexpr uint8_t PS2_CMD_RESET             = 0xFF;

// PS/2 Response Constants
static constexpr uint8_t PS2_RESP_ACK              = 0xFA;
static constexpr uint8_t PS2_RESP_BAT_OK           = 0xAA;
static constexpr uint8_t PS2_RESP_RESEND           = 0xFE;

PS2Mouse::PS2Mouse(uint8_t clkPin, uint8_t dataPin)
    : _clkPin(clkPin),
      _dataPin(dataPin),
      _rxHead(0),
      _rxTail(0),
      _bitIndex(0),
      _incomingByte(0),
      _parityCount(0),
      _parityOk(false),
      _lastEdgeUs(0),
      _rxEnabled(false),
      _errorCount(0),
      _packetIndex(0),
      _packetSize(3),
      _lastPacketByteMs(0),
      _lastActivityMs(0),
      _lastWatchdogMs(0),
      _packetCount(0),
      _connected(false),
      _hasWheel(false),
      _deviceId(0x00) {
    memset(_packetBuf, 0, sizeof(_packetBuf));
}

void PS2Mouse::releaseLine(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
}

void PS2Mouse::pullLineLow(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

bool PS2Mouse::waitLineState(uint8_t pin, uint8_t expectedState, uint32_t timeoutUs) {
    uint32_t startUs = micros();
    while (digitalRead(pin) != expectedState) {
        if ((micros() - startUs) >= timeoutUs) {
            return false;
        }
    }
    return true;
}

void IRAM_ATTR PS2Mouse::handleClockInterrupt() {
    if (!_rxEnabled) {
        return;
    }

    uint32_t nowUs = micros();
    uint8_t dataBit = digitalRead(_dataPin) ? 1 : 0;

    // Reset frame state machine if too much time elapsed since the last clock edge
    if ((nowUs - _lastEdgeUs) > PS2_FRAME_TIMEOUT_US) {
        _bitIndex = 0;
        _incomingByte = 0;
        _parityCount = 0;
    }
    _lastEdgeUs = nowUs;

    if (_bitIndex == 0) {
        // Bit 0: Start bit (must be LOW)
        if (dataBit == 0) {
            _incomingByte = 0;
            _parityCount = 0;
            _parityOk = false;
            _bitIndex = 1;
        } else {
            _errorCount++;
        }
    } else if (_bitIndex >= 1 && _bitIndex <= 8) {
        // Bits 1..8: Data bits (LSB first)
        if (dataBit) {
            _incomingByte |= (1U << (_bitIndex - 1));
            _parityCount++;
        }
        _bitIndex++;
    } else if (_bitIndex == 9) {
        // Bit 9: Odd parity bit (total number of 1s including parity must be odd)
        _parityOk = (((_parityCount + dataBit) & 0x01) == 1);
        _bitIndex = 10;
    } else if (_bitIndex == 10) {
        // Bit 10: Stop bit (must be HIGH)
        if (dataBit == 1 && _parityOk) {
            size_t nextHead = (_rxHead + 1) % RX_BUFFER_SIZE;
            if (nextHead != _rxTail) {
                _rxBuffer[_rxHead] = _incomingByte;
                _rxHead = nextHead;
            } else {
                _errorCount++;
            }
        } else {
            _errorCount++;
        }
        _bitIndex = 0;
    }
}

bool PS2Mouse::readByteFromBuffer(uint8_t &outByte) {
    if (_rxTail == _rxHead) {
        return false;
    }
    outByte = _rxBuffer[_rxTail];
    _rxTail = (_rxTail + 1) % RX_BUFFER_SIZE;
    return true;
}

bool PS2Mouse::waitByte(uint8_t &outByte, uint32_t timeoutMs) {
    uint32_t startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        if (readByteFromBuffer(outByte)) {
            return true;
        }
        delayMicroseconds(100);
    }
    return false;
}

void PS2Mouse::clearRxBuffer() {
    _rxTail = _rxHead;
    _packetIndex = 0;
}

bool PS2Mouse::sendByte(uint8_t data) {
    // Temporarily disable RX state machine while host drives the bus
    _rxEnabled = false;
    _bitIndex = 0;

    // 1. Inhibit communication by pulling CLK low for >= 100 us
    pullLineLow(_clkPin);
    delayMicroseconds(120);

    // 2. Pull DATA low (Request-to-Send / Start bit = 0)
    pullLineLow(_dataPin);
    delayMicroseconds(20);

    // 3. Release CLK (HIGH) so the mouse begins generating clock pulses
    releaseLine(_clkPin);

    // Compute odd parity bit (1 if number of 1-bits in data is even, else 0)
    uint8_t ones = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (data & (1U << i)) {
            ones++;
        }
    }
    uint8_t parityBit = (ones & 0x01) ? 0 : 1;

    // 4. Shift out 8 data bits (LSB first) on each falling edge of CLK
    for (uint8_t i = 0; i < 8; i++) {
        // Wait for mouse to bring CLK low (up to 20 ms for first bit, 2 ms after)
        uint32_t timeoutUs = (i == 0) ? 20000 : 2500;
        if (!waitLineState(_clkPin, LOW, timeoutUs)) {
            releaseLine(_dataPin);
            releaseLine(_clkPin);
            _rxEnabled = true;
            return false;
        }

        if (data & (1U << i)) {
            releaseLine(_dataPin);
        } else {
            pullLineLow(_dataPin);
        }

        // Wait for mouse to release CLK high
        if (!waitLineState(_clkPin, HIGH, 2500)) {
            releaseLine(_dataPin);
            releaseLine(_clkPin);
            _rxEnabled = true;
            return false;
        }
    }

    // 5. Shift out Odd Parity bit on next falling edge of CLK
    if (!waitLineState(_clkPin, LOW, 2500)) {
        releaseLine(_dataPin);
        releaseLine(_clkPin);
        _rxEnabled = true;
        return false;
    }
    if (parityBit) {
        releaseLine(_dataPin);
    } else {
        pullLineLow(_dataPin);
    }
    if (!waitLineState(_clkPin, HIGH, 2500)) {
        releaseLine(_dataPin);
        releaseLine(_clkPin);
        _rxEnabled = true;
        return false;
    }

    // 6. Shift out Stop bit (release DATA line) on next falling edge of CLK
    if (!waitLineState(_clkPin, LOW, 2500)) {
        releaseLine(_dataPin);
        releaseLine(_clkPin);
        _rxEnabled = true;
        return false;
    }
    releaseLine(_dataPin);
    if (!waitLineState(_clkPin, HIGH, 2500)) {
        _rxEnabled = true;
        return false;
    }

    // 7. Wait for mouse to pull DATA low and CLK low (ACK bit)
    if (!waitLineState(_dataPin, LOW, 5000)) {
        _rxEnabled = true;
        return false;
    }
    if (!waitLineState(_clkPin, LOW, 2500)) {
        _rxEnabled = true;
        return false;
    }

    // Wait for mouse to release CLK and DATA high at end of ACK pulse
    waitLineState(_clkPin, HIGH, 2500);
    waitLineState(_dataPin, HIGH, 2500);

    // Re-enable interrupt receiver immediately so the 0xFA ACK byte is captured
    _bitIndex = 0;
    _lastEdgeUs = micros();
    _rxEnabled = true;
    return true;
}

bool PS2Mouse::sendCommand(uint8_t cmd) {
    clearRxBuffer();
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (!sendByte(cmd)) {
            delay(10);
            continue;
        }
        uint8_t resp = 0;
        if (waitByte(resp, 100)) {
            if (resp == PS2_RESP_ACK) {
                return true;
            }
            if (resp == PS2_RESP_RESEND) {
                continue;
            }
        }
    }
    return false;
}

bool PS2Mouse::sendCommandWithArg(uint8_t cmd, uint8_t arg) {
    if (!sendCommand(cmd)) {
        return false;
    }
    return sendCommand(arg);
}

bool PS2Mouse::setSampleRate(uint8_t rate) {
    return sendCommandWithArg(PS2_CMD_SET_SAMPLE_RATE, rate);
}

bool PS2Mouse::setResolution(uint8_t resolutionCode) {
    return sendCommandWithArg(PS2_CMD_SET_RESOLUTION, resolutionCode & 0x03);
}

bool PS2Mouse::enableStreaming() {
    return sendCommand(PS2_CMD_ENABLE_REPORTING);
}

bool PS2Mouse::tryEnableIntelliMouse() {
    // Microsoft IntelliMouse magic knock sequence: sample rates 200, 100, 80
    if (!setSampleRate(200)) return false;
    if (!setSampleRate(100)) return false;
    if (!setSampleRate(80))  return false;

    if (!sendCommand(PS2_CMD_GET_DEVICE_ID)) {
        return false;
    }
    uint8_t id = 0;
    if (waitByte(id, 100)) {
        _deviceId = id;
        _hasWheel = (id == 0x03 || id == 0x04);
        _packetSize = _hasWheel ? 4 : 3;
        return _hasWheel;
    }
    return false;
}

bool PS2Mouse::resetDevice() {
    clearRxBuffer();
    if (!sendCommand(PS2_CMD_RESET)) {
        return false;
    }

    // Wait up to 750 ms for Basic Assurance Test (BAT = 0xAA) and Device ID (0x00)
    uint8_t bat = 0;
    uint8_t devId = 0;
    if (!waitByte(bat, 750) || bat != PS2_RESP_BAT_OK) {
        return false;
    }
    waitByte(devId, 200);
    _deviceId = 0x00;
    _hasWheel = false;
    _packetSize = 3;
    return true;
}

bool PS2Mouse::configureDevice() {
    // Attempt a full reset first; if the mouse doesn't respond to 0xFF,
    // fall back to Set Defaults (0xF6) for clones that skip BAT.
    if (!resetDevice()) {
        if (!sendCommand(PS2_CMD_SET_DEFAULTS)) {
            _connected = false;
            return false;
        }
    }

    // Detect if mouse supports IntelliMouse 4-byte protocol (for scroll wheel / 3rd button)
    tryEnableIntelliMouse();

    // Configure 1:1 linear scaling, resolution, and sample rate
    sendCommand(PS2_CMD_SET_SCALING_1_1);
    setResolution(PS2_DEFAULT_RESOLUTION);
    setSampleRate(PS2_DEFAULT_SAMPLE_RATE);

    // Enable continuous data reporting (0xF4)
    if (!enableStreaming()) {
        _connected = false;
        return false;
    }

    clearRxBuffer();
    _connected = true;
    _lastActivityMs = millis();
    return true;
}

bool PS2Mouse::begin() {
    s_ps2Instance = this;

    releaseLine(_clkPin);
    releaseLine(_dataPin);
    delay(50);

    _rxEnabled = true;
    attachInterrupt(digitalPinToInterrupt(_clkPin), ps2ClockIsrTrampoline, FALLING);

    // Allow power-on BAT (0xAA 0x00) if mouse was just powered up
    delay(200);
    clearRxBuffer();

    return configureDevice();
}

bool PS2Mouse::poll(PS2MousePacket &outPacket) {
    uint8_t b = 0;
    uint32_t nowMs = millis();

    // If partial packet bytes were received long ago (> 50 ms), discard stale partial packet
    if (_packetIndex > 0 && (nowMs - _lastPacketByteMs) > 50) {
        _packetIndex = 0;
    }

    while (readByteFromBuffer(b)) {
        _lastPacketByteMs = nowMs;
        _lastActivityMs = nowMs;

        // Detect hot-plug self-test completion (0xAA followed by 0x00) when at packet boundary
        if (_packetIndex == 0 && b == PS2_RESP_BAT_OK) {
            uint8_t nextByte = 0;
            if (waitByte(nextByte, 50) && nextByte == 0x00) {
                configureDevice();
                return false;
            }
        }

        // Byte 0 of a standard PS/2 mouse packet MUST have bit 3 (0x08) set to 1
        if (_packetIndex == 0 && ((b & 0x08) == 0)) {
            _errorCount++;
            continue; // Re-synchronize stream until a valid header byte arrives
        }

        _packetBuf[_packetIndex++] = b;

        if (_packetIndex >= _packetSize) {
            _packetIndex = 0;
            _packetCount++;
            _connected = true;

            uint8_t state = _packetBuf[0];
            uint8_t rawX  = _packetBuf[1];
            uint8_t rawY  = _packetBuf[2];

            outPacket.leftButton   = (state & 0x01) != 0;
            outPacket.rightButton  = (state & 0x02) != 0;
            outPacket.middleButton = (state & 0x04) != 0;
            outPacket.xOverflow    = (state & 0x40) != 0;
            outPacket.yOverflow    = (state & 0x80) != 0;

            // Reconstruct 9-bit signed two's complement X and Y deltas
            int16_t dx = static_cast<int16_t>(rawX) - ((state & 0x10) ? 256 : 0);
            int16_t dy = static_cast<int16_t>(rawY) - ((state & 0x20) ? 256 : 0);

            // Clamp on hardware overflow
            if (outPacket.xOverflow) {
                dx = (state & 0x10) ? -255 : 255;
            }
            if (outPacket.yOverflow) {
                dy = (state & 0x20) ? -255 : 255;
            }

            outPacket.dx = dx;
            outPacket.dy = dy;
            outPacket.dz = (_packetSize == 4) ? static_cast<int8_t>(_packetBuf[3]) : 0;
            return true;
        }
    }

    return false;
}

void PS2Mouse::update() {
    uint32_t nowMs = millis();
    if ((nowMs - _lastWatchdogMs) < PS2_WATCHDOG_MS) {
        return;
    }
    _lastWatchdogMs = nowMs;

    // If the mouse has never connected, retry initialization periodically
    if (!_connected) {
        configureDevice();
    }
}
