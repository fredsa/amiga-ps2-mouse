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
static constexpr uint8_t PS2_CMD_STATUS_REQUEST    = 0xE9;
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
      _rxErrorHead(0),
      _bitIndex(0),
      _incomingByte(0),
      _parityCount(0),
      _parityOk(false),
      _lastEdgeUs(0),
      _rxEnabled(false),
      _rxFrameError(false),
      _errorCount(0),
      _packetIndex(0),
      _packetSize(3),
      _lastPacketByteUs(0),
      _lastActivityMs(0),
      _lastWatchdogMs(0),
      _packetCount(0),
      _connected(false),
      _hasWheel(false),
      _deviceId(0x00),
      _defaultStatus{},
      _status{} {
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

    // Verify CLK is actually LOW. On a genuine PS/2 falling edge, CLK remains
    // LOW for 30-50 us. False triggers from slow level-shifter rising edges or
    // crosstalk spikes (e.g. DATA or adjacent GPIO transitions) read HIGH.
    if (digitalRead(_clkPin) != LOW) {
        return;
    }
    delayMicroseconds(2);
    if (digitalRead(_clkPin) != LOW) {
        return;
    }

    uint32_t nowUs = micros();
    uint32_t dtUs  = nowUs - _lastEdgeUs;

    // Reject switch bounce / ringing (< 35 us; valid PS/2 clock period is 60-100 us)
    if (dtUs < PS2_MIN_CLOCK_PERIOD_US) {
        return;
    }

    uint8_t dataBit = digitalRead(_dataPin) ? 1 : 0;

    // Reset frame state machine if too much time elapsed since the last clock edge
    if (dtUs > PS2_FRAME_TIMEOUT_US) {
        if (_bitIndex != 0) {
            // Previous 11-bit frame was incomplete; abort any partial packet
            _errorCount++;
            _rxErrorHead = _rxHead;
            _rxFrameError = true;
        }
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
            _rxErrorHead = _rxHead;
            _rxFrameError = true;
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
                _rxTimeUs[_rxHead] = nowUs;
                _rxHead = nextHead;
            } else {
                _errorCount++;
                _rxErrorHead = _rxHead;
                _rxFrameError = true;
            }
        } else {
            _errorCount++;
            _rxErrorHead = _rxHead;
            _rxFrameError = true;
        }
        _bitIndex = 0;
    }
}

bool PS2Mouse::readByteFromBuffer(uint8_t &outByte, uint32_t &outTimeUs) {
    if (_rxTail == _rxHead) {
        return false;
    }
    outByte = _rxBuffer[_rxTail];
    outTimeUs = _rxTimeUs[_rxTail];
    _rxTail = (_rxTail + 1) % RX_BUFFER_SIZE;
    return true;
}

bool PS2Mouse::readByteFromBuffer(uint8_t &outByte) {
    uint32_t unusedTimeUs = 0;
    return readByteFromBuffer(outByte, unusedTimeUs);
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
    _rxErrorHead = _rxHead;
    _packetIndex = 0;
    _rxFrameError = false;
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
            _lastEdgeUs = micros() - 1000;
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
            _lastEdgeUs = micros() - 1000;
            _rxEnabled = true;
            return false;
        }
    }

    // 5. Shift out Odd Parity bit on next falling edge of CLK
    if (!waitLineState(_clkPin, LOW, 2500)) {
        releaseLine(_dataPin);
        releaseLine(_clkPin);
        _lastEdgeUs = micros() - 1000;
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
        _lastEdgeUs = micros() - 1000;
        _rxEnabled = true;
        return false;
    }

    // 6. Shift out Stop bit (release DATA line) on next falling edge of CLK
    if (!waitLineState(_clkPin, LOW, 2500)) {
        releaseLine(_dataPin);
        releaseLine(_clkPin);
        _lastEdgeUs = micros() - 1000;
        _rxEnabled = true;
        return false;
    }
    releaseLine(_dataPin);
    if (!waitLineState(_clkPin, HIGH, 2500)) {
        _lastEdgeUs = micros() - 1000;
        _rxEnabled = true;
        return false;
    }

    // 7. Wait for mouse to pull DATA low and CLK low (ACK bit)
    if (!waitLineState(_dataPin, LOW, 5000)) {
        _lastEdgeUs = micros() - 1000;
        _rxEnabled = true;
        return false;
    }
    if (!waitLineState(_clkPin, LOW, 2500)) {
        _lastEdgeUs = micros() - 1000;
        _rxEnabled = true;
        return false;
    }

    // Wait for mouse to release CLK and DATA high at end of ACK pulse
    waitLineState(_clkPin, HIGH, 2500);
    waitLineState(_dataPin, HIGH, 2500);

    // Re-enable interrupt receiver immediately so the 0xFA ACK byte is captured
    _bitIndex = 0;
    _rxFrameError = false;
    _lastEdgeUs = micros() - 1000;
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
    // 1. Microsoft IntelliMouse magic knock sequence (wheel / scroll nub): 200, 100, 80
    if (!setSampleRate(200)) return false;
    if (!setSampleRate(100)) return false;
    if (!setSampleRate(80))  return false;

    if (!sendCommand(PS2_CMD_GET_DEVICE_ID)) {
        return false;
    }
    uint8_t id = 0;
    if (!waitByte(id, 100)) {
        return false;
    }

    // 2. If IntelliMouse (0x03) succeeded, also try IntelliMouse Explorer
    //    magic knock sequence (5-button + 2D scroll nub/wheel): 200, 200, 80
    if (id == 0x03) {
        if (setSampleRate(200) && setSampleRate(200) && setSampleRate(80)) {
            if (sendCommand(PS2_CMD_GET_DEVICE_ID)) {
                uint8_t expId = 0;
                if (waitByte(expId, 100) && (expId == 0x03 || expId == 0x04)) {
                    id = expId;
                }
            }
        }
    }

    _deviceId = id;
    _hasWheel = (id == 0x03 || id == 0x04);
    _packetSize = _hasWheel ? 4 : 3;
    return _hasWheel;
}

bool PS2Mouse::queryStatus(PS2MouseStatus &outStatus) {
    outStatus = PS2MouseStatus{};
    clearRxBuffer();
    if (!sendCommand(PS2_CMD_STATUS_REQUEST)) {
        return false;
    }

    uint8_t s0 = 0, s1 = 0, s2 = 0;
    if (!waitByte(s0, 100) || !waitByte(s1, 100) || !waitByte(s2, 100)) {
        return false;
    }

    outStatus.valid            = true;
    outStatus.rawStatus        = s0;
    outStatus.remoteMode       = (s0 & 0x40) != 0;
    outStatus.reportingEnabled = (s0 & 0x20) != 0;
    outStatus.scaling2To1      = (s0 & 0x10) != 0;
    outStatus.leftButton       = (s0 & 0x04) != 0;
    outStatus.middleButton     = (s0 & 0x02) != 0;
    outStatus.rightButton      = (s0 & 0x01) != 0;
    outStatus.resolutionCode   = s1;
    outStatus.sampleRate       = s2;
    return true;
}

static uint8_t resolutionCountsPerMm(uint8_t code) {
    return (code <= 3) ? static_cast<uint8_t>(1U << code) : 0;
}

void PS2Mouse::printInterrogatedConfig() const {
#if ENABLE_SERIAL_DEBUG
    const char *devTypeStr =
        (_deviceId == 0x04) ? "IntelliMouse Explorer (5-btn + scroll)" :
        (_deviceId == 0x03) ? "IntelliMouse (3-btn + scroll wheel/nub)" :
                              "Standard PS/2";

    Serial.println(F("----------------------------------------------------------"));
    Serial.printf("[PS2Mouse] Mouse Detected! Device ID=0x%02X (%s, %u-byte packets)\n",
                  _deviceId, devTypeStr, static_cast<unsigned>(_packetSize));

    if (_defaultStatus.valid) {
        Serial.printf("  Reset Defaults (0xE9) : Scaling=%s | Resolution=%u cnt/mm (code %u) | SampleRate=%u Hz | Mode=%s | Raw=[0x%02X 0x%02X 0x%02X]\n",
                      _defaultStatus.scaling2To1 ? "2:1" : "1:1",
                      static_cast<unsigned>(resolutionCountsPerMm(_defaultStatus.resolutionCode)),
                      static_cast<unsigned>(_defaultStatus.resolutionCode),
                      static_cast<unsigned>(_defaultStatus.sampleRate),
                      _defaultStatus.remoteMode ? "Remote" : "Stream",
                      _defaultStatus.rawStatus,
                      _defaultStatus.resolutionCode,
                      _defaultStatus.sampleRate);
    } else {
        Serial.println(F("  Reset Defaults (0xE9) : (not reported by device)"));
    }

    if (_status.valid) {
        Serial.printf("  Active Config  (0xE9) : Scaling=%s | Resolution=%u cnt/mm (code %u) | SampleRate=%u Hz | Mode=%s | Reporting=%s | Btns(L/M/R)=%d/%d/%d | Raw=[0x%02X 0x%02X 0x%02X]\n",
                      _status.scaling2To1 ? "2:1" : "1:1",
                      static_cast<unsigned>(resolutionCountsPerMm(_status.resolutionCode)),
                      static_cast<unsigned>(_status.resolutionCode),
                      static_cast<unsigned>(_status.sampleRate),
                      _status.remoteMode ? "Remote" : "Stream",
                      _status.reportingEnabled ? "ON" : "OFF",
                      _status.leftButton ? 1 : 0,
                      _status.middleButton ? 1 : 0,
                      _status.rightButton ? 1 : 0,
                      _status.rawStatus,
                      _status.resolutionCode,
                      _status.sampleRate);
    } else {
        Serial.println(F("  Active Config  (0xE9) : (not reported by device)"));
    }
    Serial.println(F("----------------------------------------------------------"));
#endif
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
    _deviceId = 0x00;
    _hasWheel = false;
    _packetSize = 3;
    _defaultStatus = PS2MouseStatus{};
    _status        = PS2MouseStatus{};

    // Attempt a full reset first; if the mouse doesn't respond to 0xFF,
    // fall back to Set Defaults (0xF6) for clones that skip BAT.
    if (!resetDevice()) {
        if (!sendCommand(PS2_CMD_SET_DEFAULTS)) {
            _connected = false;
            return false;
        }
    }

    // Interrogate the mouse's power-on default configuration (0xE9) while reporting is disabled
    queryStatus(_defaultStatus);

    // Configure 1:1 scaling and resolution before negotiating IntelliMouse extensions
    sendCommand(PS2_CMD_SET_SCALING_1_1);
    setResolution(PS2_DEFAULT_RESOLUTION);

    // Enable full mouse functionality (IntelliMouse 0x03 or Explorer 0x04) if supported
    tryEnableIntelliMouse();

    // Set final operating sample rate
    setSampleRate(PS2_DEFAULT_SAMPLE_RATE);

    // Re-query Device ID (0xF2) after all sample-rate/configuration commands so
    // _packetSize (3 vs 4 bytes) is guaranteed to match the mouse's actual mode.
    if (sendCommand(PS2_CMD_GET_DEVICE_ID)) {
        uint8_t finalId = 0;
        if (waitByte(finalId, 100)) {
            _deviceId = finalId;
            _hasWheel = (finalId == 0x03 || finalId == 0x04);
            _packetSize = _hasWheel ? 4 : 3;
        }
    }

    // Interrogate active configuration (scaling, resolution, sample rate, mode) via 0xE9
    // before enabling streaming so no movement packets can interleave with the 3-byte reply.
    queryStatus(_status);

    // Enable continuous data reporting (0xF4)
    if (!enableStreaming()) {
        _connected = false;
        return false;
    }
    if (_status.valid) {
        _status.reportingEnabled = true;
        _status.rawStatus |= 0x20;
    }

    clearRxBuffer();
    _connected = true;
    _lastActivityMs = millis();
    _lastPacketByteUs = micros();

    printInterrogatedConfig();
    return true;
}

bool PS2Mouse::begin() {
    s_ps2Instance = this;

    releaseLine(_clkPin);
    releaseLine(_dataPin);
    delay(50);

    _lastEdgeUs = micros() - 1000;
    _rxEnabled = true;
    attachInterrupt(digitalPinToInterrupt(_clkPin), ps2ClockIsrTrampoline, FALLING);

    // Allow power-on BAT (0xAA 0x00) if mouse was just powered up
    delay(200);
    clearRxBuffer();

    return configureDevice();
}

bool PS2Mouse::poll(PS2MousePacket &outPacket) {
    uint8_t b = 0;
    uint32_t byteTimeUs = 0;

    while (true) {
        // If the ISR detected any framing/parity error, discard both any
        // partial packet in _packetBuf AND any pre-error bytes in _rxBuffer.
        if (_rxFrameError) {
            _rxFrameError = false;
            _rxTail = _rxErrorHead;
            _packetIndex = 0;
        }

        if (!readByteFromBuffer(b, byteTimeUs)) {
            break;
        }
        uint32_t nowMs = millis();

        // Bytes within the same packet arrive ~1 ms apart on the wire, whereas
        // consecutive packets are ~10 ms apart (at 100 Hz). Comparing ISR capture
        // timestamps ensures USB Serial debug printing delays never cause false
        // packet timeouts. Any wire gap > 3.5 ms means a new packet is starting.
        if (_packetIndex > 0 && (byteTimeUs - _lastPacketByteUs) > PS2_PACKET_TIMEOUT_US) {
            _packetIndex = 0;
        }
        _lastPacketByteUs = byteTimeUs;

        _lastActivityMs = nowMs;

        // Detect hot-plug or brownout self-test completion (0xAA followed by 0x00
        // and no 3rd packet byte within 5 ms). Because 0xAA has bit 7 set (0xC0 != 0),
        // it is never a valid normal header byte, and verifying silence after 0x00
        // distinguishes a 2-byte BAT reset from any 3/4-byte movement packet even
        // if a power glitch resets the mouse < 500 ms after movement.
        if (_packetIndex == 0 && b == PS2_RESP_BAT_OK) {
            uint8_t nextByte = 0;
            uint8_t thirdByte = 0;
            if (waitByte(nextByte, 50) && nextByte == 0x00 && !waitByte(thirdByte, 5)) {
                configureDevice();
                return false;
            }
            _errorCount++;
            continue;
        }

        // Validate Byte 0 (header/state byte):
        // - Bit 3 (0x08) MUST be 1 on all standard PS/2 packets.
        // - Bits 6 & 7 (0xC0, X/Y overflow) must be 0. Rejecting 0xC0 != 0 at
        //   Byte 0 prevents negative movement deltas (-1..-128 = 0x80..0xFF)
        //   from ever being mistaken for a header byte if a byte was dropped.
        if (_packetIndex == 0 && ((b & 0x08) == 0 || (b & 0xC0) != 0)) {
            _errorCount++;
            continue;
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

            outPacket.dx = dx;
            outPacket.dy = dy;

            if (_packetSize == 4) {
                uint8_t b3 = _packetBuf[3];
                outPacket.rawZ = b3;
                if (_deviceId == 0x04 && (b3 & 0xC0) == 0) {
                    // IntelliMouse Explorer (0x04): bits 0..3 = 4-bit signed scroll,
                    // bit 4 = Button 4, bit 5 = Button 5
                    uint8_t z4 = b3 & 0x0F;
                    outPacket.dz = (z4 & 0x08) ? static_cast<int8_t>(z4) - 16
                                               : static_cast<int8_t>(z4);
                    outPacket.button4 = (b3 & 0x10) != 0;
                    outPacket.button5 = (b3 & 0x20) != 0;
                } else {
                    // Standard IntelliMouse (0x03) or sign-extended 8-bit Z delta
                    outPacket.dz = static_cast<int8_t>(b3);
                    outPacket.button4 = false;
                    outPacket.button5 = false;
                }
            } else {
                outPacket.rawZ = 0;
                outPacket.dz = 0;
                outPacket.button4 = false;
                outPacket.button5 = false;
            }
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
