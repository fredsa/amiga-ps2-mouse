#ifndef PS2_MOUSE_H
#define PS2_MOUSE_H

#include <Arduino.h>
#include "config.h"

// Decoded PS/2 mouse report structure
struct PS2MousePacket {
    int16_t dx;          // Signed X movement (+Right, -Left)
    int16_t dy;          // Signed Y movement (+Up, -Down in PS/2 coordinates)
    int8_t  dz;          // Signed Z scroll wheel / scroll nub movement (if IntelliMouse active)
    uint8_t rawZ;        // Raw 4th byte from IntelliMouse / Explorer packet
    bool    leftButton;  // Left mouse button pressed
    bool    rightButton; // Right mouse button pressed
    bool    middleButton;// Middle mouse button pressed
    bool    button4;     // 4th mouse button pressed (IntelliMouse Explorer 0x04)
    bool    button5;     // 5th mouse button pressed (IntelliMouse Explorer 0x04)
    bool    xOverflow;   // X delta overflow flag
    bool    yOverflow;   // Y delta overflow flag
};

class PS2Mouse {
public:
    PS2Mouse(uint8_t clkPin, uint8_t dataPin);

    // Initialize GPIO pins, attach falling-edge interrupt on CLK, and reset/configure mouse
    bool begin();

    // Poll and decode incoming bytes from the ISR ring buffer; returns true when a full
    // movement packet has been decoded into `outPacket`.
    bool poll(PS2MousePacket &outPacket);

    // Periodic maintenance: handles hot-plug detection and bus recovery
    void update();

    // Send configuration commands to the PS/2 mouse
    bool setSampleRate(uint8_t rate);
    bool setResolution(uint8_t resolutionCode);
    bool enableStreaming();
    bool resetDevice();

    // Accessors
    bool isConnected() const { return _connected; }
    bool hasScrollWheel() const { return _hasWheel; }
    uint8_t getDeviceId() const { return _deviceId; }
    uint32_t getPacketCount() const { return _packetCount; }
    uint32_t getErrorCount() const { return _errorCount; }

    // ISR entry point (must be public so the static trampoline can invoke it)
    void IRAM_ATTR handleClockInterrupt();

private:
    uint8_t _clkPin;
    uint8_t _dataPin;

    // ISR state for Device-to-Host 11-bit frame reception
    static constexpr size_t RX_BUFFER_SIZE = 64;
    volatile uint8_t  _rxBuffer[RX_BUFFER_SIZE];
    volatile uint32_t _rxTimeUs[RX_BUFFER_SIZE];
    volatile size_t   _rxHead;
    volatile size_t   _rxTail;
    volatile size_t   _rxErrorHead;
    volatile uint8_t  _bitIndex;
    volatile uint8_t  _incomingByte;
    volatile uint8_t  _parityCount;
    volatile bool     _parityOk;
    volatile uint32_t _lastEdgeUs;
    volatile bool     _rxEnabled;
    volatile bool     _rxFrameError;
    volatile uint32_t _errorCount;

    // Packet assembly state
    uint8_t  _packetBuf[4];
    uint8_t  _packetIndex;
    uint8_t  _packetSize;
    uint32_t _lastPacketByteUs;
    uint32_t _lastActivityMs;
    uint32_t _lastWatchdogMs;
    uint32_t _packetCount;

    // Device state
    bool     _connected;
    bool     _hasWheel;
    uint8_t  _deviceId;

    // Low-level open-drain line helpers
    void releaseLine(uint8_t pin);
    void pullLineLow(uint8_t pin);
    bool waitLineState(uint8_t pin, uint8_t expectedState, uint32_t timeoutUs);

    // Ring buffer helpers
    bool readByteFromBuffer(uint8_t &outByte);
    bool readByteFromBuffer(uint8_t &outByte, uint32_t &outTimeUs);
    bool waitByte(uint8_t &outByte, uint32_t timeoutMs);
    void clearRxBuffer();

    // Host-to-Device command primitives
    bool sendByte(uint8_t data);
    bool sendCommand(uint8_t cmd);
    bool sendCommandWithArg(uint8_t cmd, uint8_t arg);
    bool tryEnableIntelliMouse();
    bool configureDevice();
};

#endif // PS2_MOUSE_H
