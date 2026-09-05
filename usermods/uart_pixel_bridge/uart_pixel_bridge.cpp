#include "wled.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <HardwareSerial.h>
#include <new>

/**
 * UART Pixel Bridge usermod
 *
 * Master:
 *   Captures a logical LED range after WLED renders each frame and sends RGB888
 *   frames through a dedicated ESP32 UART.
 *
 * Receiver:
 *   Receives complete frames, validates CRC-16/CCITT-FALSE, and injects them into
 *   WLED's realtime framebuffer. Incomplete or corrupt frames are never shown.
 *
 * Electrical layer:
 *   Use two 3.3 V differential transceivers (for example MAX3485). The usermod
 *   handles UART only; DE and /RE may be tied permanently for one-way operation.
 */
class UartPixelBridge : public Usermod {
  private:
    enum class Role : uint8_t {
      Master   = 0,
      Receiver = 1
    };

    enum class BufferState : uint8_t {
      Free,
      Filling,
      Queued,
      Active
    };

    enum class RxState : uint8_t {
      FindMagic,
      ReadHeader,
      ReadPayload,
      ReadCrcLow,
      ReadCrcHigh
    };

    enum class ReceiverState : uint8_t {
      WaitingForMaster,
      Active,
      Holding,
      Fading,
      Blackout
    };

    enum class FailsafeAction : uint8_t {
      KeepLast = 0,
      Black = 1,
      FadeToBlack = 2
    };

    struct RxMeta {
      uint32_t sequence = 0;
      uint16_t ledCount = 0;
      uint8_t brightness = 0;
    };

    static constexpr uint8_t PROTOCOL_VERSION = 1;
    static constexpr uint8_t MAGIC[4] = {'W', 'P', 'B', '1'};
    static constexpr size_t MAGIC_SIZE = sizeof(MAGIC);
    static constexpr size_t HEADER_AFTER_MAGIC_SIZE = 12;
    static constexpr size_t HEADER_SIZE = MAGIC_SIZE + HEADER_AFTER_MAGIC_SIZE;
    static constexpr size_t CRC_SIZE = 2;
    static constexpr uint16_t MAX_LED_COUNT = 2048;
    static constexpr uint8_t BUFFER_COUNT = 3;
    static constexpr uint32_t MIN_BAUD = 115200;
    static constexpr uint32_t MAX_BAUD = 5000000;

    static const char _name[];

    // Persistent settings.
    bool _enabled = true;
    Role _role = Role::Master;
    uint8_t _uartNumber = 1;
    int8_t _txPin = 17;
    int8_t _rxPin = 16;
    uint32_t _baud = 2000000;
    uint16_t _sourceStart = 0;
    uint16_t _targetStart = 0;
    uint16_t _ledCount = 300;
    uint8_t _maxFps = 60;
    uint16_t _realtimeTimeoutMs = 500;
    bool _reverse = false;
    bool _fastReceiver = false;

    // Receiver startup/failsafe policy. These settings only affect Receiver role.
    bool _startupGuardEnabled = true;
    uint8_t _startupValidFrames = 2;
    bool _failsafeEnabled = true;
    uint32_t _failsafeHoldMs = 500;
    FailsafeAction _failsafeAction = FailsafeAction::FadeToBlack;
    uint32_t _failsafeFadeMs = 500;
    bool _reacquireGuardEnabled = true;
    uint8_t _reacquireValidFrames = 2;

    // Lifecycle.
    bool _initDone = false;
    bool _running = false;
    bool _restartRequested = false;
    bool _pinAllocated = false;
    String _lastError;

    HardwareSerial* _uart = nullptr;
    TaskHandle_t _ioTaskHandle = nullptr;
    SemaphoreHandle_t _bufferMutex = nullptr;
    volatile bool _taskStop = false;

    // Three buffers allow one active LED frame, one UART fill buffer and one
    // queued newest frame without sharing storage between tasks.
    uint8_t* _buffers[BUFFER_COUNT] = {nullptr};
    BufferState _bufferState[BUFFER_COUNT] = {};
    RxMeta _rxMeta[BUFFER_COUNT];
    uint8_t* _lastAppliedPayload = nullptr;
    uint16_t _lastAppliedLedCount = 0;
    size_t _payloadSize = 0;
    size_t _frameSize = 0;

    // Receiver startup/failsafe runtime.
    ReceiverState _receiverState = ReceiverState::WaitingForMaster;
    bool _receiverHasAppliedFrame = false;
    bool _receiverOwnsRealtime = false;
    volatile uint8_t _consecutiveValidFrames = 0;
    uint32_t _bridgeStartedMs = 0;
    uint32_t _firstValidFrameDelayMs = 0;
    bool _hasFirstValidFrameDelay = false;
    uint32_t _fadeStartedMs = 0;
    uint32_t _lastFailsafeShowUs = 0;
    volatile uint32_t _failsafeEvents = 0;
    volatile uint32_t _startupFramesSuppressed = 0;
    volatile uint32_t _reacquireFramesSuppressed = 0;
    volatile uint32_t _failsafeShows = 0;

    // Master state.
    uint32_t _nextSequence = 0;
    uint32_t _lastCaptureUs = 0;
    uint32_t _frameIntervalUs = 16666;

    // Receiver parser state. Used only by the receiver task.
    RxState _rxState = RxState::FindMagic;
    uint8_t _magicPos = 0;
    uint8_t _rxHeader[HEADER_AFTER_MAGIC_SIZE] = {0};
    size_t _rxHeaderPos = 0;
    size_t _rxPayloadPos = 0;
    uint16_t _rxPayloadLength = 0;
    uint16_t _rxLedCount = 0;
    uint32_t _rxSequence = 0;
    uint8_t _rxBrightness = 0;
    uint16_t _rxCrc = 0xFFFF;
    uint16_t _rxReceivedCrc = 0;
    int8_t _rxFillBuffer = -1;

    // Runtime statistics.
    volatile uint32_t _framesCaptured = 0;
    volatile uint32_t _framesSent = 0;
    volatile uint32_t _framesReceived = 0;
    volatile uint32_t _framesApplied = 0;
    volatile uint32_t _framesDropped = 0;
    volatile uint32_t _crcErrors = 0;
    volatile uint32_t _framingErrors = 0;
    volatile uint32_t _uartWriteErrors = 0;
    volatile uint32_t _sequenceGapEvents = 0;
    volatile uint32_t _missingSequences = 0;
    volatile uint32_t _duplicateSequences = 0;
    volatile uint32_t _sequenceResyncs = 0;
    volatile uint32_t _lastValidFrameMs = 0;
    volatile uint32_t _lastReceivedSequence = 0;
    volatile uint32_t _lastAppliedSequence = 0;
    volatile uint32_t _statsStartedMs = 0;
    volatile bool _statsResetRequested = false;
    bool _hasReceivedSequence = false;
    uint64_t _applyMicrosTotal = 0;
    uint64_t _showMicrosTotal = 0;
    uint32_t _applyMicrosMax = 0;
    uint32_t _showMicrosMax = 0;
    size_t _rxDriverBufferBytes = 0;

    static uint16_t crc16Update(uint16_t crc, uint8_t data) {
      crc ^= static_cast<uint16_t>(data) << 8;
      for (uint8_t bit = 0; bit < 8; bit++) {
        crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                              : static_cast<uint16_t>(crc << 1);
      }
      return crc;
    }

    static uint16_t crc16(const uint8_t* data, size_t length, uint16_t initial = 0xFFFF) {
      uint16_t crc = initial;
      for (size_t i = 0; i < length; i++) crc = crc16Update(crc, data[i]);
      return crc;
    }

    static uint16_t readLe16(const uint8_t* data) {
      return static_cast<uint16_t>(data[0]) |
             (static_cast<uint16_t>(data[1]) << 8);
    }

    static uint32_t readLe32(const uint8_t* data) {
      return static_cast<uint32_t>(data[0]) |
             (static_cast<uint32_t>(data[1]) << 8) |
             (static_cast<uint32_t>(data[2]) << 16) |
             (static_cast<uint32_t>(data[3]) << 24);
    }

    static void writeLe16(uint8_t* data, uint16_t value) {
      data[0] = static_cast<uint8_t>(value & 0xFFU);
      data[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    }

    static void writeLe32(uint8_t* data, uint32_t value) {
      data[0] = static_cast<uint8_t>(value & 0xFFU);
      data[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
      data[2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
      data[3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
    }

    static uint8_t saturatingAdd(uint8_t a, uint8_t b) {
      const uint16_t value = static_cast<uint16_t>(a) + b;
      return static_cast<uint8_t>(value > 255U ? 255U : value);
    }

    static uint32_t saturatingAdd32(uint32_t value, uint32_t increment) {
      return increment > (0xFFFFFFFFUL - value) ? 0xFFFFFFFFUL : value + increment;
    }

    double crcErrorRatePercent() const {
      const uint64_t valid = _framesReceived;
      const uint64_t bad = _crcErrors;
      const uint64_t total = valid + bad;
      return total ? (static_cast<double>(bad) * 100.0 / static_cast<double>(total)) : 0.0;
    }

    double sequenceLossRatePercent() const {
      const uint64_t valid = _framesReceived;
      const uint64_t missing = _missingSequences;
      const uint64_t total = valid + missing;
      return total ? (static_cast<double>(missing) * 100.0 / static_cast<double>(total)) : 0.0;
    }

    double appliedFps() const {
      if (!_statsStartedMs) return 0.0;
      const uint32_t age = static_cast<uint32_t>(millis() - _statsStartedMs);
      return age ? (static_cast<double>(_framesApplied) * 1000.0 / static_cast<double>(age)) : 0.0;
    }

    uint32_t averageApplyMicros() const {
      return _framesApplied ? static_cast<uint32_t>(_applyMicrosTotal / _framesApplied) : 0U;
    }

    uint32_t averageShowMicros() const {
      return _framesApplied ? static_cast<uint32_t>(_showMicrosTotal / _framesApplied) : 0U;
    }

    void resetStatistics() {
      _framesCaptured = 0;
      _framesSent = 0;
      _framesReceived = 0;
      _framesApplied = 0;
      _framesDropped = 0;
      _crcErrors = 0;
      _framingErrors = 0;
      _uartWriteErrors = 0;
      _sequenceGapEvents = 0;
      _missingSequences = 0;
      _duplicateSequences = 0;
      _sequenceResyncs = 0;
      _lastValidFrameMs = 0;
      _lastReceivedSequence = 0;
      _lastAppliedSequence = 0;
      _hasReceivedSequence = false;
      _applyMicrosTotal = 0;
      _showMicrosTotal = 0;
      _applyMicrosMax = 0;
      _showMicrosMax = 0;
      _failsafeEvents = 0;
      _startupFramesSuppressed = 0;
      _reacquireFramesSuppressed = 0;
      _failsafeShows = 0;
      _statsStartedMs = millis();
      _statsResetRequested = false;
    }

    void trackReceivedSequence(uint32_t sequence) {
      if (!_hasReceivedSequence) {
        _lastReceivedSequence = sequence;
        _hasReceivedSequence = true;
        return;
      }

      const uint32_t delta = sequence - _lastReceivedSequence;
      if (delta == 0U) {
        _duplicateSequences++;
      } else if (delta == 1U) {
        // Expected next frame. uint32_t wrap-around is handled naturally.
      } else if (delta < 0x80000000UL) {
        // Forward jump: one or more sequence numbers were not received as valid
        // frames. This includes frames corrupted in transit and frames dropped by
        // the sender before they reached the wire. Compare with Master `dropped`
        // to distinguish transport loss from sender-side queue pressure.
        _sequenceGapEvents++;
        _missingSequences = saturatingAdd32(_missingSequences, delta - 1U);
      } else {
        // A large backwards jump normally means that the Master restarted or its
        // statistics/sequence counter was reset. Do not turn that into billions
        // of false missing frames.
        _sequenceResyncs++;
      }

      _lastReceivedSequence = sequence;
    }

    const char* receiverStateName() const {
      switch (_receiverState) {
        case ReceiverState::WaitingForMaster: return "waiting";
        case ReceiverState::Active:           return "active";
        case ReceiverState::Holding:          return "holding";
        case ReceiverState::Fading:           return "fading";
        case ReceiverState::Blackout:         return "blackout";
      }
      return "unknown";
    }

    const char* failsafeActionName() const {
      switch (_failsafeAction) {
        case FailsafeAction::KeepLast:    return "keep-last";
        case FailsafeAction::Black:       return "black";
        case FailsafeAction::FadeToBlack: return "fade-to-black";
      }
      return "unknown";
    }

    void refreshRealtimeOwnership() {
      if (!_running || _role != Role::Receiver) return;
      realtimeLock(_realtimeTimeoutMs, REALTIME_MODE_ADALIGHT);
      _receiverOwnsRealtime = true;
    }

    void showControlledRangeBlack() {
      if (_role != Role::Receiver) return;
      refreshRealtimeOwnership();
      if (realtimeOverride) return;
      const uint16_t totalLength = strip.getLengthTotal();
      for (uint16_t i = 0; i < _ledCount; i++) {
        const uint32_t target = static_cast<uint32_t>(_targetStart) + i;
        if (target >= totalLength) break;
        strip.setRealtimePixelColor(static_cast<uint16_t>(target), 0);
      }
      strip.show();
      _failsafeShows++;
    }

    void showLastPayloadScaled(uint8_t scale) {
      if (!_lastAppliedPayload || !_lastAppliedLedCount) {
        showControlledRangeBlack();
        return;
      }

      refreshRealtimeOwnership();
      if (realtimeOverride) return;
      const uint16_t totalLength = strip.getLengthTotal();
      for (uint16_t i = 0; i < _lastAppliedLedCount; i++) {
        const uint32_t target = static_cast<uint32_t>(_targetStart) + i;
        if (target >= totalLength) break;
        const size_t offset = static_cast<size_t>(i) * 3U;
        const uint8_t r = static_cast<uint8_t>((static_cast<uint16_t>(_lastAppliedPayload[offset]) * scale + 127U) / 255U);
        const uint8_t g = static_cast<uint8_t>((static_cast<uint16_t>(_lastAppliedPayload[offset + 1U]) * scale + 127U) / 255U);
        const uint8_t b = static_cast<uint8_t>((static_cast<uint16_t>(_lastAppliedPayload[offset + 2U]) * scale + 127U) / 255U);
        strip.setRealtimePixelColor(static_cast<uint16_t>(target), RGBW32(r, g, b, 0));
      }
      strip.show();
      _failsafeShows++;
    }

    void enterInitialWaitingState() {
      _receiverState = ReceiverState::WaitingForMaster;
      _receiverHasAppliedFrame = false;
      _consecutiveValidFrames = 0;
      _lastValidFrameMs = 0;
      _fadeStartedMs = 0;
      _lastFailsafeShowUs = 0;
      if (_startupGuardEnabled) showControlledRangeBlack();
    }

    void enterFailsafe(uint32_t nowMs) {
      _failsafeEvents++;
      _consecutiveValidFrames = 0;

      switch (_failsafeAction) {
        case FailsafeAction::KeepLast:
          _receiverState = ReceiverState::Holding;
          break;

        case FailsafeAction::Black:
          _receiverState = ReceiverState::Blackout;
          showControlledRangeBlack();
          break;

        case FailsafeAction::FadeToBlack:
          if (_failsafeFadeMs == 0U) {
            _receiverState = ReceiverState::Blackout;
            showControlledRangeBlack();
          } else {
            _receiverState = ReceiverState::Fading;
            _fadeStartedMs = nowMs;
            _lastFailsafeShowUs = 0;
          }
          break;
      }
    }

    void serviceFailsafeFade(uint32_t nowMs) {
      if (_receiverState != ReceiverState::Fading) return;

      const uint32_t elapsed = static_cast<uint32_t>(nowMs - _fadeStartedMs);
      if (elapsed >= _failsafeFadeMs) {
        showControlledRangeBlack();
        _receiverState = ReceiverState::Blackout;
        return;
      }

      const uint32_t nowUs = micros();
      if (_lastFailsafeShowUs &&
          static_cast<uint32_t>(nowUs - _lastFailsafeShowUs) < _frameIntervalUs) return;
      _lastFailsafeShowUs = nowUs;

      const uint32_t remaining = _failsafeFadeMs - elapsed;
      const uint8_t scale = static_cast<uint8_t>((static_cast<uint64_t>(remaining) * 255ULL) / _failsafeFadeMs);
      showLastPayloadScaled(scale);
    }

    void handleReceiverSafety() {
      if (!_running || _role != Role::Receiver) return;

      // While startup guard or failsafe owns the output, keep WLED realtime mode
      // refreshed so its normal local effect cannot appear between bridge frames.
      if ((_startupGuardEnabled && !_receiverHasAppliedFrame) || _failsafeEnabled) {
        refreshRealtimeOwnership();
      }

      if (!_receiverHasAppliedFrame || !_lastValidFrameMs) return;

      const uint32_t nowMs = millis();
      const uint32_t ageMs = static_cast<uint32_t>(nowMs - _lastValidFrameMs);

      if (!_failsafeEnabled) {
        if (_receiverState == ReceiverState::Active && ageMs > _realtimeTimeoutMs) {
          _receiverState = ReceiverState::WaitingForMaster;
        }
        return;
      }

      if (_receiverState == ReceiverState::Active && ageMs > _failsafeHoldMs) {
        enterFailsafe(nowMs);
      }

      if (_receiverState == ReceiverState::Fading) serviceFailsafeFade(nowMs);
    }

    bool receiverMayApplyFrame() {
      if (!_receiverHasAppliedFrame) {
        if (!_startupGuardEnabled) return true;
        if (_consecutiveValidFrames >= _startupValidFrames) return true;
        _startupFramesSuppressed++;
        return false;
      }

      if (_receiverState == ReceiverState::Active) return true;
      if (!_failsafeEnabled) return true;

      if (!_reacquireGuardEnabled) return true;
      if (_consecutiveValidFrames >= _reacquireValidFrames) return true;
      _reacquireFramesSuppressed++;
      return false;
    }

    bool takeMutex(TickType_t waitTicks = 0) {
      return _bufferMutex && xSemaphoreTake(_bufferMutex, waitTicks) == pdTRUE;
    }

    void giveMutex() {
      if (_bufferMutex) xSemaphoreGive(_bufferMutex);
    }

    void accountQueuedFrameDiscard() {
      if (_role == Role::Receiver) {
        if (!_receiverHasAppliedFrame && _startupGuardEnabled) {
          _startupFramesSuppressed++;
          return;
        }
        if (_receiverHasAppliedFrame && _receiverState != ReceiverState::Active && _failsafeEnabled) {
          _reacquireFramesSuppressed++;
          return;
        }
      }
      _framesDropped++;
    }

    int8_t acquireFreeBuffer(bool reclaimQueued) {
      int8_t result = -1;
      if (!takeMutex(pdMS_TO_TICKS(2))) return result;

      for (uint8_t i = 0; i < BUFFER_COUNT; i++) {
        if (_bufferState[i] == BufferState::Free) {
          result = static_cast<int8_t>(i);
          break;
        }
      }

      if (result < 0 && reclaimQueued) {
        for (uint8_t i = 0; i < BUFFER_COUNT; i++) {
          if (_bufferState[i] == BufferState::Queued) {
            result = static_cast<int8_t>(i);
            accountQueuedFrameDiscard();
            break;
          }
        }
      }

      if (result >= 0) _bufferState[result] = BufferState::Filling;
      giveMutex();
      return result;
    }

    void releaseBuffer(int8_t index) {
      if (index < 0 || index >= static_cast<int8_t>(BUFFER_COUNT)) return;
      if (!takeMutex(portMAX_DELAY)) return;
      _bufferState[index] = BufferState::Free;
      giveMutex();
    }

    void queueBuffer(int8_t index) {
      if (index < 0 || index >= static_cast<int8_t>(BUFFER_COUNT)) return;
      if (!takeMutex(portMAX_DELAY)) {
        _framesDropped++;
        return;
      }

      // Keep latency low: only the newest not-yet-consumed frame is retained.
      for (uint8_t i = 0; i < BUFFER_COUNT; i++) {
        if (i != static_cast<uint8_t>(index) && _bufferState[i] == BufferState::Queued) {
          _bufferState[i] = BufferState::Free;
          accountQueuedFrameDiscard();
        }
      }
      _bufferState[index] = BufferState::Queued;
      giveMutex();

      if (_ioTaskHandle) xTaskNotifyGive(_ioTaskHandle);
    }

    int8_t takeQueuedBuffer() {
      int8_t result = -1;
      if (!takeMutex(pdMS_TO_TICKS(5))) return result;

      for (uint8_t i = 0; i < BUFFER_COUNT; i++) {
        if (_bufferState[i] == BufferState::Queued) {
          result = static_cast<int8_t>(i);
          _bufferState[i] = BufferState::Active;
          break;
        }
      }
      giveMutex();
      return result;
    }

    void resetParser(bool releaseCurrentBuffer = true) {
      if (releaseCurrentBuffer && _rxFillBuffer >= 0) releaseBuffer(_rxFillBuffer);
      _rxState = RxState::FindMagic;
      _magicPos = 0;
      _rxHeaderPos = 0;
      _rxPayloadPos = 0;
      _rxPayloadLength = 0;
      _rxLedCount = 0;
      _rxSequence = 0;
      _rxBrightness = 0;
      _rxCrc = 0xFFFF;
      _rxReceivedCrc = 0;
      _rxFillBuffer = -1;
    }

    bool allocateRolePin() {
      if (_role == Role::Master) {
        if (_txPin < 0) {
          _lastError = F("Master TX pin is disabled");
          return false;
        }
        if (!PinManager::allocatePin(static_cast<byte>(_txPin), true, PinOwner::UM_Unspecified)) {
          _lastError = F("Master TX pin is already in use");
          return false;
        }
      } else {
        if (_rxPin < 0) {
          _lastError = F("Receiver RX pin is disabled");
          return false;
        }
        if (!PinManager::allocatePin(static_cast<byte>(_rxPin), false, PinOwner::UM_Unspecified)) {
          _lastError = F("Receiver RX pin is already in use");
          return false;
        }
      }
      _pinAllocated = true;
      return true;
    }

    void deallocateRolePin() {
      if (!_pinAllocated) return;
      if (_role == Role::Master && _txPin >= 0) {
        PinManager::deallocatePin(static_cast<byte>(_txPin), PinOwner::UM_Unspecified);
      } else if (_role == Role::Receiver && _rxPin >= 0) {
        PinManager::deallocatePin(static_cast<byte>(_rxPin), PinOwner::UM_Unspecified);
      }
      _pinAllocated = false;
    }

    bool allocateBuffers() {
      _payloadSize = static_cast<size_t>(_ledCount) * 3U;
      _frameSize = HEADER_SIZE + _payloadSize + CRC_SIZE;

      const size_t allocationSize = (_role == Role::Master) ? _frameSize : _payloadSize;
      for (uint8_t i = 0; i < BUFFER_COUNT; i++) {
        _buffers[i] = new (std::nothrow) uint8_t[allocationSize];
        if (!_buffers[i]) {
          _lastError = F("Unable to allocate frame buffers");
          return false;
        }
        memset(_buffers[i], 0, allocationSize);
        _bufferState[i] = BufferState::Free;
      }

      if (_role == Role::Receiver) {
        _lastAppliedPayload = new (std::nothrow) uint8_t[_payloadSize];
        if (!_lastAppliedPayload) {
          _lastError = F("Unable to allocate receiver failsafe buffer");
          return false;
        }
        memset(_lastAppliedPayload, 0, _payloadSize);
        _lastAppliedLedCount = 0;
      }
      return true;
    }

    void freeBuffers() {
      for (uint8_t i = 0; i < BUFFER_COUNT; i++) {
        delete[] _buffers[i];
        _buffers[i] = nullptr;
        _bufferState[i] = BufferState::Free;
      }
      delete[] _lastAppliedPayload;
      _lastAppliedPayload = nullptr;
      _lastAppliedLedCount = 0;
      _payloadSize = 0;
      _frameSize = 0;
    }

    bool startBridge() {
      _lastError = String();
      if (!_enabled) return false;
      if (_ledCount == 0 || _ledCount > MAX_LED_COUNT) {
        _lastError = F("Invalid LED count");
        return false;
      }
      if (_uartNumber < 1 || _uartNumber > 2) {
        _lastError = F("UART must be 1 or 2");
        return false;
      }

      _bufferMutex = xSemaphoreCreateMutex();
      if (!_bufferMutex) {
        _lastError = F("Unable to create buffer mutex");
        return false;
      }

      if (!allocateRolePin() || !allocateBuffers()) {
        stopBridge();
        return false;
      }

      _uart = new (std::nothrow) HardwareSerial(_uartNumber);
      if (!_uart) {
        _lastError = F("Unable to allocate UART");
        stopBridge();
        return false;
      }

      if (_role == Role::Receiver) {
        // Must be configured before begin(). Keep several complete UART frames
        // in the driver ring so a long LED show cannot starve the UART receiver.
        size_t desiredRxBuffer = _frameSize * 6U;
        if (desiredRxBuffer < 8192U) desiredRxBuffer = 8192U;
        if (desiredRxBuffer > 24576U) desiredRxBuffer = 24576U;
        _rxDriverBufferBytes = desiredRxBuffer;
        _uart->setRxBufferSize(desiredRxBuffer);
        _uart->setTimeout(5);
        _uart->begin(_baud, SERIAL_8N1, _rxPin, -1);
      } else {
        _uart->begin(_baud, SERIAL_8N1, -1, _txPin);
      }

      _frameIntervalUs = 1000000UL / (_maxFps == 0 ? 1U : _maxFps);
      _taskStop = false;
      resetParser(false);
      if (_role == Role::Receiver) _hasReceivedSequence = false;
      if (_statsStartedMs == 0U) _statsStartedMs = millis();

      const char* taskName = (_role == Role::Master) ? "WPB UART TX" : "WPB UART RX";
      // Keep the bridge task at the Arduino loop priority. The UART driver ISR
      // and enlarged RX buffer absorb bursts without starving WLED effects.
      const UBaseType_t priority = 1U;
      if (xTaskCreate(ioTaskEntry, taskName, 4096, this, priority, &_ioTaskHandle) != pdPASS) {
        _lastError = F("Unable to create UART task");
        stopBridge();
        return false;
      }

      _running = true;
      _bridgeStartedMs = millis();
      _firstValidFrameDelayMs = 0;
      _hasFirstValidFrameDelay = false;
      _receiverOwnsRealtime = false;
      _receiverHasAppliedFrame = false;
      _consecutiveValidFrames = 0;
      _fadeStartedMs = 0;
      _lastFailsafeShowUs = 0;
      if (_role == Role::Receiver) enterInitialWaitingState();
      return true;
    }

    void stopBridge() {
      if (_role == Role::Receiver && _receiverOwnsRealtime && realtimeMode == REALTIME_MODE_ADALIGHT) {
        exitRealtime();
      }
      _receiverOwnsRealtime = false;
      _running = false;
      _taskStop = true;

      if (_ioTaskHandle) {
        xTaskNotifyGive(_ioTaskHandle);
        const uint32_t waitStarted = millis();
        while (_ioTaskHandle && millis() - waitStarted < 50U) delay(1);
        if (_ioTaskHandle) {
          vTaskDelete(_ioTaskHandle);
          _ioTaskHandle = nullptr;
        }
      }

      if (_uart) {
        _uart->end();
        delete _uart;
        _uart = nullptr;
      }
      _rxDriverBufferBytes = 0;

      deallocateRolePin();
      freeBuffers();

      if (_bufferMutex) {
        vSemaphoreDelete(_bufferMutex);
        _bufferMutex = nullptr;
      }
      _taskStop = false;
    }

    void restartBridgeIfRequested() {
      if (!_restartRequested) return;
      _restartRequested = false;
      stopBridge();
      startBridge();
    }

    void buildMasterFrame(uint8_t* frame, uint32_t sequence) {
      memcpy(frame, MAGIC, MAGIC_SIZE);
      frame[4] = PROTOCOL_VERSION;
      frame[5] = _reverse ? 0x01U : 0x00U;
      writeLe32(frame + 6, sequence);
      writeLe16(frame + 10, _ledCount);
      frame[12] = bri;
      frame[13] = 0;
      writeLe16(frame + 14, static_cast<uint16_t>(_payloadSize));

      uint8_t* payload = frame + HEADER_SIZE;
      const uint16_t totalLength = strip.getLengthTotal();
      for (uint16_t destination = 0; destination < _ledCount; destination++) {
        const uint16_t logicalOffset = _reverse
          ? static_cast<uint16_t>(_ledCount - 1U - destination)
          : destination;
        const uint32_t logicalIndex = static_cast<uint32_t>(_sourceStart) + logicalOffset;
        uint32_t color = 0;
        if (logicalIndex < totalLength) color = strip.getPixelColor(static_cast<uint16_t>(logicalIndex));

        const uint8_t white = W(color);
        payload[destination * 3U + 0U] = saturatingAdd(R(color), white);
        payload[destination * 3U + 1U] = saturatingAdd(G(color), white);
        payload[destination * 3U + 2U] = saturatingAdd(B(color), white);
      }

      const uint16_t crc = crc16(frame + MAGIC_SIZE, HEADER_AFTER_MAGIC_SIZE + _payloadSize);
      writeLe16(frame + HEADER_SIZE + _payloadSize, crc);
    }

    void captureMasterFrame() {
      if (!_running || _role != Role::Master || !_uart) return;

      const uint32_t nowUs = micros();
      if (static_cast<uint32_t>(nowUs - _lastCaptureUs) < _frameIntervalUs) return;
      _lastCaptureUs = nowUs;

      const int8_t bufferIndex = acquireFreeBuffer(true);
      if (bufferIndex < 0) {
        _framesDropped++;
        return;
      }

      buildMasterFrame(_buffers[bufferIndex], _nextSequence++);
      _framesCaptured++;
      queueBuffer(bufferIndex);
    }

    void txTaskLoop() {
      while (!_taskStop) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (_taskStop) break;

        int8_t index;
        while ((index = takeQueuedBuffer()) >= 0) {
          const size_t written = _uart ? _uart->write(_buffers[index], _frameSize) : 0;
          if (written == _frameSize) {
            // write() may return after buffering. flush() ensures the buffer is no
            // longer referenced before it is returned to the renderer.
            _uart->flush();
            _framesSent++;
          } else {
            _uartWriteErrors++;
          }
          releaseBuffer(index);
        }
      }
    }

    void consumeMagicByte(uint8_t value) {
      if (value == MAGIC[_magicPos]) {
        _magicPos++;
        if (_magicPos == MAGIC_SIZE) {
          _rxState = RxState::ReadHeader;
          _rxHeaderPos = 0;
          _rxCrc = 0xFFFF;
          _magicPos = 0;
        }
      } else {
        _magicPos = (value == MAGIC[0]) ? 1U : 0U;
      }
    }

    bool parseHeader() {
      const uint8_t version = _rxHeader[0];
      _rxSequence = readLe32(_rxHeader + 2);
      _rxLedCount = readLe16(_rxHeader + 6);
      _rxBrightness = _rxHeader[8];
      _rxPayloadLength = readLe16(_rxHeader + 10);

      if (version != PROTOCOL_VERSION ||
          _rxLedCount == 0 ||
          _rxLedCount > _ledCount ||
          _rxPayloadLength != static_cast<uint16_t>(_rxLedCount * 3U) ||
          _rxPayloadLength > _payloadSize) {
        _framingErrors++;
        _consecutiveValidFrames = 0;
        return false;
      }

      _rxFillBuffer = acquireFreeBuffer(true);
      _rxPayloadPos = 0;
      return true;
    }

    void completeReceivedFrame() {
      if (_rxFillBuffer < 0) {
        _framesDropped++;
        resetParser(false);
        return;
      }

      if (_rxReceivedCrc != _rxCrc) {
        _crcErrors++;
        _consecutiveValidFrames = 0;
        resetParser(true);
        return;
      }

      const bool sequenceContinues = _hasReceivedSequence &&
        static_cast<uint32_t>(_rxSequence - _lastReceivedSequence) == 1U;
      _consecutiveValidFrames = sequenceContinues
        ? static_cast<uint8_t>(_consecutiveValidFrames < 255U ? _consecutiveValidFrames + 1U : 255U)
        : 1U;

      trackReceivedSequence(_rxSequence);
      _rxMeta[_rxFillBuffer].sequence = _rxSequence;
      _rxMeta[_rxFillBuffer].ledCount = _rxLedCount;
      _rxMeta[_rxFillBuffer].brightness = _rxBrightness;
      _framesReceived++;
      _lastValidFrameMs = millis();
      if (!_hasFirstValidFrameDelay) {
        _firstValidFrameDelayMs = static_cast<uint32_t>(_lastValidFrameMs - _bridgeStartedMs);
        _hasFirstValidFrameDelay = true;
      }

      const int8_t completedBuffer = _rxFillBuffer;
      _rxFillBuffer = -1;
      queueBuffer(completedBuffer);
      resetParser(false);
    }

    void consumeRxByte(uint8_t value) {
      switch (_rxState) {
        case RxState::FindMagic:
          consumeMagicByte(value);
          break;

        case RxState::ReadHeader:
          _rxHeader[_rxHeaderPos++] = value;
          _rxCrc = crc16Update(_rxCrc, value);
          if (_rxHeaderPos == HEADER_AFTER_MAGIC_SIZE) {
            if (parseHeader()) {
              _rxState = RxState::ReadPayload;
            } else {
              resetParser(true);
            }
          }
          break;

        case RxState::ReadPayload:
          if (_rxFillBuffer >= 0) _buffers[_rxFillBuffer][_rxPayloadPos] = value;
          _rxPayloadPos++;
          _rxCrc = crc16Update(_rxCrc, value);
          if (_rxPayloadPos >= _rxPayloadLength) _rxState = RxState::ReadCrcLow;
          break;

        case RxState::ReadCrcLow:
          _rxReceivedCrc = value;
          _rxState = RxState::ReadCrcHigh;
          break;

        case RxState::ReadCrcHigh:
          _rxReceivedCrc |= static_cast<uint16_t>(value) << 8;
          completeReceivedFrame();
          break;
      }
    }

    void rxTaskLoop() {
      uint8_t chunk[512];
      while (!_taskStop) {
        if (!_uart) break;
        const int available = _uart->available();
        if (available <= 0) {
          vTaskDelay(pdMS_TO_TICKS(1));
          continue;
        }

        const size_t wanted = static_cast<size_t>(available) > sizeof(chunk)
          ? sizeof(chunk)
          : static_cast<size_t>(available);
        const size_t received = _uart->readBytes(chunk, wanted);
        for (size_t i = 0; i < received; i++) consumeRxByte(chunk[i]);
      }
    }

    static void ioTaskEntry(void* argument) {
      UartPixelBridge* bridge = static_cast<UartPixelBridge*>(argument);
      if (bridge->_role == Role::Master) bridge->txTaskLoop();
      else bridge->rxTaskLoop();
      bridge->_ioTaskHandle = nullptr;
      vTaskDelete(nullptr);
    }

    void applyReceiverFrame() {
      if (!_running || _role != Role::Receiver) return;

      const int8_t bufferIndex = takeQueuedBuffer();
      if (bufferIndex < 0) return;

      const uint32_t applyStartedUs = micros();
      const RxMeta meta = _rxMeta[bufferIndex];

      if (!receiverMayApplyFrame()) {
        releaseBuffer(bufferIndex);
        return;
      }

      // Use the same realtime mode class as WLED's native serial/Adalight
      // receiver. REALTIME_MODE_GENERIC is intentionally NOT used here:
      // realtimeLock() performs an immediate strip.show() for GENERIC mode,
      // which would make every bridge frame perform two physical LED shows
      // (one in realtimeLock() and another after copying the received pixels).
      // With long WS281x strings that halves the attainable frame rate and
      // creates a periodic hitch as the RX cadence beats against the output
      // cadence. ADALIGHT refreshes the realtime timeout without issuing that
      // extra show, so each received frame results in at most one strip.show().
      refreshRealtimeOwnership();

      if (!realtimeOverride) {
        if (bri != meta.brightness) {
          bri = meta.brightness;
          strip.setBrightness(bri, true);
        }

        const uint16_t totalLength = strip.getLengthTotal();
        for (uint16_t i = 0; i < meta.ledCount; i++) {
          const uint32_t target = static_cast<uint32_t>(_targetStart) + i;
          if (target >= totalLength) break;
          const size_t offset = static_cast<size_t>(i) * 3U;
          const uint32_t color = RGBW32(
            _buffers[bufferIndex][offset],
            _buffers[bufferIndex][offset + 1U],
            _buffers[bufferIndex][offset + 2U],
            0
          );

          // Keep WLED's realtime framebuffer authoritative. Writing directly to
          // BusManager bypasses WS2812FX state and can cause visible flicker when
          // any normal WLED show path repaints the bus from its framebuffer.
          strip.setRealtimePixelColor(static_cast<uint16_t>(target), color);
        }

        if (_lastAppliedPayload) {
          memcpy(_lastAppliedPayload, _buffers[bufferIndex], static_cast<size_t>(meta.ledCount) * 3U);
          _lastAppliedLedCount = meta.ledCount;
        }

        const uint32_t showStartedUs = micros();
        strip.show();
        const uint32_t showElapsedUs = static_cast<uint32_t>(micros() - showStartedUs);
        const uint32_t applyElapsedUs = static_cast<uint32_t>(micros() - applyStartedUs);

        _showMicrosTotal += showElapsedUs;
        _applyMicrosTotal += applyElapsedUs;
        if (showElapsedUs > _showMicrosMax) _showMicrosMax = showElapsedUs;
        if (applyElapsedUs > _applyMicrosMax) _applyMicrosMax = applyElapsedUs;

        _framesApplied++;
        _lastAppliedSequence = meta.sequence;
        _receiverHasAppliedFrame = true;
        _receiverState = ReceiverState::Active;
        _fadeStartedMs = 0;
        _lastFailsafeShowUs = 0;
      }

      releaseBuffer(bufferIndex);
    }

    void validateSettings() {
      _fastReceiver = false; // direct BusManager output remains intentionally disabled
      if (_uartNumber < 1 || _uartNumber > 2) _uartNumber = 1;
      if (_baud < MIN_BAUD) _baud = MIN_BAUD;
      if (_baud > MAX_BAUD) _baud = MAX_BAUD;
      if (_ledCount == 0) _ledCount = 1;
      if (_ledCount > MAX_LED_COUNT) _ledCount = MAX_LED_COUNT;
      if (_maxFps == 0) _maxFps = 1;
      if (_maxFps > 120) _maxFps = 120;
      if (_realtimeTimeoutMs < 100) _realtimeTimeoutMs = 100;
      if (_realtimeTimeoutMs > 5000) _realtimeTimeoutMs = 5000;
      if (_startupValidFrames < 1) _startupValidFrames = 1;
      if (_startupValidFrames > 20) _startupValidFrames = 20;
      if (_reacquireValidFrames < 1) _reacquireValidFrames = 1;
      if (_reacquireValidFrames > 20) _reacquireValidFrames = 20;
      if (_failsafeHoldMs > 60000UL) _failsafeHoldMs = 60000UL;
      if (_failsafeFadeMs > 10000UL) _failsafeFadeMs = 10000UL;
      const uint8_t failsafeValue = static_cast<uint8_t>(_failsafeAction);
      if (failsafeValue > static_cast<uint8_t>(FailsafeAction::FadeToBlack)) {
        _failsafeAction = FailsafeAction::FadeToBlack;
      }
    }

  public:
    void setup() override {
      _initDone = true;
      validateSettings();
      _restartRequested = true;
    }

    void loop() override {
      restartBridgeIfRequested();
      if (_statsResetRequested) resetStatistics();
      if (_role == Role::Receiver) {
        applyReceiverFrame();
        handleReceiverSafety();
      }
    }

    void handleOverlayDraw() override {
      captureMasterFrame();
    }

    void readFromJsonState(JsonObject& root) override {
      if (!_initDone) return;
      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) return;
      if (top["reset-stats"] | false) _statsResetRequested = true;
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top["enabled"] = _enabled;
      top["role"] = static_cast<uint8_t>(_role);
      top["uart"] = _uartNumber;
      JsonArray pins = top.createNestedArray("pin");
      pins.add(_txPin);
      pins.add(_rxPin);
      top["baud"] = _baud;
      top["source-start"] = _sourceStart;
      top["target-start"] = _targetStart;
      top["led-count"] = _ledCount;
      top["max-fps"] = _maxFps;
      top["timeout-ms"] = _realtimeTimeoutMs;
      top["reverse"] = _reverse;
      top["startup-guard"] = _startupGuardEnabled;
      top["startup-valid-frames"] = _startupValidFrames;
      top["failsafe"] = _failsafeEnabled;
      top["failsafe-hold-ms"] = _failsafeHoldMs;
      top["failsafe-action"] = static_cast<uint8_t>(_failsafeAction);
      top["failsafe-fade-ms"] = _failsafeFadeMs;
      top["reacquire-guard"] = _reacquireGuardEnabled;
      top["reacquire-valid-frames"] = _reacquireValidFrames;
    }

    bool readFromConfig(JsonObject& root) override {
      // On a live settings save, stop while the old role, pins and buffer sizes
      // are still intact. This prevents a show callback or UART task from using
      // newly loaded settings with buffers allocated for the previous settings.
      const bool reconfigureLive = _initDone;
      if (reconfigureLive) stopBridge();

      JsonObject top = root[FPSTR(_name)];
      bool complete = !top.isNull();

      uint8_t roleValue = static_cast<uint8_t>(_role);
      uint8_t failsafeActionValue = static_cast<uint8_t>(_failsafeAction);
      complete &= getJsonValue(top["enabled"], _enabled, true);
      complete &= getJsonValue(top["role"], roleValue, 0);
      complete &= getJsonValue(top["uart"], _uartNumber, 1);
      complete &= getJsonValue(top["pin"][0], _txPin, 17);
      complete &= getJsonValue(top["pin"][1], _rxPin, 16);
      complete &= getJsonValue(top["baud"], _baud, 2000000UL);
      complete &= getJsonValue(top["source-start"], _sourceStart, 0);
      complete &= getJsonValue(top["target-start"], _targetStart, 0);
      complete &= getJsonValue(top["led-count"], _ledCount, 300);
      complete &= getJsonValue(top["max-fps"], _maxFps, 60);
      complete &= getJsonValue(top["timeout-ms"], _realtimeTimeoutMs, 500);
      complete &= getJsonValue(top["reverse"], _reverse, false);
      complete &= getJsonValue(top["startup-guard"], _startupGuardEnabled, true);
      complete &= getJsonValue(top["startup-valid-frames"], _startupValidFrames, 2);
      complete &= getJsonValue(top["failsafe"], _failsafeEnabled, true);
      complete &= getJsonValue(top["failsafe-hold-ms"], _failsafeHoldMs, 500UL);
      complete &= getJsonValue(top["failsafe-action"], failsafeActionValue, 2);
      complete &= getJsonValue(top["failsafe-fade-ms"], _failsafeFadeMs, 500UL);
      complete &= getJsonValue(top["reacquire-guard"], _reacquireGuardEnabled, true);
      complete &= getJsonValue(top["reacquire-valid-frames"], _reacquireValidFrames, 2);

      _role = roleValue == 1 ? Role::Receiver : Role::Master;
      _failsafeAction = failsafeActionValue <= 2
        ? static_cast<FailsafeAction>(failsafeActionValue)
        : FailsafeAction::FadeToBlack;
      validateSettings();
      if (reconfigureLive) _restartRequested = true;
      return complete;
    }

    void appendConfigData(Print& settingsScript) override {
      settingsScript.print(F("dd=addDropdown('UART Pixel Bridge','role');"));
      settingsScript.print(F("addOption(dd,'Master / sender',0);"));
      settingsScript.print(F("addOption(dd,'Receiver',1);"));
      settingsScript.print(F("dd=addDropdown('UART Pixel Bridge','uart');"));
      settingsScript.print(F("addOption(dd,'UART 1',1);"));
      settingsScript.print(F("addOption(dd,'UART 2',2);"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:pin:0',1,'TX pin; used only by Master');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:pin:1',1,'RX pin; used only by Receiver');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:source-start',1,'First logical LED copied by Master');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:target-start',1,'First physical LED written by Receiver');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:led-count',1,'Must match on both controllers');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:reverse',1,'Reverse the transmitted logical range on Master');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:timeout-ms',1,'WLED realtime timeout; failsafe refreshes it while active');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:startup-guard',1,'Receiver only: keep controlled LEDs black until enough consecutive valid frames arrive');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:startup-valid-frames',1,'Receiver only: 1-20 consecutive valid frames required before first output');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:failsafe',1,'Receiver only: own output behavior when Master frames stop');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:failsafe-hold-ms',1,'Receiver only: keep last frame this long before failsafe action; 0-60000 ms');"));
      settingsScript.print(F("dd=addDropdown('UART Pixel Bridge','failsafe-action');"));
      settingsScript.print(F("addOption(dd,'Keep last frame',0);"));
      settingsScript.print(F("addOption(dd,'Black after hold',1);"));
      settingsScript.print(F("addOption(dd,'Fade to black after hold',2);"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:failsafe-fade-ms',1,'Receiver only: fade duration, 0-10000 ms');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:reacquire-guard',1,'Receiver only: require stable frames again after a failsafe event');"));
      settingsScript.print(F("addInfo('UART Pixel Bridge:reacquire-valid-frames',1,'Receiver only: 1-20 consecutive valid frames required to resume');"));
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray status = user.createNestedArray(FPSTR(_name));
      if (!_enabled) status.add(F("disabled"));
      else if (!_running) status.add(_lastError.length() ? _lastError : String(F("stopped")));
      else if (_role == Role::Master) status.add(F("Master TX"));
      else {
        if (_receiverState == ReceiverState::Active) status.add(F("Receiver active"));
        else if (_receiverState == ReceiverState::WaitingForMaster) status.add(F("Receiver waiting"));
        else if (_receiverState == ReceiverState::Holding) status.add(F("Receiver holding"));
        else if (_receiverState == ReceiverState::Fading) status.add(F("Receiver fading"));
        else status.add(F("Receiver blackout"));
      }

      JsonArray frames = user.createNestedArray(F("UART bridge frames"));
      frames.add(_role == Role::Master ? _framesSent : _framesApplied);
      frames.add(_role == Role::Master ? F(" sent") : F(" applied"));

      JsonArray dropped = user.createNestedArray(F("UART bridge dropped"));
      dropped.add(_framesDropped);
      dropped.add(F(" frames"));

      if (_role == Role::Master) {
        JsonArray writeErrors = user.createNestedArray(F("UART write errors"));
        writeErrors.add(_uartWriteErrors);
        writeErrors.add(F(" errors"));
      } else {
        JsonArray received = user.createNestedArray(F("UART valid received"));
        received.add(_framesReceived);
        received.add(F(" frames"));

        JsonArray crc = user.createNestedArray(F("UART CRC errors"));
        crc.add(_crcErrors);
        crc.add(F(" frames"));

        JsonArray framing = user.createNestedArray(F("UART framing errors"));
        framing.add(_framingErrors);
        framing.add(F(" errors"));

        JsonArray missing = user.createNestedArray(F("UART missing sequences"));
        missing.add(_missingSequences);
        missing.add(F(" frames"));

        JsonArray gaps = user.createNestedArray(F("UART sequence gap events"));
        gaps.add(_sequenceGapEvents);
        gaps.add(F(" events"));

        JsonArray duplicates = user.createNestedArray(F("UART duplicate sequences"));
        duplicates.add(_duplicateSequences);
        duplicates.add(F(" frames"));

        JsonArray resyncs = user.createNestedArray(F("UART sequence resyncs"));
        resyncs.add(_sequenceResyncs);
        resyncs.add(F(" events"));

        JsonArray crcRate = user.createNestedArray(F("UART CRC error rate"));
        crcRate.add(String(crcErrorRatePercent(), 6));
        crcRate.add(F(" %"));

        JsonArray lossRate = user.createNestedArray(F("UART sequence loss rate"));
        lossRate.add(String(sequenceLossRatePercent(), 6));
        lossRate.add(F(" %"));

        JsonArray lastRx = user.createNestedArray(F("UART last RX sequence"));
        lastRx.add(_lastReceivedSequence);

        JsonArray lastApplied = user.createNestedArray(F("UART last applied sequence"));
        lastApplied.add(_lastAppliedSequence);
      }

      if (_role == Role::Receiver) {
        JsonArray receiverState = user.createNestedArray(F("UART receiver state"));
        receiverState.add(receiverStateName());

        JsonArray firstFrame = user.createNestedArray(F("UART first valid frame"));
        firstFrame.add(_hasFirstValidFrameDelay ? _firstValidFrameDelayMs : 0U);
        firstFrame.add(F(" ms after bridge start"));

        JsonArray lastAge = user.createNestedArray(F("UART last frame age"));
        lastAge.add(_lastValidFrameMs ? static_cast<uint32_t>(millis() - _lastValidFrameMs) : 0U);
        lastAge.add(F(" ms"));

        JsonArray failsafeEvents = user.createNestedArray(F("UART failsafe events"));
        failsafeEvents.add(_failsafeEvents);
        failsafeEvents.add(F(" events"));
      }

      JsonArray mode = user.createNestedArray(F("UART receiver output"));
      mode.add(F("WLED serial realtime (single-show)"));

      if (_role == Role::Receiver) {
        JsonArray fps = user.createNestedArray(F("UART applied FPS"));
        fps.add(String(appliedFps(), 2));
        fps.add(F(" fps"));

        JsonArray applyAvg = user.createNestedArray(F("UART apply avg"));
        applyAvg.add(averageApplyMicros());
        applyAvg.add(F(" us"));

        JsonArray applyMax = user.createNestedArray(F("UART apply max"));
        applyMax.add(_applyMicrosMax);
        applyMax.add(F(" us"));

        JsonArray showAvg = user.createNestedArray(F("UART show avg"));
        showAvg.add(averageShowMicros());
        showAvg.add(F(" us"));

        JsonArray rxbuf = user.createNestedArray(F("UART RX driver buffer"));
        rxbuf.add(static_cast<uint32_t>(_rxDriverBufferBytes));
        rxbuf.add(F(" bytes"));
      }

      JsonArray age = user.createNestedArray(F("UART stats age"));
      age.add(_statsStartedMs ? static_cast<uint32_t>(millis() - _statsStartedMs) / 1000U : 0U);
      age.add(F(" s"));
    }

    void addToJsonState(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) top = root.createNestedObject(FPSTR(_name));
      top["bridge-version"] = F("1.4.0");
      top["running"] = _running;
      top["role"] = static_cast<uint8_t>(_role);
      top["captured"] = _framesCaptured;
      top["sent"] = _framesSent;
      top["received"] = _framesReceived;
      top["applied"] = _framesApplied;
      top["dropped"] = _framesDropped;
      top["crc-errors"] = _crcErrors;
      top["framing-errors"] = _framingErrors;
      top["uart-write-errors"] = _uartWriteErrors;
      top["sequence-gap-events"] = _sequenceGapEvents;
      top["missing-sequences"] = _missingSequences;
      top["duplicate-sequences"] = _duplicateSequences;
      top["sequence-resyncs"] = _sequenceResyncs;
      top["crc-error-rate-pct"] = crcErrorRatePercent();
      top["sequence-loss-rate-pct"] = sequenceLossRatePercent();
      top["last-received-sequence"] = _lastReceivedSequence;
      top["last-applied-sequence"] = _lastAppliedSequence;
      top["last-sequence"] = _lastAppliedSequence; // backwards compatibility
      top["stats-age-ms"] = _statsStartedMs ? static_cast<uint32_t>(millis() - _statsStartedMs) : 0U;
      top["fast-receiver"] = false; // direct-bus mode disabled due framebuffer desynchronization
      top["receiver-output"] = F("wled-adalight-realtime");
      top["applied-fps"] = appliedFps();
      top["apply-us-avg"] = averageApplyMicros();
      top["apply-us-max"] = _applyMicrosMax;
      top["show-us-avg"] = averageShowMicros();
      top["show-us-max"] = _showMicrosMax;
      top["rx-driver-buffer-bytes"] = static_cast<uint32_t>(_rxDriverBufferBytes);
      top["receiver-state"] = receiverStateName();
      top["startup-guard-enabled"] = _startupGuardEnabled;
      top["startup-valid-frames"] = _startupValidFrames;
      top["failsafe-enabled"] = _failsafeEnabled;
      top["failsafe-hold-ms"] = _failsafeHoldMs;
      top["failsafe-action"] = failsafeActionName();
      top["failsafe-fade-ms"] = _failsafeFadeMs;
      top["reacquire-guard-enabled"] = _reacquireGuardEnabled;
      top["reacquire-valid-frames"] = _reacquireValidFrames;
      top["consecutive-valid-frames"] = _consecutiveValidFrames;
      top["failsafe-events"] = _failsafeEvents;
      top["startup-frames-suppressed"] = _startupFramesSuppressed;
      top["reacquire-frames-suppressed"] = _reacquireFramesSuppressed;
      top["failsafe-shows"] = _failsafeShows;
      top["first-valid-frame-ms"] = _hasFirstValidFrameDelay ? _firstValidFrameDelayMs : 0U;
      top["last-frame-age-ms"] = _lastValidFrameMs ? static_cast<uint32_t>(millis() - _lastValidFrameMs) : 0U;
    }

    uint16_t getId() override {
      // A private ID avoids requiring a modification to WLED's const.h.
      return 0xA501;
    }
};

const char UartPixelBridge::_name[] PROGMEM = "UART Pixel Bridge";
constexpr uint8_t UartPixelBridge::MAGIC[4];

static UartPixelBridge uartPixelBridge;
REGISTER_USERMOD(uartPixelBridge);

#else

// Deliberately fail early if somebody adds this usermod to an ESP8266 build.
#error "UART Pixel Bridge requires an ESP32-family target."

#endif
