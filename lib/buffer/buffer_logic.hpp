#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "tiny_printf.hpp"

#ifndef BUFFER_ID
#define BUFFER_ID 0
#endif

#if BUFFER_ID < 0 || BUFFER_ID > 9
#error "BUFFER_ID must be between 0 and 9"
#endif

constexpr uint32_t CMD_TIMEOUT = 3000;
constexpr uint32_t SHORT_PRESS_MS = 150;
constexpr uint32_t MULTI_PRESS_MIN_MS = 50;
constexpr uint32_t MULTI_PRESS_MAX_MS = 500;

constexpr uint32_t DEFAULT_TIMEOUT_MS = 90000;
constexpr uint32_t DEFAULT_HOLD_TIMEOUT_MS = 10000;
constexpr uint8_t DEFAULT_MULTI_PRESS_COUNT = 2;

constexpr float DEFAULT_SPEED_MM_S = 45.0F;
constexpr uint32_t DEFAULT_EMPTYING_PUSH_TIMEOUT_MS = 2500;

// Firmware-side prestage defaults.
// LEVEL=75 maps to the existing normal-high fill state.
constexpr uint8_t DEFAULT_PRESTAGE_LEVEL = 75;
constexpr float DEFAULT_PRESTAGE_MAX_TRAVEL_MM = 35.0F;

constexpr size_t UART_CMD_BUF_SIZE = 64;

#ifdef ENABLE_I2C_PROTOCOL
enum I2CRegister : uint8_t {
  REG_COMMAND = 0x00, // 1 byte
  REG_MOVE_DIST = 0x01, // 4 bytes (float)
  REG_STATUS = 0x05, // 1 byte
  REG_MODE = 0x06, // 1 byte
  REG_MOTOR = 0x07, // 1 byte
  REG_PARAM_SPEED = 0x08, // 4 bytes (float)
  REG_PARAM_TIMEOUT = 0x0C, // 4 bytes (uint32_t)
  REG_PARAM_EMPTYING_TIMEOUT = 0x10, // 4 bytes (uint32_t)
  REG_PARAM_HOLD_TIMEOUT = 0x14, // 4 bytes (uint32_t)
  REG_PARAM_HOLD_TIMEOUT_ENABLED = 0x18, // 1 byte
  REG_PARAM_MULTI_PRESS_COUNT = 0x19, // 1 byte

  // Local read-only buffer status extension.
  // REG_SENSOR_BITS exposes raw optical sensor bits from BufferHardware.
  // REG_FILL_STATE exposes the interpreted buffer range state.
  REG_SENSOR_BITS = 0x1A, // 1 byte
  REG_FILL_STATE = 0x1B, // 1 byte

  // User-facing prestage target level, 0-100.
  // Host-side commands may present text aliases, but firmware stores the numeric level.
  REG_PARAM_PRESTAGE_LEVEL = 0x1C, // 1 byte
};

enum I2CCommand : uint8_t {
  CMD_OFF = 0,
  CMD_REGULAR = 1,
  CMD_HOLD = 2,
  CMD_PUSH = 3,
  CMD_RETRACT = 4,
  CMD_PRESTAGE = 5,
  CMD_REBOOT_DFU = 0xDF,
};
#endif

template<typename T>
void reinterpret_assign(T &dest, const uint8_t *src) {
  std::memcpy(&dest, src, sizeof(T));
}

template<class HW>
class Buffer {
public:
  enum class Mode : uint8_t {
    Regular = 0,
    Continuous,
    MoveCommand,
    Hold,
    Manual,
    Emptying,
    Prestage
  };

private:
  enum class Motor : uint8_t {
    Off = 0,
    Push,
    Retract,
    Hold,
  };

  static uint8_t levelToFillRank(const uint8_t level) {
    // Convert a user-facing 0-100 level into the nearest known fill-state rank.
    // These ranks match the existing BufferFillState enum values provided by the
    // hardware implementation:
    //   0 = low
    //   1 = normal-low
    //   2 = normal-mid
    //   3 = normal-high
    //   4 = over-full
    if (level <= 12) {
      return 0;
    }

    if (level <= 37) {
      return 1;
    }

    if (level <= 62) {
      return 2;
    }

    if (level <= 87) {
      return 3;
    }

    return 4;
  }

  struct ButtonState {
    bool pressed{ false };
    uint32_t pressStart{ 0 };
    uint32_t lastRelease{ 0 };
    uint8_t count{ 0 };
  };

  HW hw;
  Mode mode{ Mode::Regular };
  Motor motor{ Motor::Off };

  uint32_t timeoutMs{ DEFAULT_TIMEOUT_MS };
  uint32_t holdTimeoutMs{ DEFAULT_HOLD_TIMEOUT_MS };
  bool holdTimeoutEnabled{ false };
  uint8_t multiPressCount{ DEFAULT_MULTI_PRESS_COUNT };
  float speedMmS{ DEFAULT_SPEED_MM_S };
  uint32_t emptyingPushTimeoutMs{ DEFAULT_EMPTYING_PUSH_TIMEOUT_MS };

  uint8_t prestageLevel{ DEFAULT_PRESTAGE_LEVEL };

  bool filamentPresent{ false };
  bool timedOut{ false };

  ButtonState btnFwd;
  ButtonState btnBack;

  char cmdBuf[UART_CMD_BUF_SIZE]{};
  size_t cmdLen{ 0 };
  uint32_t lastCharTime{ 0 };

  uint32_t moveEnd{ 0 };
  Motor moveDir{ Motor::Off };

  uint32_t moveStart{ 0 };
  uint32_t holdStart{ 0 };
  uint32_t continuousStart{ 0 };
  uint32_t emptyingStart{ 0 };
  uint32_t emptyingPushStart{ 0 };

  // Prestage mode state.
  // prestageLevel is the user-facing 0-100 target.
  // prestageTargetRank is the mapped sensor-state rank used by the movement logic.
  uint8_t prestageTargetRank{ levelToFillRank(DEFAULT_PRESTAGE_LEVEL) };
  float prestageTravelMm{ 0.0F };
  uint32_t prestageLastTick{ 0 };

  Mode lastMode{ Mode::Regular };
  Motor lastMotor{ Motor::Off };
  bool lastFilament{ false };
  bool lastTimedOut{ false };

  uint32_t lastTimeoutMs{ 0 };
  uint32_t lastHoldTimeoutMs{ 0 };
  bool lastHoldTimeoutEnabled{ false };
  uint32_t lastMultiPressCount{ 0 };
  float lastSpeedMmS{ 0.0F };
  uint32_t lastEmptyingPushTimeoutMs{ 0 };
  uint32_t lastPrestageLevel{ 0 };

#ifdef ENABLE_I2C_PROTOCOL
  // Staged I2C byte writes let CP2112 write float registers using reliable
  // 2-byte transactions: [register_offset, one_payload_byte].
  // Avoids failing 5-byte write transaction while preserving
  // the original full-register write path for hosts that support it.
  uint8_t i2cMoveDistBytes[sizeof(float)]{};
  uint8_t i2cSpeedBytes[sizeof(float)]{};
  uint8_t i2cMoveDistMask{ 0 };
  uint8_t i2cSpeedMask{ 0 };
#endif

public:
  Buffer() = default;

  void init() {
    hw.initHardware();
    filamentPresent = hw.filamentPresent();
    lastFilament = filamentPresent;
    setMode(Mode::Regular);
    setMotor(filamentPresent ? Motor::Hold : Motor::Off);
    lastMotor = motor;
    hw.setPresenceLed(filamentPresent);
    hw.setPresenceOutput(filamentPresent);
    updateStatus(true);

#ifdef ENABLE_I2C_PROTOCOL
    hw.setI2CCallbacks(this, onI2CReadTrampoline, onI2CWriteTrampoline);
#endif
  }

  Mode getMode() const { return mode; }

  void loop() {
    hw.loop();

#ifdef ENABLE_UART_PROTOCOL
    processUartCommands();
#endif

    lastFilament = filamentPresent;
    filamentPresent = hw.filamentPresent();
    if (filamentPresent != lastFilament) {
      hw.setPresenceLed(filamentPresent);
      hw.setPresenceOutput(filamentPresent);
    }

    handleButtons();

    switch (mode) {
    case Mode::Regular:
      handleRegular();
      break;
    case Mode::Emptying:
      handleEmptying();
      break;
    case Mode::MoveCommand:
      handleMoveCommand();
      break;
    case Mode::Continuous:
      handleContinuous();
      break;
    case Mode::Hold:
      setMotor(Motor::Hold);
      break;
    case Mode::Manual:
      break;
    case Mode::Prestage:
      handlePrestage();
      break;
    }

    updateHoldTimeout();
    updateStatus();
  }

#ifdef UNIT_TEST
  HW &getHardware() { return hw; }
#endif

private:
#ifdef ENABLE_UART_PROTOCOL
  void processUartCommands() {
    char c = '\0';
    while (hw.readChar(c)) {
      if (c == '\r') {
        continue;
      }
      lastCharTime = hw.timeMs();
      if (c == '\n') {
        if (cmdLen > 0) {
          cmdBuf[cmdLen] = '\0';

          if (cmdBuf[0] >= '0' && cmdBuf[0] <= '9') {
            // A specific buffer is being addressed: check if it matches ours.
            if (const int bufID = cmdBuf[0] - '0'; bufID != hw.getBufferID()) {
              cmdLen = 0;
              continue;
            }
          }

          handleUartCommand(cmdBuf);
          cmdLen = 0;
        }
      } else {
        if (cmdLen < sizeof(cmdBuf) - 1) {
          cmdBuf[cmdLen++] = c;
        }
      }
    }

    if (cmdLen > 0 && hw.timeMs() - lastCharTime > CMD_TIMEOUT) {
      cmdLen = 0;
    }
  }

  static const char *startsWith(const char *str, ...) {
    va_list args;
    va_start(args, str);
    size_t offset = 0;
    while (const char *prefix = va_arg(args, const char *)) {
      const size_t len = std::strlen(prefix);
      if (std::strncmp(str + offset, prefix, len) != 0) {
        va_end(args);
        return nullptr;
      }
      offset += len;
    }
    va_end(args);
    return str + offset;
  }

  void handleUartCommand(const char *cmd) {
    const char *arg = nullptr;

    if (strcmp(cmd, "push") == 0 || strcmp(cmd, "p") == 0) {
      if (!hw.filamentPresent())
        return;
      setMode(Mode::Continuous);
      setMotor(Motor::Push);
    } else if (strcmp(cmd, "retract") == 0 || strcmp(cmd, "r") == 0) {
      if (!hw.filamentPresent())
        return;
      setMode(Mode::Continuous);
      setMotor(Motor::Retract);
    } else if (strcmp(cmd, "hold") == 0 || strcmp(cmd, "h") == 0) {
      holdTimeoutEnabled = false;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
    } else if (strcmp(cmd, "regular") == 0 || strcmp(cmd, "n") == 0) {
      setMode(Mode::Regular);
      setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
    } else if (strcmp(cmd, "off") == 0 || strcmp(cmd, "o") == 0) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
    } else if (strcmp(cmd, "query") == 0 || strcmp(cmd, "q") == 0) {
      updateStatus(true);
    } else if (strcmp(cmd, "prestage") == 0) {
      startPrestage(prestageLevel);
    } else if ((arg = startsWith(cmd, "prestage ", nullptr))) {
      const uint32_t requestedLevel = tiny::strtoul(arg);
      startPrestage(static_cast<uint8_t>(requestedLevel > 100 ? 100 : requestedLevel));
    } else if (strcmp(cmd, "reboot_dfu") == 0) {
      hw.writeLineF("mode=dfu");
      HW::rebootDFU();
    } else if (((arg = startsWith(cmd, "move ", nullptr))) || ((arg = startsWith(cmd, "m ", nullptr)))) {
      if (const float val = tiny::strtof(arg); val != 0.0F && speedMmS > 0.0F) {
        moveDir = val > 0 ? Motor::Push : Motor::Retract;
        const float ms = tiny::abs(val) * 1000.0F / speedMmS;
        moveEnd = hw.timeMs() + static_cast<uint32_t>(ms);
        setMode(Mode::MoveCommand);
        setMotor(moveDir);
      }
    } else if ((arg = startsWith(cmd, "set_", "timeout", " ", nullptr))) {
      timeoutMs = tiny::strtoul(arg);
    } else if ((arg = startsWith(cmd, "set_", "hold_", "timeout", nullptr))) {
      const char *oldArg = arg;
      if ((arg = startsWith(arg, "_en ", nullptr))) {
        holdTimeoutEnabled = static_cast<bool>(tiny::strtoul(arg));
      } else {
        holdTimeoutMs = tiny::strtoul(oldArg);
      }
    } else if ((arg = startsWith(cmd, "set_", "multi_press_count", " ", nullptr))) {
      multiPressCount = static_cast<uint8_t>(tiny::strtoul(arg));
    } else if ((arg = startsWith(cmd, "set_", "speed", " ", nullptr))) {
      speedMmS = tiny::strtof(arg);
    } else if ((arg = startsWith(cmd, "set_", "emptying_", "timeout", " ", nullptr))) {
      emptyingPushTimeoutMs = tiny::strtoul(arg);
    }

    updateStatus();
  }
#endif

#ifdef ENABLE_I2C_PROTOCOL
  static size_t onI2CReadTrampoline(void *ctx, const uint8_t reg) {
    if (auto *self = static_cast<Buffer *>(ctx))
      return self->onI2CRead(reg);
    return 0;
  }

  static void onI2CWriteTrampoline(void *ctx, const uint8_t reg, const size_t size, const uint8_t *data) {
    if (auto *self = static_cast<Buffer *>(ctx))
      self->onI2CWrite(reg, size, data);
  }

  size_t onI2CRead(const uint8_t reg) {
    if (reg == REG_STATUS)
      hw.setInterrupt(false);

    switch (reg) {
    case REG_STATUS: {
      uint8_t status = 0;
      if (filamentPresent)
        status |= 0x01;
      if (timedOut)
        status |= 0x02;
      return hw.i2cWrite(status);
    }
    case REG_MODE:
      return hw.i2cWrite(static_cast<uint8_t>(mode));
    case REG_MOTOR:
      return hw.i2cWrite(static_cast<uint8_t>(motor));
    case REG_PARAM_SPEED:
      return hw.i2cWriteValue(speedMmS);
    case REG_PARAM_TIMEOUT:
      return hw.i2cWriteValue(timeoutMs);
    case REG_PARAM_EMPTYING_TIMEOUT:
      return hw.i2cWriteValue(emptyingPushTimeoutMs);
    case REG_PARAM_HOLD_TIMEOUT:
      return hw.i2cWriteValue(holdTimeoutMs);
    case REG_PARAM_HOLD_TIMEOUT_ENABLED:
      return hw.i2cWrite(holdTimeoutEnabled ? 1 : 0);
    case REG_PARAM_MULTI_PRESS_COUNT:
      return hw.i2cWrite(multiPressCount);
    case REG_SENSOR_BITS:
      return hw.i2cWrite(hw.getSensorBits());
    case REG_FILL_STATE:
      return hw.i2cWrite(static_cast<uint8_t>(hw.getFillState()));
    case REG_PARAM_PRESTAGE_LEVEL:
      return hw.i2cWrite(prestageLevel);
    default:
      return hw.i2cWrite(0);
    }
  }

  void applyI2CMoveDistance(const float dist) {
    // Positive distance pushes/feeds. Negative distance retracts/pulls.
    // The move duration is still calculated by firmware using speedMmS.
    if (dist == 0.0F || speedMmS <= 0.0F) {
      return;
    }

    moveDir = dist > 0 ? Motor::Push : Motor::Retract;

    const float ms = tiny::abs(dist) * 1000.0F / speedMmS;
    moveEnd = hw.timeMs() + static_cast<uint32_t>(ms);

    setMode(Mode::MoveCommand);
    setMotor(moveDir);
    updateStatus();
  }

  bool handleI2CStagedWrite(const uint8_t reg, const uint8_t value) {
    // MOVE_DIST lives at 0x01..0x04 as a 4-byte little-endian float.
    // Commit the move only after all four bytes have been received.
    if (reg >= REG_MOVE_DIST && reg < REG_MOVE_DIST + sizeof(float)) {
      const uint8_t offset = reg - REG_MOVE_DIST;

      i2cMoveDistBytes[offset] = value;
      i2cMoveDistMask |= static_cast<uint8_t>(1U << offset);

      if (i2cMoveDistMask == 0x0F) {
        float dist = 0.0F;
        std::memcpy(&dist, i2cMoveDistBytes, sizeof(dist));

        i2cMoveDistMask = 0;
        applyI2CMoveDistance(dist);
      }

      return true;
    }

    // SPEED lives at 0x08..0x0B as a 4-byte little-endian float.
    // Commit the new speed only after all four bytes have been received.
    if (reg >= REG_PARAM_SPEED && reg < REG_PARAM_SPEED + sizeof(float)) {
      const uint8_t offset = reg - REG_PARAM_SPEED;

      i2cSpeedBytes[offset] = value;
      i2cSpeedMask |= static_cast<uint8_t>(1U << offset);

      if (i2cSpeedMask == 0x0F) {
        float newSpeed = 0.0F;
        std::memcpy(&newSpeed, i2cSpeedBytes, sizeof(newSpeed));

        i2cSpeedMask = 0;

        if (newSpeed > 0.0F) {
          speedMmS = newSpeed;
          updateStatus();
        }
      }

      return true;
    }

    return false;
  }

  void onI2CWrite(const uint8_t reg, const size_t size, const uint8_t *data) {
    if (size == 0) {
      return;
    }

    // CP2112 currently fails on 5-byte writes, so allow staged 1-byte writes
    // to float registers while preserving the original full-payload protocol.
    if (size == 1 && handleI2CStagedWrite(reg, data[0])) {
      return;
    }

    switch (reg) {
    case REG_COMMAND:
      handleI2CCommand(data[0]);
      break;

    case REG_MOVE_DIST:
      if (size >= sizeof(float)) {
        float dist = 0.0F;
        reinterpret_assign(dist, data);
        applyI2CMoveDistance(dist);
      }
      break;

    case REG_PARAM_SPEED:
      if (size >= sizeof(speedMmS)) {
        reinterpret_assign(speedMmS, data);
        updateStatus();
      }
      break;

    case REG_PARAM_TIMEOUT:
      if (size >= sizeof(timeoutMs)) {
        reinterpret_assign(timeoutMs, data);
        updateStatus();
      }
      break;

    case REG_PARAM_EMPTYING_TIMEOUT:
      if (size >= sizeof(emptyingPushTimeoutMs)) {
        reinterpret_assign(emptyingPushTimeoutMs, data);
        updateStatus();
      }
      break;

    case REG_PARAM_HOLD_TIMEOUT:
      if (size >= sizeof(holdTimeoutMs)) {
        reinterpret_assign(holdTimeoutMs, data);
        updateStatus();
      }
      break;

    case REG_PARAM_HOLD_TIMEOUT_ENABLED:
      if (size >= 1) {
        holdTimeoutEnabled = data[0] != 0;
        updateStatus();
      }
      break;

    case REG_PARAM_MULTI_PRESS_COUNT:
      if (size >= sizeof(multiPressCount)) {
        reinterpret_assign(multiPressCount, data);
        updateStatus();
      }
      break;

    case REG_PARAM_PRESTAGE_LEVEL:
      if (size >= 1) {
        prestageLevel = data[0] > 100 ? 100 : data[0];
        updateStatus();
      }
      break;

    default:
      // Unknown register, ignore.
      break;
    }
  }

  void handleI2CCommand(const uint8_t cmd) {
    switch (cmd) {
    case CMD_PUSH:
      if (!hw.filamentPresent())
        break;
      setMode(Mode::Continuous);
      setMotor(Motor::Push);
      break;
    case CMD_RETRACT:
      if (!hw.filamentPresent())
        break;
      setMode(Mode::Continuous);
      setMotor(Motor::Retract);
      break;
    case CMD_PRESTAGE:
      startPrestage(prestageLevel);
      break;
    case CMD_HOLD:
      holdTimeoutEnabled = false;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
      break;
    case CMD_REGULAR:
      setMode(Mode::Regular);
      setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
      break;
    case CMD_OFF:
      setMode(Mode::Regular);
      setMotor(Motor::Off);
      break;
    case CMD_REBOOT_DFU:
      hw.writeLineF("mode=dfu");
      HW::rebootDFU();
      break;
    default:
      // Unknown command, ignore.
      break;
    }

    updateStatus();
  }
#endif

  void doHandleButton(bool pressed, Motor dir, ButtonState &s, uint32_t now) {
    if (pressed) {
      if (!s.pressed) {
        s.pressed = true;
        s.pressStart = now;
        setMode(Mode::Manual);
        setMotor(dir);
      }
      return;
    }

    if (!s.pressed) {
      return;
    }

    const uint32_t dur = now - s.pressStart;
    s.pressed = false;

    if (dur <= SHORT_PRESS_MS) {
      if (now - s.lastRelease >= MULTI_PRESS_MIN_MS && now - s.lastRelease <= MULTI_PRESS_MAX_MS) {
        ++s.count;
      } else {
        s.count = 1;
      }

      s.lastRelease = now;

      if (s.count >= multiPressCount) {
        if (hw.filamentPresent()) {
          setMode(Mode::Continuous);
          setMotor(dir);
        }
        s.count = 0;
        return;
      }

      holdTimeoutEnabled = false;
      if (hw.filamentPresent()) {
        setMode(Mode::Hold);
        setMotor(Motor::Hold);
      } else {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      }
    } else {
      s.count = 0;
      if (hw.filamentPresent()) {
        setMode(Mode::Regular);
        setMotor(Motor::Hold);
      } else {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      }
    }
  }

  void handleButtons() {
    const uint32_t now = hw.timeMs();
    doHandleButton(hw.buttonForward(), Motor::Push, btnFwd, now);
    doHandleButton(hw.buttonBackward(), Motor::Retract, btnBack, now);
  }

  void startPrestage(const uint8_t requestedLevel) {
    // Clamp the requested target to a real 0-100 percentage before mapping it
    // to the nearest known fill-state rank.
    prestageLevel = requestedLevel > 100 ? 100 : requestedLevel;
    prestageTargetRank = levelToFillRank(prestageLevel);

    // Reset safety tracking for this prestage run.
    prestageTravelMm = 0.0F;
    prestageLastTick = hw.timeMs();
    timedOut = false;

    setMode(Mode::Prestage);

    // Evaluate immediately so command responses are not delayed until the next
    // loop pass.
    handlePrestage();

    updateStatus();
  }

  void handlePrestage() {
    const uint32_t now = hw.timeMs();

    // Track approximate travel while the motor is actually moving.
    // This is only a safety cap; the final target is still sensor-state based.
    if (motor == Motor::Push || motor == Motor::Retract) {
      const uint32_t deltaMs = now - prestageLastTick;
      prestageTravelMm += speedMmS * (static_cast<float>(deltaMs) / 1000.0F);
    }

    prestageLastTick = now;

    if (!hw.filamentPresent()) {
      // Do not prestage an empty filament path.
      timedOut = true;
      setMode(Mode::Regular);
      setMotor(Motor::Off);
      return;
    }

    if (prestageTravelMm >= DEFAULT_PRESTAGE_MAX_TRAVEL_MM) {
      // Stop before a bad sensor state or bad command can run the buffer too far.
      timedOut = true;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
      return;
    }

    const uint8_t currentRank = static_cast<uint8_t>(hw.getFillState());

    if (currentRank > 4) {
      // The existing hardware implementation reports unknown as 5.
      // Unknown states are not safe to chase.
      timedOut = true;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
      return;
    }

    if (currentRank == prestageTargetRank) {
      // Target reached. Hold here so the host can inspect the final state or
      // intentionally return the buffer to Regular/AUTO afterward.
      timedOut = false;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
      return;
    }

    if (currentRank < prestageTargetRank) {
      // Buffer is lower than requested; feed toward the toolhead side.
      setMotor(Motor::Push);
    } else {
      // Buffer is higher than requested; retract toward the spool side.
      setMotor(Motor::Retract);
    }
  }

  void handleRegular() {
    if (!hw.filamentPresent()) {
      if (lastFilament) {
        setMode(Mode::Emptying);
      } else {
        setMotor(Motor::Off);
        timedOut = false;
      }
      return;
    }

    bool move = false;

    if (hw.optical1()) {
      // Low buffer position: feed filament toward the toolhead side.
      setMotor(Motor::Push);
      moveStart = hw.timeMs();
      move = true;
    } else if (hw.optical3()) {
      // Over-extended buffer position: retract filament toward the spool side.
      setMotor(Motor::Retract);
      moveStart = hw.timeMs();
      move = true;
    } else if (hw.optical2()) {
      // Center/settled position: hold the buffer if the motor was previously active.
      if (motor != Motor::Off) {
        setMotor(Motor::Hold);
      }
    }

    if (motor == Motor::Push || motor == Motor::Retract) {
      if (!move && hw.timeMs() - moveStart >= timeoutMs) {
        // Movement was commanded but no sensor state refreshed the motion timer.
        timedOut = true;
        setMode(Mode::Hold);
        setMotor(Motor::Hold);
      }
    } else {
      timedOut = false;
    }
  }

  void handleEmptying() {
    if (hw.filamentPresent()) {
      setMode(Mode::Regular);
      return;
    }

    if (hw.timeMs() - emptyingStart >= timeoutMs) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
      return;
    }

    if (hw.optical1() || (motor == Motor::Push && !hw.optical1() && !hw.optical2() && !hw.optical3())) {
      // Continue pushing while emptying so the remaining filament tail can leave the buffer path.
      if (motor != Motor::Push || emptyingPushStart == 0) {
        emptyingPushStart = hw.timeMs();
      }
      setMotor(Motor::Push);
    } else if (hw.optical2()) {
      // Center/settled position during emptying.
      setMotor(Motor::Hold);
    } else if (hw.optical3()) {
      // Over-extended position during emptying.
      setMotor(Motor::Retract);
    }

    if (motor == Motor::Push && emptyingPushTimeoutMs > 0 && hw.timeMs() - emptyingPushStart >= emptyingPushTimeoutMs) {
      // Stop emptying after the tail-push window expires.
      setMode(Mode::Regular);
      setMotor(Motor::Off);
    }
  }
  
  void handleMoveCommand() {
    if (hw.timeMs() >= moveEnd) {
      if (hw.filamentPresent()) {
        setMode(Mode::Hold);
        setMotor(Motor::Hold);
      } else {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      }
    }
  }

  void handleContinuous() {
    if (!hw.filamentPresent()) {
      if (motor == Motor::Retract) {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      } else {
        setMode(Mode::Emptying);
      }
      return;
    }

    if (timeoutMs > 0 && hw.timeMs() - continuousStart >= timeoutMs) {
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
    }
  }

  void updateHoldTimeout() {
    if (!holdTimeoutEnabled || motor != Motor::Hold) {
      return;
    }

    if (hw.timeMs() - holdStart >= holdTimeoutMs) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
    }
  }

  void updateStatus(bool force = false) {
#ifdef ENABLE_UART_PROTOCOL
    if (filamentPresent != lastFilament || force) {
      hw.writeLineF("filament_present=%d", filamentPresent ? 1 : 0);
    }
    if (mode != lastMode || force) {
      auto modeStr = "";
      switch (mode) {
      case Mode::Regular:
        modeStr = "regular";
        break;
      case Mode::Continuous:
        modeStr = "continuous";
        break;
      case Mode::MoveCommand:
        modeStr = "move_command";
        break;
      case Mode::Hold:
        modeStr = "hold";
        break;
      case Mode::Manual:
        modeStr = "manual";
        break;
      case Mode::Emptying:
        modeStr = "emptying";
        break;
      case Mode::Prestage:
        modeStr = "prestage";
        break;
      }
      hw.writeLineF("mode=%s", modeStr);
      lastMode = mode;
    }
    if (motor != lastMotor || force) {
      auto statusStr = "";
      switch (motor) {
      case Motor::Push:
        statusStr = "push";
        break;
      case Motor::Retract:
        statusStr = "retract";
        break;
      case Motor::Hold:
        statusStr = "hold";
        break;
      case Motor::Off:
        statusStr = "off";
        break;
      }
      hw.writeLineF("status=%s", statusStr);
      lastMotor = motor;
    }
    if (timedOut != lastTimedOut || force) {
      hw.writeLineF("timed_out=%d", timedOut ? 1 : 0);
      lastTimedOut = timedOut;
    }
    if (lastTimeoutMs != timeoutMs || force) {
      hw.writeLineF("%s=%u", "timeout", timeoutMs);
      lastTimeoutMs = timeoutMs;
    }
    if (lastHoldTimeoutMs != holdTimeoutMs || force) {
      hw.writeLineF("%s=%u", "hold_timeout", holdTimeoutMs);
      lastHoldTimeoutMs = holdTimeoutMs;
    }
    if (lastHoldTimeoutEnabled != holdTimeoutEnabled || force) {
      hw.writeLineF("%s%s=%d", "hold_timeout", "_en", holdTimeoutEnabled ? 1 : 0);
      lastHoldTimeoutEnabled = holdTimeoutEnabled;
    }
    if (lastMultiPressCount != multiPressCount || force) {
      hw.writeLineF("%s=%u", "multi_press_count", multiPressCount);
      lastMultiPressCount = multiPressCount;
    }
    if (tiny::abs(lastSpeedMmS - speedMmS) > 0.01F || force) {
      hw.writeLineF("speed=%f", speedMmS);
      lastSpeedMmS = speedMmS;
    }
    if (lastEmptyingPushTimeoutMs != emptyingPushTimeoutMs || force) {
      hw.writeLineF("%s%s=%u", "emptying_", "timeout", emptyingPushTimeoutMs);
      lastEmptyingPushTimeoutMs = emptyingPushTimeoutMs;
    }
    if (lastPrestageLevel != prestageLevel || force) {
      hw.writeLineF("prestage_level=%u", prestageLevel);
      lastPrestageLevel = prestageLevel;
    }
#endif

#ifdef ENABLE_I2C_PROTOCOL
    bool changed = false;
    if (hw.filamentPresent() != lastFilament || force)
      changed = true;
    if (mode != lastMode || force)
      changed = true;
    if (motor != lastMotor || force)
      changed = true;
    if (timedOut != lastTimedOut || force)
      changed = true;
    if (lastTimeoutMs != timeoutMs || force)
      changed = true;
    if (lastHoldTimeoutMs != holdTimeoutMs || force)
      changed = true;
    if (lastHoldTimeoutEnabled != holdTimeoutEnabled || force)
      changed = true;
    if (lastMultiPressCount != multiPressCount || force)
      changed = true;
    if (tiny::abs(lastSpeedMmS - speedMmS) > 0.01F || force)
      changed = true;
    if (lastEmptyingPushTimeoutMs != emptyingPushTimeoutMs || force)
      changed = true;
    if (lastPrestageLevel != prestageLevel || force)
      changed = true;

    if (changed) {
      hw.setInterrupt(true);
    }
#endif
  }

  void setMotor(Motor m) {
    if (motor == m) {
      return;
    }

    motor = m;

    switch (motor) {
    case Motor::Push:
      hw.stepperPush(speedMmS);
      break;
    case Motor::Retract:
      hw.stepperRetract(speedMmS);
      break;
    case Motor::Hold:
      hw.stepperHold();
      holdStart = hw.timeMs();
      break;
    case Motor::Off:
      hw.stepperOff();
      break;
    }
  }

  void setMode(Mode m) {
    if (mode == m) {
      return;
    }

    const uint32_t now = hw.timeMs();

    switch (m) {
    case Mode::Continuous:
      continuousStart = now;
      break;
    case Mode::Emptying:
      emptyingStart = now;
      emptyingPushStart = 0;
      break;
    case Mode::Prestage:
      prestageLastTick = now;
      break;
    default:
      break;
    }

    mode = m;
  }
};