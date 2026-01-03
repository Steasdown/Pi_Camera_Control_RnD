#pragma once
#include <Arduino.h>

// BigMover-style PCCommunication equivalent.
// Included for structure only; NOT used in the loop yet.
//
// Reserved command words (future):
//   MODE <MANUAL|AUTO_SWEEP|AUTO_PI>
//   PANSPD <steps_per_sec>
//   STOP
//   HOME
//   STATUS?
//
// Suggested responses (future):
//   OK <cmd>
//   ERR <reason>

enum class PiCommandType : uint8_t {
  None,
  MODE,
  PANSPD,
  STOP,
  HOME,
  STATUS
};

struct PiCommand {
  PiCommandType type = PiCommandType::None;
  // Parsed payload (optional)
  char arg0[16] = {0};
  float value = 0.0f;
};

class PiCommunication {
public:
  PiCommunication() = default;

  // Call update() to read from Serial and populate lastCmd().
  // Not called in current refactor to avoid behaviour change.
  void update();

  bool hasCommand() const { return hasCmd_; }
  PiCommand lastCmd() const { return lastCmd_; }
  void clear() { hasCmd_ = false; lastCmd_ = PiCommand{}; }

private:
  static constexpr size_t BUF_N = 96;
  char buf_[BUF_N] = {0};
  size_t n_ = 0;

  bool hasCmd_ = false;
  PiCommand lastCmd_;

  void processLine_(const char* line);
};
