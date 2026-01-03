#include "PiCommunication.h"

static bool startsWith(const char* s, const char* prefix) {
  while (*prefix) {
    if (*s++ != *prefix++) return false;
  }
  return true;
}

void PiCommunication::update() {
  // Minimal line reader (\n terminated)
  while (Serial.available() > 0 && !hasCmd_) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf_[n_] = '\0';
      if (n_ > 0) processLine_(buf_);
      n_ = 0;
      return;
    }

    if (n_ < (BUF_N - 1)) buf_[n_++] = c;
  }
}

void PiCommunication::processLine_(const char* line) {
  // Placeholder parser. Keep tiny; extend later.
  // Example accepted:
  //   MODE AUTO_PI
  //   PANSPD 250.0
  //   STOP
  //   HOME
  //   STATUS?
  PiCommand cmd;

  if (startsWith(line, "MODE ")) {
    cmd.type = PiCommandType::MODE;
    strncpy(cmd.arg0, line + 5, sizeof(cmd.arg0) - 1);
  } else if (startsWith(line, "PANSPD ")) {
    cmd.type = PiCommandType::PANSPD;
    cmd.value = atof(line + 7);
  } else if (strcmp(line, "STOP") == 0) {
    cmd.type = PiCommandType::STOP;
  } else if (strcmp(line, "HOME") == 0) {
    cmd.type = PiCommandType::HOME;
  } else if (strcmp(line, "STATUS?") == 0) {
    cmd.type = PiCommandType::STATUS;
  } else {
    cmd.type = PiCommandType::None;
  }

  if (cmd.type != PiCommandType::None) {
    lastCmd_ = cmd;
    hasCmd_ = true;
  }
}
