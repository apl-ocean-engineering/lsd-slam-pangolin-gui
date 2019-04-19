

#include <iostream>

#pragma once

struct ColorStderrSink {

  ColorStderrSink( const LEVELS threshold = INFO )
    : _threshold( threshold )
  {;}

  ~ColorStderrSink()
  {
    std::cerr << std::endl;
  }

  void setThreshold( const LEVELS t )
    { _threshold = t; }

  // Linux xterm color
  // http://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
  enum FG_Color {YELLOW = 33, RED = 31, GREEN=32, WHITE = 97};

  FG_Color GetColor(const LEVELS level) const {
    if (level.value == WARNING.value) { return YELLOW; }
    if (level.value == INFO.value)   { return WHITE; }
    if (level.value == DEBUG.value) { return GREEN; }
    if (g3::internal::wasFatal(level)) { return RED; }

    return WHITE;
  }

  void ReceiveLogMessage(g3::LogMessageMover logEntry) {
    auto level = logEntry.get()._level;

    if( level.value >= _threshold.value ) {
      auto color = GetColor(level);

      std::cerr << "\033[" << color << "m"
      << logEntry.get().toString() << "\033[m"; // << std::endl;
    }
  }

private:

  LEVELS _threshold;
};
