#pragma once
#include <Arduino.h>

namespace LogBuffer {
  // Ring buffer used to keep recent important serial messages in RAM.
  void append(const String& line);
  int  count();
  const char* lineAt(int index);
  void clear();
}

class LoggingSerial {
public:
  // Thin wrapper around HardwareSerial that mirrors critical lines to LogBuffer.
  explicit LoggingSerial(HardwareSerial& hw);
  void begin(unsigned long baud);
  void println();
  template<typename T>
  size_t print(const T& value) {
    size_t n = hw.print(value);
    appendChunk(String(value));
    return n;
  }
  template<typename T>
  size_t print(const T& value, int format) {
    size_t n = hw.print(value, format);
    appendChunk(String(value, format));
    return n;
  }
  size_t print(const __FlashStringHelper* value);
  size_t println(const __FlashStringHelper* value);
  template<typename T>
  size_t println(const T& value) {
    size_t n = hw.println(value);
    appendChunk(String(value));
    commitLine();
    return n;
  }
  template<typename T>
  size_t println(const T& value, int format) {
    size_t n = hw.println(value, format);
    appendChunk(String(value, format));
    commitLine();
    return n;
  }
  size_t printf(const char* fmt, ...);
  void   flushLine();
private:
  HardwareSerial& hw;
  String currentLine;
  void appendChunk(const String& chunk);
  void appendChunk(const char* chunk);
  void commitLine();
};

extern LoggingSerial LogSerial;
