#include "Logger.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

namespace {
  // Keep a short in-memory history of important serial lines.
  static const int    LOG_MAX_LINES = 80;
  static const size_t LOG_MAX_BYTES = 100 * 1024;
  static String logLines[LOG_MAX_LINES];
  static int    logHead  = 0;
  static int    logCount = 0;
  static size_t logBytes = 0;

  // Drop the oldest stored line when the ring buffer is full.
  void dropOldestLine(){
    if (logCount <= 0) return;
    size_t len = logLines[logHead].length();
    if (logBytes >= len){
      logBytes -= len;
    } else {
      logBytes = 0;
    }
    logLines[logHead] = "";
    logHead = (logHead + 1) % LOG_MAX_LINES;
    --logCount;
  }

  // Store only warnings and errors so the buffer stays useful.
  bool isCriticalLogLine(const String& line){
    if (line.length() == 0) return false;
    String upper = line;
    upper.toUpperCase();
    static const char* kMarkers[] = {
      "ERR", "WARN", "FAIL", "CRIT", "PANIC", "OVRFLW", "OVERFLOW", "CLIP", "I2S"
    };
    for (size_t i = 0; i < sizeof(kMarkers)/sizeof(kMarkers[0]); ++i){
      if (upper.indexOf(kMarkers[i]) >= 0){
        return true;
      }
    }
    // Also keep lines emitted by diagnostic macros with symbol prefixes.
    if (line.indexOf("\xE2\x9D\x8C") >= 0 || line.indexOf("\xE2\x9A\xA0") >= 0){
      return true;
    }
    return false;
  }
}

namespace LogBuffer {

  // Append a line, trimming old entries when count or byte limits are reached.
  void append(const String& srcLine){
    if (LOG_MAX_LINES <= 0) return;
    String line = srcLine;
    size_t lineBytes = line.length();
    if (lineBytes > LOG_MAX_BYTES){
      line = line.substring(lineBytes - LOG_MAX_BYTES);
      lineBytes = line.length();
    }
    while (logCount > 0 && (logBytes + lineBytes) > LOG_MAX_BYTES){
      dropOldestLine();
    }
    if (logCount == LOG_MAX_LINES){
      dropOldestLine();
    }
    int dst = (logHead + logCount) % LOG_MAX_LINES;
    logLines[dst] = line;
    logBytes += lineBytes;
    ++logCount;
  }

  // Number of stored lines.
  int count(){
    return logCount;
  }

  // Return a stable C string for a stored line.
  const char* lineAt(int index){
    if (index < 0 || index >= logCount || LOG_MAX_LINES <= 0) return "";
    int realIdx = (logHead + index) % LOG_MAX_LINES;
    return logLines[realIdx].c_str();
  }

  // Clear every buffered log line.
  void clear(){
    logHead = 0;
    logCount = 0;
    logBytes = 0;
    for(int i=0;i<LOG_MAX_LINES;++i){
      logLines[i] = "";
    }
  }

} // namespace LogBuffer

LoggingSerial LogSerial(::Serial);

// Wrap the hardware serial object.
LoggingSerial::LoggingSerial(HardwareSerial& hwRef) : hw(hwRef) {}

// Start serial output.
void LoggingSerial::begin(unsigned long baud){
  hw.begin(baud);
}

// Print a newline and finish the current buffered line.
void LoggingSerial::println(){
  hw.println();
  commitLine();
}

// Mirror flash-string output to the critical log filter.
size_t LoggingSerial::print(const __FlashStringHelper* value){
  size_t n = hw.print(value);
  String s(value);
  appendChunk(s);
  return n;
}

// Print a flash string and finish the current buffered line.
size_t LoggingSerial::println(const __FlashStringHelper* value){
  size_t n = hw.println(value);
  String s(value);
  appendChunk(s);
  commitLine();
  return n;
}

// Format output through the hardware serial port and the log filter.
size_t LoggingSerial::printf(const char* fmt, ...){
  if (!fmt) return 0;
  va_list args;
  va_start(args, fmt);
  va_list argsCopy;
  va_copy(argsCopy, args);
  int needed = vsnprintf(nullptr, 0, fmt, argsCopy);
  va_end(argsCopy);
  if (needed < 0){
    va_end(args);
    return 0;
  }
  char* buf = (char*)malloc(needed + 1);
  if (!buf){
    va_end(args);
    return 0;
  }
  vsnprintf(buf, needed + 1, fmt, args);
  va_end(args);
  size_t n = hw.print(buf);
  appendChunk(buf);
  free(buf);
  return n;
}

// Force the current partial line through the critical log filter.
void LoggingSerial::flushLine(){
  if (currentLine.length() == 0) return;
  commitLine();
}

// Append an Arduino String to the current buffered line.
void LoggingSerial::appendChunk(const String& chunk){
  appendChunk(chunk.c_str());
}

// Split raw text into complete log lines.
void LoggingSerial::appendChunk(const char* chunk){
  if (!chunk) return;
  while (*chunk){
    char c = *chunk++;
    if (c == '\r'){
      continue;
    }
    if (c == '\n'){
      commitLine();
    } else {
      currentLine += c;
      if (currentLine.length() > 220){
        commitLine();
      }
    }
  }
}

// Store the current line if it matches the critical-line filter.
void LoggingSerial::commitLine(){
  if (currentLine.length() == 0) return;
  if (isCriticalLogLine(currentLine)){
    LogBuffer::append(currentLine);
  }
  currentLine = "";
}
