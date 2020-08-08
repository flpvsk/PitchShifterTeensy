#ifndef DebugUtils_h_
#define DebugUtils_h_

#include <algorithm>
#include <iterator>

#define DEBUG 0
#include <Arduino.h>

namespace zao {

  inline float *snapshot(float *src, int size) {
    auto res = new float[size];
    std::copy(src, src + size, res);
    return res;
  }

  inline std::string to_debug_string(
    std::string name,
    float *arr,
    int size
  ) {
    std::string result = "<" + name + ">\n";
    for (int i = 0; i < size; i++) {
      // result += std::to_string(arr[i]);
      result += arr[i];
      result += ", ";
    }
    result += "\n</" + name + ">\n";
    return result;
  }

#if DEBUG
  inline void log(const char* s) {
    if (!DEBUG) {
      return;
    }

    // if (!ARDUINO_LOG) {
    //   std::cout << s << "\n";
    // }

    Serial.println(s);
  }
#else
  inline void log(const char* s) {
    return;
  }
#endif

}

#endif
