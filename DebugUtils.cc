#ifndef DebugUtils_
#define DebugUtils_

#include <algorithm>
#include <iterator>

#define DEBUG 1

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
      result += std::to_string(arr[i]);
      result += ", ";
    }
    result += "\n</" + name + ">\n";
    return result;
  }

  inline void log(std::string s) {
    if (!DEBUG) {
      return;
    }
    std::cout << s << "\n";
  }

}

#endif