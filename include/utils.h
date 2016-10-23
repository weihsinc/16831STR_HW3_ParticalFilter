#ifndef __UTILS_H_
#define __UTILS_H_
#include <time.h>
#include <string>
#include <sstream>
#include <vector>

void split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim = ' ');

template <typename T>
std::ostream& operator << (std::ostream& os, const std::vector<T>& arr) {
  for (const auto& x : arr)
    os << x << " ";
  os << std::endl;
  return os;
}

std::vector<float> splitAsFloat(const std::string &s, char delim = ' ');

// call this function to start a nanosecond-resolution timer
struct timespec timer_start();

// call this function to end a timer, returning nanoseconds elapsed as a long
double timer_end(struct timespec start_time);

#endif // __UTILS_H_
