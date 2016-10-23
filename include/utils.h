#ifndef __UTILS_H_
#define __UTILS_H_
#include <time.h>
#include <string>
#include <sstream>
#include <vector>

#define PI 3.14159265359

void split(const std::string &s, char delim, std::vector<std::string> &elems);

std::vector<std::string> split(const std::string &s, char delim = ' ');

template <typename T>
std::ostream& operator << (std::ostream& os, const std::vector<T>& arr) {
  for (const auto& x : arr)
    os << x << " ";
  os << std::endl;
  return os;
}

template <typename T>
std::vector<bool> operator == (const std::vector<T>& arr, const T val) {
  std::vector<bool> ind(arr.size());
  for (size_t i=0; i<arr.size(); ++i)
    ind[i] = (arr[i] == val);
  return ind;
}

bool any(const std::vector<bool>& arr);
bool all(const std::vector<bool>& arr);

std::vector<float> splitAsFloat(const std::string &s, char delim = ' ');

// call this function to start a nanosecond-resolution timer
struct timespec timer_start();

// call this function to end a timer, returning nanoseconds elapsed as a long
double timer_end(struct timespec start_time);

#endif // __UTILS_H_
