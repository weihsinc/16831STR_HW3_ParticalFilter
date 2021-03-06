#ifndef __UTILS_H_
#define __UTILS_H_
#include <time.h>
#include <string>
#include <sstream>
#include <vector>

#define PI 3.14159265359
#define debug(x) {std::cout << #x " = " << x << std::endl; }

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

template <typename T>
std::vector<T>& operator+=(std::vector<T>& lhs, const std::vector<T>& rhs)
{
	for (size_t i=0; i<rhs.size(); ++i)
		lhs[i] += rhs[i];
    return lhs;
}

template <typename T>
std::vector<size_t> sort(const std::vector<T>& arr) {

  std::vector<size_t> indices(arr.size());
  for (size_t i=0; i<indices.size(); ++i)
      indices[i] = i;

  sort(indices.begin(), indices.end(), 
        [&arr] (const size_t &i, const size_t& j) -> bool {
        return arr[i] < arr[j];
  });

  return indices;
}

template <typename T>
T arrsum(const std::vector<T>& arr) {
  T s = 0;
  for (auto& x : arr)
    s += x;
  return s;
}

#endif // __UTILS_H_
