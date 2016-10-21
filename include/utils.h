#ifndef __UTILS_H_
#define __UTILS_H_
#include <string>
#include <sstream>
#include <vector>

void split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim = ' ');

#endif // __UTILS_H_
