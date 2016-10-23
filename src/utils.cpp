#include <iostream>
#include <utils.h>
using namespace std;

void split(const string &s, char delim, vector<string> &elems) {
  stringstream ss;
  ss.str(s);
  string item;
  while (getline(ss, item, delim)) {
    elems.push_back(item);
  }
}


vector<string> split(const string &s, char delim) {
  vector<string> elems;
  split(s, delim, elems);
  return elems;
}

vector<float> splitAsFloat(const string &s, char delim) {
  auto tokens = split(s, delim);
  vector<float> output(tokens.size());

  for (size_t i=0; i<output.size(); ++i)
    output[i] = stof(tokens[i]);

  return output;
}

// call this function to start a nanosecond-resolution timer
struct timespec timer_start(){
  struct timespec start_time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
  return start_time;
}

// call this function to end a timer, returning nanoseconds elapsed as a long
double timer_end(struct timespec start_time){
  struct timespec end_time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
  long diffInNanos = (end_time.tv_sec - start_time.tv_sec) * 1e9 + end_time.tv_nsec - start_time.tv_nsec;
  return double(diffInNanos) / 1e9;
}

bool any(const vector<bool>& arr) {
  for (const auto& x : arr) {
    if (x)
      return true;
  }
  return false;
}

bool all(const vector<bool>& arr) {
  for (const auto& x : arr) {
    if (!x)
      return false;
  }
  return true;
}
