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
