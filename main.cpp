#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include <map.h>

int main(int argc, char* argv[]) {

  Map map("data/map/wean.dat");
  cout << map << endl;
  return 0;
}
