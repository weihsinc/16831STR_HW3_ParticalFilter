#include <vector>
#include <string>
#include <iostream>

class Map {
public:

  Map(const std::string& map_fn);

/*
private:
*/
  int resolution;
  float offset_x, offset_y;
  int size_x, size_y;

  int min_x, max_x;
  int min_y, max_y;

  std::vector<std::vector<float>> prob;
};

std::ostream& operator << (std::ostream&, const Map& map);
