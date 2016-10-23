#include <vector>
#include <string>
#include <iostream>

// OpenCV (for visualization and debugging)
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Map {
public:

  Map(const std::string& map_fn);

  void read_map_from_file(const std::string& mapName);

/*
private:
*/
  int resolution;
  float offset_x, offset_y;
  int size_x, size_y;

  int min_x, max_x;
  int min_y, max_y;

  std::vector<std::vector<float>> prob;
  cv::Mat cv_img;
};

std::ostream& operator << (std::ostream&, const Map& map);
