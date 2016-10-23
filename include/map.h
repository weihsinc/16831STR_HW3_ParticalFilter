#include <vector>
#include <string>
#include <iostream>

// OpenCV (for visualization and debugging)
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pose.h>

class Map {
public:

  Map(const std::string& map_fn);
  ~Map();

  bool inside(const Pose& p) const;
  void read_map_from_file(const std::string& mapName);

/*
private:
*/
  int resolution;
  float offset_x, offset_y;
  size_t size_x, size_y;

  size_t min_x, max_x;
  size_t min_y, max_y;

  float* prob;
  cv::Mat cv_img;
};

std::ostream& operator << (std::ostream&, const Map& map);
