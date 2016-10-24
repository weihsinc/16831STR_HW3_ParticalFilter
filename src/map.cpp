#include <map.h>
#include <stdexcept>
#include <cstdio>
#include <cstring>

using namespace std;

Map::Map(const string& mapName) {
  this->read_map_from_file(mapName);

  cv_img = cv::Mat(size_y, size_x, CV_8UC3, cv::Scalar(0, 0, 0));
  for (size_t i=0; i<size_x; ++i) {
    for (size_t j=0; j<size_y; ++j) {
      for (size_t k=0; k<3; ++k)
	cv_img.data[j*cv_img.step + i * 3 + k] = (1. - this->prob[i * size_y + j]) * 255;
    }
  }
}

Map::~Map() {
  delete [] prob;
}

bool Map::inside(const Pose& p) const {
  size_t ix = p.x / resolution;
  size_t iy = p.y / resolution;

  return ix < size_x && iy < size_y && prob[ix * size_y + iy] == 0;
}

cv::Point Map::to_idx(const Pose& p) const {
  return {(int) (p.x / resolution), (int) (p.y / resolution)};
}

void Map::read_map_from_file(const string& mapName) {
  char line[256];
  FILE *fp;

  if((fp = fopen(mapName.c_str(), "rt")) == NULL)
    throw std::runtime_error("# Could not open file " + mapName);

  fprintf(stderr, "# Reading map: %s\n", mapName.c_str());
  while((fgets(line, 256, fp) != NULL)
      && (strncmp("global_map[0]", line , 13) != 0)) {
    if(strncmp(line, "robot_specifications->resolution", 32) == 0)
      if(sscanf(&line[32], "%d", &(this->resolution)) != 0)
	fprintf(stderr, "# Map resolution: %d cm\n", this->resolution);
    if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
      if(sscanf(&line[35], "%g", &(this->offset_x)) != 0) {
	fprintf(stderr, "# Map offsetX: %g cm\n", this->offset_x);
      }
    if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
      if (sscanf(&line[35], "%g", &(this->offset_y)) != 0) {
	fprintf(stderr, "# Map offsetY: %g cm\n", this->offset_y);
      }
    }
  }

  if(sscanf(line,"global_map[0]: %zu %zu", &this->size_y, &this->size_x) != 2)
    throw std::runtime_error("ERROR: corrupted file " + mapName);

  fprintf(stderr, "# Map size: %zu %zu\n", this->size_x, this->size_y);

  // new_hornetsoft_map(map, this->size_x, this->size_y);

  this->min_x = this->size_x;
  this->max_x = 0;
  this->min_y = this->size_y;
  this->max_y = 0;

  int count = 0;
  float temp;

  this->prob = new float[this->size_x * this->size_y];

  for (size_t x = 0; x < this->size_x; x++)
    for (size_t y = 0; y < this->size_y; y++, count++) {
      if (count % 10000 == 0)
	fprintf(stderr, "\r# Reading ... (%.2f%%)",
	    count / (float)(this->size_x * this->size_y) * 100);

      fscanf(fp,"%e", &temp);
      if(temp < 0.0)
	this->prob[x * size_y + y] = -1;
      else {
	if(x < this->min_x)
	  this->min_x = x;
	else if(x > this->max_x)
	  this->max_x = x;
	if(y < this->min_y)
	  this->min_y = y;
	else if(y > this->max_y)
	  this->max_y = y;
	this->prob[x * size_y + y] = 1 - temp;
      }
    }

  fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
      count / (float)(this->size_x * this->size_y) * 100);

  fclose(fp);
}

ostream& operator << (ostream& os, const Map& map) {

  os << "resolution: " << map.resolution << endl;
  os << "size_x: " << map.size_x << ", size_y: " << map.size_y << endl;
  os << "offset_x: " << map.offset_x << ", offset_y: " << map.offset_y << endl;

  os << "x ~ [" << map.min_x << ", " << map.max_x << "], y ~ [" << map.min_y << ", " << map.max_y << "]" << endl;

  return os;
}
