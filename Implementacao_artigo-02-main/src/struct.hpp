
#ifndef STRUCT_HPP 
#define STRUCT_HPP 

#include <vector>

struct Pixel {
  int x;
  int y;
  int R;
  int G;
  int B;
};

struct Image {
  int width;
  int height;
  int max_color;
  vector<vector<struct Pixel>> pixel_matrix;
};

#endif
