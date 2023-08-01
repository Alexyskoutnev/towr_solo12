#include "towr/terrain/custom_terrain.hpp"

namespace towr {

double CustomTerrain::GetHeight(double x, double y) const {
  double h = 0.0;

  if (x > box1_x_pos_ && x <= box1_x_pos_ + box1_length_ && y > box1_y_pos_ && y <= box1_y_pos_ + box1_width_) {
    h = box1_height_;
  }

  if (x > box2_x_pos_ && x <= box2_x_pos_ + box2_length_ && y > box2_y_pos_ && y <= box2_y_pos_ + box2_width_) {
    h = box2_height_;
  }
 
  return h;
}

} // namespace towr