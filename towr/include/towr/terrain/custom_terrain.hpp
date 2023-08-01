#ifndef CUSTOM_TERRAIN_HPP_CINARAL_230801_1412
#define CUSTOM_TERRAIN_HPP_CINARAL_230801_1412

#include "towr/terrain/height_map.h"

namespace towr {

/**
 * @addtogroup Terrains
 * @{
 */

/**
 * @brief Custom terrain
 */
class CustomTerrain : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  const double box1_x_pos_ = 0.5;  // [m] x position of the obstacle
  const double box1_y_pos_ = -0.5; // [m] y position of the obstacle
  const double box1_length_ = 0.5; // [m] length of the obstacle (in x direction)
  const double box1_width_ = 1.0;  // [m] width of the obstacle (in y direction)
  const double box1_height_ = 0.1; // [m] height of the obstacle

  const double box2_x_pos_ = 1.7;  // [m] x position of the obstacle
  const double box2_y_pos_ = -1.0; // [m] y position of the obstacle
  const double box2_length_ = box1_length_; // [m] length of the obstacle (in x direction)
  const double box2_width_ = box1_width_;  // [m] width of the obstacle (in y direction)
  const double box2_height_ = 0.2; // [m] height of the obstacle
};
} // namespace towr

#endif