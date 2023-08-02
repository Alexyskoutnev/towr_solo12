#ifndef CUSTOM_TERRAIN_HPP_CINARAL_230801_1412
#define CUSTOM_TERRAIN_HPP_CINARAL_230801_1412

#include "towr/terrain/height_map.h"

namespace towr {

using HeightField = std::vector<std::vector<double>>;

/**
 * @brief Custom terrain
 */
class CustomTerrain : public HeightMap {
public:
  // figure out how to pass the file using roslaunch???
  CustomTerrain(const std::string &file_name =
                    "/home/cinaral/git_ws/solo12_towr_ws/src/"
                    "towr_solo12/towr/data/heightfield.txt");
  double GetHeight(double x, double y) const override;
  HeightField ReadHeightField(const std::string &file_name);

private:
  HeightField height_field_;
  const double mesh_size_ = .1;      // [m]
  const double mesh_x_offset_ = 0.0; // [m]
  const double mesh_y_offset_ = 0.0; // [m]
};
} // namespace towr

#endif