#ifndef CUSTOM_TERRAIN_HPP_CINARAL_230801_1412
#define CUSTOM_TERRAIN_HPP_CINARAL_230801_1412

#include "towr/terrain/height_map.h"
#include <iostream>
#include <filesystem>

namespace towr
{

using HeightField = std::vector<std::vector<double>>;

/**
 * @brief Custom terrain
 */
class CustomTerrain : public HeightMap
{
  public:
	// figure out how to pass the file using roslaunch???
	// static std::string height_file = "towr/towr/data/data/heightfield.txt";
	CustomTerrain(const std::string &file_name = "towr/towr/data/data/heightfield.txt");
	HeightField ReadHeightField(const std::string &file_name);
	double GetHeight(double x, double y) const override;
	double GetHeightDerivWrtX(double x, double y) const override;
	double GetHeightDerivWrtY(double x, double y) const override;

  private:
	HeightField height_field_;
	const double x_step_length_ = .1;  // [m]
	const double y_step_length_ = .1;  // [m]
	const double z_scaling_ = 1.0;       // [m]
	const double mesh_x_offset_ = 0.3; // [m]
	const double mesh_y_offset_ = 1.0; // [m]
	const double mesh_z_offset_ = 0.0;      // [m]
};
} // namespace towr

#endif