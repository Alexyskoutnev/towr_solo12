#include "towr/terrain/custom_terrain.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace towr
{

CustomTerrain::CustomTerrain(const std::string &file_name)
{
	height_field_ = ReadHeightField(file_name);
}

// read heightfield from a file formatted as follows:
// 1, 2, 3,
// 4, 5, 6,
// 7, 8, 9,
HeightField
CustomTerrain::ReadHeightField(const std::string &file_name)
{
	HeightField height_field;

	std::ifstream file(file_name);
	if (!file) {
		std::cerr << "Could not open file " << file_name << std::endl;
		return height_field;
	}

	std::string line;
	while (std::getline(file, line)) {
		std::stringstream ss(line);
		std::vector<double> row;
		double val;
		while (ss >> val) {
			row.push_back(val);
			if (ss.peek() == ',')
				ss.ignore();
		}
		height_field.push_back(row);
	}

	return height_field;
}

double
CustomTerrain::GetHeight(double x, double y) const
{
	double z = 0.0;

	// bilinear interpolation
	const size_t x_size = height_field_.size();
	const size_t y_size = height_field_[0].size();

	const double x_floor = std::floor((x - mesh_x_offset_) / x_step_length_);
	const double y_floor = std::floor((y - mesh_y_offset_) / y_step_length_);

	const size_t x0_index = std::min(static_cast<size_t>(x_floor), x_size - 1);
	const size_t y0_index = std::min(static_cast<size_t>(y_floor), y_size - 1);
	const size_t x1_index = std::min(x0_index + 1, x_size - 1);
	const size_t y1_index = std::min(y0_index + 1, y_size - 1);

	const double x0 = x0_index * x_step_length_ + mesh_x_offset_;
	const double x1 = x1_index * x_step_length_ + mesh_x_offset_;
	const double y0 = y0_index * y_step_length_ + mesh_y_offset_;
	const double y1 = y1_index * y_step_length_ + mesh_y_offset_;
	const double z00 = height_field_[x0_index][y0_index] * z_scaling_ + mesh_z_offset_;
	const double z01 = height_field_[x0_index][y1_index] * z_scaling_ + mesh_z_offset_;
	const double z10 = height_field_[x1_index][y0_index] * z_scaling_ + mesh_z_offset_;
	const double z11 = height_field_[x1_index][y1_index] * z_scaling_ + mesh_z_offset_;

	// u = [x1 - x;
	//      x - x0],
	Eigen::Vector2d u;
	u << x1 - x, x - x0;
	// A = [f(x0, y0), f(x0, y1);
	//      f(x1, y0), f(x1, y1)]
	Eigen::Matrix<double, 2, 2> A;
	A << z00, z01, z10, z11;
	// v = [y1 - y;
	//      y - y0]
	Eigen::Vector2d v;
	v << y1 - y, y - y0;
	// https://en.wikipedia.org/wiki/Bilinear_interpolation#Repeated_linear_interpolation
	// 1/(dx*xy) * u^T * A * v
	z = 1 / (x_step_length_ * y_step_length_) * u.transpose() * A * v;

	return z;
}

double
CustomTerrain::GetHeightDerivWrtX(double x, double y) const
{
	double z_dx = 0.0;

	///*
	const size_t x_size = height_field_.size();
	const size_t y_size = height_field_[0].size();

	const double x_floor = std::floor((x - mesh_x_offset_) / x_step_length_);
	const double y_floor = std::floor((y - mesh_y_offset_) / y_step_length_);

	const size_t x0_index = std::min(static_cast<size_t>(x_floor), x_size - 1);
	const size_t y0_index = std::min(static_cast<size_t>(y_floor), y_size - 1);
	const size_t x1_index = std::min(x0_index + 1, x_size - 1);
	const size_t y1_index = std::min(y0_index + 1, y_size - 1);

	const double x0 = x0_index * x_step_length_ + mesh_x_offset_;
	const double x1 = x1_index * x_step_length_ + mesh_x_offset_;
	const double y0 = y0_index * y_step_length_ + mesh_y_offset_;
	const double y1 = y1_index * y_step_length_ + mesh_y_offset_;
	const double z00 = height_field_[x0_index][y0_index] * z_scaling_ + mesh_z_offset_;
	const double z01 = height_field_[x0_index][y1_index] * z_scaling_ + mesh_z_offset_;
	const double z10 = height_field_[x1_index][y0_index] * z_scaling_ + mesh_z_offset_;
	const double z11 = height_field_[x1_index][y1_index] * z_scaling_ + mesh_z_offset_;

	z_dx = 1 / (x_step_length_ * y_step_length_) *
	    ((-z00 + z10) * (y1 - y) + (-z01 + z11) * (y - y0));
	//*/
	return z_dx;
}

double
CustomTerrain::GetHeightDerivWrtY(double x, double y) const
{
	double z_dy = 0.0;

	///*
	const size_t x_size = height_field_.size();
	const size_t y_size = height_field_[0].size();

	const double x_floor = std::floor((x - mesh_x_offset_) / x_step_length_);
	const double y_floor = std::floor((y - mesh_y_offset_) / y_step_length_);

	const size_t x0_index = std::min(static_cast<size_t>(x_floor), x_size - 1);
	const size_t y0_index = std::min(static_cast<size_t>(y_floor), y_size - 1);
	const size_t x1_index = std::min(x0_index + 1, x_size - 1);
	const size_t y1_index = std::min(y0_index + 1, y_size - 1);

	const double x0 = x0_index * x_step_length_ + mesh_x_offset_;
	const double x1 = x1_index * x_step_length_ + mesh_x_offset_;
	const double y0 = y0_index * y_step_length_ + mesh_y_offset_;
	const double y1 = y1_index * y_step_length_ + mesh_y_offset_;
	const double z00 = height_field_[x0_index][y0_index] * z_scaling_ + mesh_z_offset_;
	const double z01 = height_field_[x0_index][y1_index] * z_scaling_ + mesh_z_offset_;
	const double z10 = height_field_[x1_index][y0_index] * z_scaling_ + mesh_z_offset_;
	const double z11 = height_field_[x1_index][y1_index] * z_scaling_ + mesh_z_offset_;

	z_dy = 1 / (x_step_length_ * y_step_length_) *
	    (z00 * (x - x1) + z10 * (x0 - x) + z01 * (x1 - x) + z11 * (x - x0));
	//*/

	return z_dy;
}

} // namespace towr