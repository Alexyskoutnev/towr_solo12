#include "towr/terrain/custom_terrain.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace towr {

CustomTerrain::CustomTerrain(const std::string &file_name) {
  height_field_ = ReadHeightField(file_name);
}

// read heightfield from a file formatted as follows:
// 1, 2, 3,
// 4, 5, 6,
// 7, 8, 9,
HeightField CustomTerrain::ReadHeightField(const std::string &file_name) {
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

double CustomTerrain::GetHeight(double x, double y) const {
  const size_t x_size = height_field_.size();
  const size_t y_size = height_field_[0].size();
  const size_t x_index = std::min(static_cast<size_t>(std::floor((x - mesh_x_offset_) / mesh_size_)), x_size - 1);
  const size_t y_index = std::min(static_cast<size_t>(std::floor((y - mesh_y_offset_) / mesh_size_)), y_size - 1);
  
  return height_field_[x_index][y_index];
}

} // namespace towr