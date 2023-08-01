#include "towr/terrain/custom_terrain.hpp"

namespace towr {

double CustomTerrain::GetHeight(double x, double y) const {
  double h = 0.0;

  if (x>=first_step_start_)
    h = height_first_step;

  if (x>=first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>=first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}

} // namespace towr