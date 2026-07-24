#pragma once

#include <cstdint>

namespace smartcar::vision::cross_geometry
{

// Use two valid boundary samples to interpolate or extrapolate x as a function
// of image row. The completed values are clamped before conversion to uint8_t.
bool fill_boundary_from_rows(std::uint8_t *boundary,
                             int row_count,
                             int source_row_a,
                             int source_row_b,
                             int first_fill_row,
                             int last_fill_row,
                             int min_valid_x,
                             int max_valid_x);

}
