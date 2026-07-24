#include "cross_fill_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace smartcar::vision::cross_geometry
{

bool fill_boundary_from_rows(std::uint8_t *boundary,
                             int row_count,
                             int source_row_a,
                             int source_row_b,
                             int first_fill_row,
                             int last_fill_row,
                             int min_valid_x,
                             int max_valid_x)
{
    if (boundary == nullptr || row_count <= 0 || min_valid_x > max_valid_x ||
        source_row_a < 0 || source_row_a >= row_count ||
        source_row_b < 0 || source_row_b >= row_count ||
        source_row_a == source_row_b ||
        first_fill_row < 0 || first_fill_row >= row_count ||
        last_fill_row < first_fill_row || last_fill_row >= row_count)
    {
        return false;
    }

    const int source_x_a = boundary[source_row_a];
    const int source_x_b = boundary[source_row_b];
    if (source_x_a < min_valid_x || source_x_a > max_valid_x ||
        source_x_b < min_valid_x || source_x_b > max_valid_x)
    {
        return false;
    }

    const float x_per_row =
        static_cast<float>(source_x_b - source_x_a) /
        static_cast<float>(source_row_b - source_row_a);
    if (!std::isfinite(x_per_row))
    {
        return false;
    }

    for (int row = first_fill_row; row <= last_fill_row; ++row)
    {
        const float projected_x = static_cast<float>(source_x_a) +
                                  static_cast<float>(row - source_row_a) * x_per_row;
        if (!std::isfinite(projected_x))
        {
            return false;
        }

        const int completed_x = std::clamp(static_cast<int>(std::lround(projected_x)),
                                           min_valid_x,
                                           max_valid_x);
        boundary[row] = static_cast<std::uint8_t>(completed_x);
    }
    return true;
}

}
