#include "cross_fill_geometry.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace
{
constexpr int k_rows = 120;
constexpr int k_min_x = 3;
constexpr int k_max_x = 156;

void test_vertical_boundary()
{
    std::array<std::uint8_t, k_rows> boundary{};
    boundary.fill(42);

    assert(smartcar::vision::cross_geometry::fill_boundary_from_rows(
        boundary.data(), k_rows, 20, 30, 5, 117, k_min_x, k_max_x));
    for (int row = 5; row <= 117; ++row)
    {
        assert(boundary[row] == 42);
    }
}

void test_clamp_before_uint8_conversion()
{
    std::array<std::uint8_t, k_rows> boundary{};
    boundary.fill(80);
    boundary[20] = 10;
    boundary[30] = 20;

    assert(smartcar::vision::cross_geometry::fill_boundary_from_rows(
        boundary.data(), k_rows, 20, 30, 0, 119, k_min_x, k_max_x));
    assert(boundary[0] == k_min_x);
    assert(boundary[119] == 109);

    boundary.fill(80);
    boundary[20] = 140;
    boundary[30] = 150;
    assert(smartcar::vision::cross_geometry::fill_boundary_from_rows(
        boundary.data(), k_rows, 20, 30, 0, 119, k_min_x, k_max_x));
    assert(boundary[119] == k_max_x);
}

void test_invalid_sources_do_not_modify_boundary()
{
    std::array<std::uint8_t, k_rows> boundary{};
    boundary.fill(60);
    const auto original = boundary;

    assert(!smartcar::vision::cross_geometry::fill_boundary_from_rows(
        boundary.data(), k_rows, 20, 20, 5, 117, k_min_x, k_max_x));
    assert(boundary == original);

    boundary[20] = 1;
    const auto lost_anchor = boundary;
    assert(!smartcar::vision::cross_geometry::fill_boundary_from_rows(
        boundary.data(), k_rows, 20, 30, 5, 117, k_min_x, k_max_x));
    assert(boundary == lost_anchor);
}
}

int main()
{
    test_vertical_boundary();
    test_clamp_before_uint8_conversion();
    test_invalid_sources_do_not_modify_boundary();
    return 0;
}
