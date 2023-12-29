#include "bicubic_interpolation.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::adaptive_tessellation::image {
BicubicVector<float> extract_samples(
    const size_t width,
    const size_t height,
    const float* buffer,
    const double sx_,
    const double sy_,
    const IMAGE_WRAPPING_MODE mode_x,
    const IMAGE_WRAPPING_MODE mode_y)
{
    BicubicVector<float> samples;

    const auto get_coordinate =
        [](const int x, const int size, const IMAGE_WRAPPING_MODE mode) -> int {
        switch (mode) {
        case IMAGE_WRAPPING_MODE::REPEAT: return (x + size) % size;

        case IMAGE_WRAPPING_MODE::MIRROR_REPEAT:
            if (x < 0)
                return -(x % size);
            else if (x < size)
                return x;
            else
                return size - 1 - (x - size) % size;
        case IMAGE_WRAPPING_MODE::CLAMP_TO_EDGE: return std::clamp(x, 0, size - 1);
        default: return (x + size) % size;
        }
    };
    const auto get_buffer_value = [&](int xx, int yy) -> float {
        xx = get_coordinate(xx, static_cast<int>(width), mode_x);
        yy = get_coordinate(yy, static_cast<int>(height), mode_y);
        const int index = (yy % height) * width + (xx % width);
        return buffer[index];
    };

    const auto sx = static_cast<int>(std::floor(sx_ - 0.5));
    const auto sy = static_cast<int>(std::floor(sy_ - 0.5));

    samples(0) = get_buffer_value(sx - 1, sy - 1);
    samples(1) = get_buffer_value(sx, sy - 1);
    samples(2) = get_buffer_value(sx + 1, sy - 1);
    samples(3) = get_buffer_value(sx + 2, sy - 1);

    samples(4) = get_buffer_value(sx - 1, sy);
    samples(5) = get_buffer_value(sx, sy);
    samples(6) = get_buffer_value(sx + 1, sy);
    samples(7) = get_buffer_value(sx + 2, sy);

    samples(8) = get_buffer_value(sx - 1, sy + 1);
    samples(9) = get_buffer_value(sx, sy + 1);
    samples(10) = get_buffer_value(sx + 1, sy + 1);
    samples(11) = get_buffer_value(sx + 2, sy + 1);

    samples(12) = get_buffer_value(sx - 1, sy + 2);
    samples(13) = get_buffer_value(sx, sy + 2);
    samples(14) = get_buffer_value(sx + 1, sy + 2);
    samples(15) = get_buffer_value(sx + 2, sy + 2);

    return samples;
}

BicubicMatrix make_samples_to_bicubic_coeffs_operator()
{
    BicubicMatrix ope;
    Eigen::Index row = 0;
    for (float yy = -1; yy < 3; yy++)
        for (float xx = -1; xx < 3; xx++) {
            ope(row, 0) = 1;
            ope(row, 1) = xx;
            ope(row, 2) = xx * xx;
            ope(row, 3) = xx * xx * xx;

            ope(row, 4) = yy;
            ope(row, 5) = xx * yy;
            ope(row, 6) = xx * xx * yy;
            ope(row, 7) = xx * xx * xx * yy;

            ope(row, 8) = yy * yy;
            ope(row, 9) = xx * yy * yy;
            ope(row, 10) = xx * xx * yy * yy;
            ope(row, 11) = xx * xx * xx * yy * yy;

            ope(row, 12) = yy * yy * yy;
            ope(row, 13) = xx * yy * yy * yy;
            ope(row, 14) = xx * xx * yy * yy * yy;
            ope(row, 15) = xx * xx * xx * yy * yy * yy;

            row++;
        }

    {
        std::stringstream ss;
        ss << ope << std::endl;
        logger().debug("ope det {}\n{}", ope.determinant(), ss.str());
    }

    // invert operator
    BicubicMatrix ope_inv = ope.inverse();

    // prune "zeros"
    ope_inv = ope_inv.unaryExpr([](const float& xx) { return fabs(xx) < 1e-5f ? 0 : xx; });

    {
        std::stringstream ss;
        ss << ope_inv << std::endl;
        logger().debug("ope_inv det {}\n{}", ope_inv.determinant(), ss.str());
    }

    // double check inverse property
    assert((ope * ope_inv - BicubicMatrix::Identity()).array().abs().maxCoeff() < 1e-5);

    return ope_inv;
}

const BicubicMatrix& get_bicubic_matrix()
{
    static BicubicMatrix mat = make_samples_to_bicubic_coeffs_operator();
    return mat;
}
} // namespace wmtk::components::adaptive_tessellation::image
