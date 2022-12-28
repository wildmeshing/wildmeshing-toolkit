#include "bicubic_interpolation.h"

#include <spdlog/spdlog.h>
using namespace wmtk;
wmtk::BicubicVector wmtk::extract_samples(
    const size_t width,
    const size_t height,
    const std::vector<float>& buffer,
    const float sx_,
    const float sy_)
{
    wmtk::BicubicVector samples;

    const auto get_buffer_value =
        [&width, &height, &buffer](const size_t xx, const size_t yy) -> float {
        const size_t index = (yy % height) * width + (xx % width);
        return buffer[index];
    };

    const auto sx = static_cast<size_t>(sx_);
    const auto sy = static_cast<size_t>(sy_);
    assert(static_cast<float>(sx) <= sx_);
    assert(static_cast<float>(sy) <= sy_);

    samples(0) = get_buffer_value(sx + width - 1, sy + height - 1);
    samples(1) = get_buffer_value(sx, sy + height - 1);
    samples(2) = get_buffer_value(sx + 1, sy + height - 1);
    samples(3) = get_buffer_value(sx + 2, sy + height - 1);

    samples(4) = get_buffer_value(sx + width - 1, sy);
    samples(5) = get_buffer_value(sx, sy);
    samples(6) = get_buffer_value(sx + 1, sy);
    samples(7) = get_buffer_value(sx + 2, sy);

    samples(8) = get_buffer_value(sx + width - 1, sy + 1);
    samples(9) = get_buffer_value(sx, sy + 1);
    samples(10) = get_buffer_value(sx + 1, sy + 1);
    samples(11) = get_buffer_value(sx + 2, sy + 1);

    samples(12) = get_buffer_value(sx + width - 1, sy + 2);
    samples(13) = get_buffer_value(sx, sy + 2);
    samples(14) = get_buffer_value(sx + 1, sy + 2);
    samples(15) = get_buffer_value(sx + 2, sy + 2);

    return samples;
}

wmtk::BicubicMatrix wmtk::make_samples_to_bicubic_coeffs_operator()
{
    wmtk::BicubicMatrix ope;
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
        spdlog::debug("ope det {}\n{}", ope.determinant(), ss.str());
    }

    // invert operator
    wmtk::BicubicMatrix ope_inv = ope.inverse();

    // prune "zeros"
    ope_inv = ope_inv.unaryExpr([](const float& xx) { return fabs(xx) < 1e-5f ? 0 : xx; });

    {
        std::stringstream ss;
        ss << ope_inv << std::endl;
        spdlog::debug("ope_inv det {}\n{}", ope_inv.determinant(), ss.str());
    }

    // double check inverse property
    assert((ope * ope_inv - wmtk::BicubicMatrix::Identity()).array().abs().maxCoeff() < 1e-5);

    return ope_inv;
}

float wmtk::eval_bicubic_coeffs(const wmtk::BicubicVector& coeffs, const float sx, const float sy)
{
    const auto xx = sx - static_cast<size_t>(sx);
    const auto yy = sy - static_cast<size_t>(sy);
    assert(0 <= xx && xx < 1);
    assert(0 <= yy && yy < 1);

    wmtk::BicubicVector vv;

    vv(0) = 1;
    vv(1) = xx;
    vv(2) = xx * xx;
    vv(3) = xx * xx * xx;

    vv(4) = yy;
    vv(5) = xx * yy;
    vv(6) = xx * xx * yy;
    vv(7) = xx * xx * xx * yy;

    vv(8) = yy * yy;
    vv(9) = xx * yy * yy;
    vv(10) = xx * xx * yy * yy;
    vv(11) = xx * xx * xx * yy * yy;

    vv(12) = yy * yy * yy;
    vv(13) = xx * yy * yy * yy;
    vv(14) = xx * xx * yy * yy * yy;
    vv(15) = xx * xx * xx * yy * yy * yy;

    return coeffs.transpose() * vv;
}
