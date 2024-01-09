#pragma once

#include <wmtk/function/utils/autodiff.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
#include <type_traits>
#include <wmtk/function/utils/AutoDiffUtils.hpp>

namespace wmtk::components::adaptive_tessellation::image {
/**
 * @brief this is a data structure for a single channel image
 *
 */
class Image
{
    using ImageMatrixf =
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Eigen::AutoAlign>;

protected:
    ImageMatrixf m_image; // saving scanline images

public:
    Image() = default;
    Image(int height_, int width_) { m_image.resize(height_, width_); };

    ImageMatrixf& ref_raw_image() { return m_image; }
    const ImageMatrixf& get_raw_image() const { return m_image; }

public:
    // point coordinates between [0, 1]
    int width() const { return static_cast<int>(m_image.cols()); };
    int height() const { return static_cast<int>(m_image.rows()); };
    template <typename T>
    float operator()(const T& u, const T& v) const
    {
        auto [i, j] = get_pixel_index(static_cast<double>(u), static_cast<double>(v));
        return m_image(i, j);
    }
    float operator()(const int i, const int j) const { return m_image(i, j); };
    std::pair<int, int> get_pixel_index(const double& u, const double& v) const;
    bool set(const std::function<float(const double&, const double&)>& f);
    bool set(const int r, const int c, const float v)
    {
        m_image(r, c) = v;
        return true;
    };
    bool save(const std::filesystem::path& path) const;
    void load(const std::filesystem::path& path);

    Image down_sample() const;
};

void split_and_save_3channels(const std::filesystem::path& path);

Image buffer_to_image(const std::vector<float>& buffer, int w, int h);

std::array<Image, 3> load_rgb_image(const std::filesystem::path& path);

std::array<Image, 3> combine_position_normal_texture(
    double normalization_scale,
    const Eigen::Matrix<double, 1, 3>& offset,
    const std::filesystem::path& position_path,
    const std::filesystem::path& normal_path,
    const std::filesystem::path& texture_path,
    float min_height = 0.f,
    float max_height = 1.f);
} // namespace wmtk::components::adaptive_tessellation::image
