#include "Image.hpp"

#include <stb_image.h>
#include <stb_image_write.h>
#include <wmtk/utils/Logger.hpp>
#include "utils/load_image_exr.hpp"
#include "utils/save_image_exr.hpp"

namespace wmtk::components::image {
namespace {
float modulo(double x, double n)
{
    float y = fmod(x, n);
    if (y < 0) {
        y += n;
    }
    return y;
}

unsigned char double_to_unsignedchar(const double d)
{
    return round(std::max(std::min(1., d), 0.) * 255);
}
} // namespace

std::pair<int, int> Image::get_pixel_index(const double& u, const double& v) const
{
    int w = width();
    int h = height();
    auto size = std::max(w, h);
    // x, y are between 0 and 1
    auto x = u * size;
    auto y = v * size;
    const auto sx = static_cast<int>(std::floor(x - 0.5));
    const auto sy = static_cast<int>(std::floor(y - 0.5));

    return {sx, sy};
}

// set an image to have same value as the analytical function and save it to the file given
bool Image::set(const std::function<float(const double&, const double&)>& f)
{
    int h = height();
    int w = width();

    m_image.resize(h, w);

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            double u, v;
            u = (static_cast<double>(j) + 0.5) / static_cast<double>(w);
            v = (static_cast<double>(i) + 0.5) / static_cast<double>(h);
            m_image(i, j) = f(u, v);
        }
    }
    // set_wrapping_mode(mode_x, mode_y);
    return true;
}

// save to hdr or exr
bool Image::save(const std::filesystem::path& path) const
{
    wmtk::logger().trace("[save_image_hdr] start \"{}\"", path.string());
    int w = width();
    int h = height();
    std::vector<float> buffer;
    buffer.resize(w * h);

    for (auto i = 0; i < h; i++) {
        for (auto j = 0; j < w; j++) {
            buffer[i * w + j] = m_image(i, j);
        }
    }
    // if (path.extension() == ".hdr") {
    //     auto res = stbi_write_hdr(path.string().c_str(), w, h, 1, buffer.data());
    //     assert(res);
    // } else
    if (path.extension() == ".exr") {
        auto res = save_image_exr_red_channel(w, h, buffer, path);
    } else {
        wmtk::logger().trace("[save_image_hdr] format doesn't support \"{}\"", path.string());
        return false;
    }

    wmtk::logger().trace("[save_image] done \"{}\"", path.string());

    return true;
}

// load from hdr or exr
void Image::load(const std::filesystem::path& path)
{
    int w, h, channels;
    channels = 1;
    std::vector<float> buffer;
    if (path.extension() == ".exr") {
        std::tie(w, h, buffer) = load_image_exr_red_channel(path);
        assert(!buffer.empty());
    } else if (path.extension() == ".hdr") {
        auto res = stbi_loadf(path.string().c_str(), &w, &h, &channels, 1);
        buffer.assign(res, res + w * h);
    } else {
        wmtk::logger().trace("[load_image] format doesn't support \"{}\"", path.string());
        return;
    }

    m_image.resize(w, h);

    for (int i = 0, k = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            m_image(i, j) = buffer[k++];
        }
    }
    m_image.colwise().reverseInPlace();
}

// down sample a image to size/2 by size/2
// used for mipmap construction
Image Image::down_sample() const
{
    auto h = height();
    auto w = width();
    Image low_res_image(h / 2, w / 2);
    for (int r = 0; r < h / 2; r++) {
        for (int c = 0; c < w / 2; c++) {
            low_res_image.set(
                r,
                c,
                (m_image(r * 2, c * 2) + m_image(r * 2, c * 2 + 1) + m_image(r * 2 + 1, c * 2) +
                 m_image(r * 2 + 1, c * 2 + 1)) /
                    4.);
        }
    }
    return low_res_image;
}

std::array<Image, 3> combine_position_normal_texture(
    double normalization_scale,
    const Eigen::Matrix<double, 1, 3>& offset,
    const std::filesystem::path& position_path,
    const std::filesystem::path& normal_path,
    const std::filesystem::path& height_path,
    float min_height,
    float max_height)
{
    assert(std::filesystem::exists(position_path));
    auto [w_p, h_p, index_red_p, index_green_p, index_blue_p, buffer_r_p, buffer_g_p, buffer_b_p] =
        load_image_exr_split_3channels(position_path);

    auto buffer_size = buffer_r_p.size();
    std::vector<float> buffer_r_d(buffer_size);
    std::vector<float> buffer_g_d(buffer_size);
    std::vector<float> buffer_b_d(buffer_size);

    if (std::filesystem::exists(normal_path) && std::filesystem::exists(height_path)) {
        // Load normal + heightmap and compute displaced positions.
        auto
            [w_n,
             h_n,
             index_red_n,
             index_green_n,
             index_blue_n,
             buffer_r_n,
             buffer_g_n,
             buffer_b_n] = load_image_exr_split_3channels(normal_path);
        auto
            [w_h,
             h_h,
             index_red_h,
             index_green_h,
             index_blue_h,
             buffer_r_h,
             buffer_g_h,
             buffer_b_h] = load_image_exr_split_3channels(height_path);
        assert(buffer_r_p.size() == buffer_r_n.size());
        assert(buffer_r_p.size() == buffer_r_h.size());
        assert(buffer_r_p.size() == buffer_g_p.size());
        assert(buffer_r_p.size() == buffer_b_p.size());
        auto scale = [&](float h) { return min_height * (1.f - h) + max_height * h; };
        // displaced = positions * normalization_scale + heights * (2.0 * normals - 1.0) - offset
        for (int i = 0; i < buffer_size; i++) {
            buffer_r_d[i] = buffer_r_p[i] * normalization_scale +
                            scale(buffer_r_h[i]) * (2.0 * buffer_r_n[i] - 1.0) - offset[0];
            buffer_g_d[i] = buffer_g_p[i] * normalization_scale +
                            scale(buffer_g_h[i]) * (2.0 * buffer_g_n[i] - 1.0) - offset[1];
            buffer_b_d[i] = buffer_b_p[i] * normalization_scale +
                            scale(buffer_b_h[i]) * (2.0 * buffer_b_n[i] - 1.0) - offset[2];
        }
    } else {
        // Missing heightmap info: we use the position map as our displaced coordinates.
        wmtk::logger().info("No heightmap provided: using positions as displaced coordinates.");
        // displaced = positions * normalization_scale - offset
        for (int i = 0; i < buffer_size; i++) {
            buffer_r_d[i] = buffer_r_p[i] * normalization_scale - offset[0];
            buffer_g_d[i] = buffer_g_p[i] * normalization_scale - offset[1];
            buffer_b_d[i] = buffer_b_p[i] * normalization_scale - offset[2];
        }
    }

    return {{
        buffer_to_image(buffer_r_d, w_p, h_p),
        buffer_to_image(buffer_g_d, w_p, h_p),
        buffer_to_image(buffer_b_d, w_p, h_p),
    }};
}

void split_and_save_3channels(const std::filesystem::path& path)
{
    int w, h, channels, index_red, index_blue, index_green;
    channels = 1;
    std::vector<float> buffer_r, buffer_g, buffer_b;
    if (path.extension() == ".exr") {
        std::tie(w, h, index_red, index_green, index_blue, buffer_r, buffer_g, buffer_b) =
            load_image_exr_split_3channels(path);
        assert(!buffer_r.empty());
        assert(!buffer_g.empty());
        assert(!buffer_b.empty());
    } else {
        logger().error("[split_image] format doesn't support \"{}\"", path.string());
        return;
    }
    const std::filesystem::path directory = path.parent_path();
    const std::string file = path.stem().string();
    const std::filesystem::path path_r = directory / (file + "_r.exr");
    const std::filesystem::path path_g = directory / (file + "_g.exr");
    const std::filesystem::path path_b = directory / (file + "_b.exr");
    // just saves single channel data to red channel
    auto res = save_image_exr_red_channel(w, h, buffer_r, path_r);
    assert(res);
    res = save_image_exr_red_channel(w, h, buffer_g, path_g);
    assert(res);
    res = save_image_exr_red_channel(w, h, buffer_b, path_b);
    assert(res);
}

Image buffer_to_image(const std::vector<float>& buffer, int w, int h)
{
    Image image(w, h);
    for (int i = 0, k = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            image.set(h - i - 1, j, buffer[k++]);
        }
    }
    return image;
}

std::array<Image, 3> load_rgb_image(const std::filesystem::path& path)
{
    int w, h, channels, index_red, index_blue, index_green;
    channels = 1;
    std::vector<float> buffer_r, buffer_g, buffer_b;
    if (path.extension() == ".exr") {
        std::tie(w, h, index_red, index_green, index_blue, buffer_r, buffer_g, buffer_b) =
            load_image_exr_split_3channels(path);
        assert(!buffer_r.empty());
        assert(!buffer_g.empty());
        assert(!buffer_b.empty());
    } else {
        wmtk::logger().error("[load_rgb_image] format doesn't support \"{}\"", path.string());
        exit(-1);
    }
    return {{
        buffer_to_image(buffer_r, w, h),
        buffer_to_image(buffer_g, w, h),
        buffer_to_image(buffer_b, w, h),
    }};
}
} // namespace wmtk::components::image