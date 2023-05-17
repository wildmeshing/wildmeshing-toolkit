#include "Image.h"

using namespace wmtk;
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

int Image::get_coordinate(const int x, const WrappingMode mode) const
{
    auto size = std::max(width(), height());
    assert(-size < x && x < 2 * size);
    switch (mode) {
    case WrappingMode::REPEAT: return (x + size) % size;

    case WrappingMode::MIRROR_REPEAT:
        if (x < 0)
            return -(x % size);
        else if (x < size)
            return x;
        else
            return size - 1 - (x - size) % size;

    case WrappingMode::CLAMP_TO_EDGE: return std::clamp(x, 0, size - 1);
    default: return (x + size) % size;
    }
}

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
bool Image::set(
    const std::function<float(const double&, const double&)>& f,
    WrappingMode mode_x,
    WrappingMode mode_y)
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
    set_wrapping_mode(mode_x, mode_y);
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
    if (path.extension() == ".hdr") {
        auto res = stbi_write_hdr(path.string().c_str(), w, h, 1, buffer.data());
        assert(res);
    } else if (path.extension() == ".exr") {
        auto res = save_image_exr_red_channel(w, h, buffer, path);
    } else {
        wmtk::logger().trace("[save_image_hdr] format doesn't support \"{}\"", path.string());
        return false;
    }

    wmtk::logger().trace("[save_image] done \"{}\"", path.string());

    return true;
}

// load from hdr or exr
void Image::load(
    const std::filesystem::path& path,
    const WrappingMode mode_x,
    const WrappingMode mode_y)
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
    set_wrapping_mode(mode_x, mode_y);
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

std::array<wmtk::Image, 3> wmtk::combine_position_normal_texture(
    double normalization_scale,
    const std::filesystem::path& position_path,
    const std::filesystem::path& normal_path,
    const std::filesystem::path& height_path)
{
    // displaced = positions + normalization_scale * heights * (2.0 * normals - 1.0)
    auto [w_p, h_p, index_red_p, index_green_p, index_blue_p, buffer_r_p, buffer_g_p, buffer_b_p] =
        load_image_exr_split_3channels(position_path);
    auto [w_n, h_n, index_red_n, index_green_n, index_blue_n, buffer_r_n, buffer_g_n, buffer_b_n] =
        load_image_exr_split_3channels(normal_path);
    auto [w_h, h_h, index_red_h, index_green_h, index_blue_h, buffer_r_h, buffer_g_h, buffer_b_h] =
        load_image_exr_split_3channels(height_path);
    assert(buffer_r_p.size() == buffer_r_n.size());
    assert(buffer_r_p.size() == buffer_r_h.size());
    assert(buffer_r_p.size() == buffer_g_p.size());
    assert(buffer_r_p.size() == buffer_b_p.size());
    auto buffer_size = buffer_r_p.size();
    auto w = w_p;
    auto h = h_p;
    std::vector<float> buffer_r_d(buffer_size), buffer_g_d(buffer_size), buffer_b_d(buffer_size);
    for (int i = 0; i < buffer_size; i++) {
        buffer_r_d[i] =
            buffer_r_p[i] + normalization_scale * buffer_r_h[i] * (2.0 * buffer_r_n[i] - 1.0);
        buffer_g_d[i] =
            buffer_g_p[i] + normalization_scale * buffer_g_h[i] * (2.0 * buffer_g_n[i] - 1.0);
        buffer_b_d[i] =
            buffer_b_p[i] + normalization_scale * buffer_b_h[i] * (2.0 * buffer_b_n[i] - 1.0);
    }
    // auto res = save_image_exr_3channels(
    //     w,
    //     h,
    //     index_red_p,
    //     index_green_p,
    //     index_blue_p,
    //     buffer_r_d,
    //     buffer_g_d,
    //     buffer_b_d,
    //     displaced_path);
    return {
        buffer_to_image(buffer_r_d, w, h),
        buffer_to_image(buffer_g_d, w, h),
        buffer_to_image(buffer_b_d, w, h),
    };
}