#include "MipMap.h"
using namespace wmtk;

MipMap::MipMap(Image image)
{
    int w = image.width();
    int h = image.height();
    // assume w=h and are power of 2
    auto size = floor(log2(std::max(w, h)));
    // arbitrarily the highest level mipmap will have have 4x4 resolution
    // therefore,  m_image_hierarchy.size() = (floor(log2(size)) - 2) + 1
    m_image_hierarchy.resize(size + 1);
    m_image_hierarchy[0] = std::move(image);

    for (int i = 0; i < size; i++) {
        m_image_hierarchy[i + 1] = m_image_hierarchy[i].down_sample();
        assert(m_image_hierarchy[i + 1].width() * 2 <= m_image_hierarchy[i].width());
        assert(m_image_hierarchy[i + 1].height() * 2 <= m_image_hierarchy[i].height());
    }
    assert(m_image_hierarchy[size].width() == 1);
    assert(m_image_hierarchy[size].height() == 1);
    int ref_h = h * 2;
}

// get the length between two points in the uv plane after projected usign displacement map
// get the appropriate mipmap pointer starting from the coarsiest to the finest
// stop when number of pixels between two points are more than 4
/* separate to 2 */
std::pair<int, int> MipMap::get_mipmap_level_pixelnum(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b) const
{
    auto levels = m_image_hierarchy.size();
    int idx = 0;
    int pixel_num = -1;
    for (int i = level() - 4; i >= 0; i--) {
        auto image = get_image(i);
        // get the pixel index of p1 and p2
        auto [x1, y1] = image.get_pixel_index(a(0), a(1));
        auto [x2, y2] = image.get_pixel_index(b(0), b(1));
        auto xx1 = image.get_coordinate(x1, m_mipmap_wrapping_mode);
        auto yy1 = image.get_coordinate(y1, m_mipmap_wrapping_mode);
        auto xx2 = image.get_coordinate(x2, m_mipmap_wrapping_mode);
        auto yy2 = image.get_coordinate(y2, m_mipmap_wrapping_mode);
        // get all the pixels in between p1 and p2
        pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
        if (pixel_num > 4) {
            idx = i;
            break;
        }
    }
    return {idx, pixel_num};
}