#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "Image.h"
namespace wmtk {

class MipMap
{
public:
    MipMap() = default;
    MipMap(Image image);

protected:
    std::vector<Image> m_image_hierarchy;
    WrappingMode m_mipmap_wrapping_mode;

public:
    int level() const { return m_image_hierarchy.size(); };
    void set_wrapping_mode(const WrappingMode wrapping_mode)
    {
        m_mipmap_wrapping_mode = wrapping_mode;
    };
    std::pair<int, int> get_mipmap_level_pixelnum(
        const Eigen::Vector2d& a,
        const Eigen::Vector2d& b) const;
    inline const Image& get_image(int idx) const { return m_image_hierarchy[idx]; };
};
} // namespace wmtk