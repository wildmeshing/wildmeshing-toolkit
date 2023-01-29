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

public:
    int level() { return m_image_hierarchy.size(); };
    int get_mipmap_level(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
    inline const Image& get_image(int idx) const { return m_image_hierarchy[idx]; };
};
} // namespace wmtk