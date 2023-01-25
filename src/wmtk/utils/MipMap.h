#pragma once
#include <Eigen/Dense>
#include "Image.h"
#include <memory>
namespace wmtk {

class MipMap
{
protected:
    Image m_original_image;
    std::vector<std::unique_ptr<Image>> m_mipmap;
protected:
    MipMap(Image& m_original_image) {
        construct_mipmap();   
    };
public: 
    int level() {return m_mipmap.size();};
    void construct_mipmap() {};
    std::unique_ptr<Image> get_image_ptr() {};
};
}