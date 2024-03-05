#pragma once

#include "Quadrature.hpp"

namespace wmtk {

class LineQuadrature
{
public:
    LineQuadrature();

    void get_quadrature(const int order, Quadrature& quad);
};
} // namespace wmtk