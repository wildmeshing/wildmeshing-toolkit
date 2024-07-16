#pragma once
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen {
class SimplexDart
{
public:
    SimplexDart(wmtk::PrimitiveType simplex_type);

private:
    wmtk::PrimitiveType m_simplex_type;
};
} // namespace wmtk::autogen
