#pragma once
#include <cstdint>

namespace wmtk::autogen {
    class SimplexDart;
}
namespace wmtk::autogen::utils{


    int8_t edge_mirror(const SimplexDart& sd, int8_t orientation, int8_t edge_dart);
    
}
