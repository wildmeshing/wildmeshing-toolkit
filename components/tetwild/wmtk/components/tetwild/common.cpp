#include "common.h"

namespace wmtk::components::tetwild {

Vector3r to_rational(const Vector3d& p0)
{
    Vector3r p(p0[0], p0[1], p0[2]);
    return p;
}

Vector3d to_double(const Vector3r& p0)
{
    Vector3d p(p0[0].to_double(), p0[1].to_double(), p0[2].to_double());
    return p;
}

} // namespace wmtk::components::tetwild