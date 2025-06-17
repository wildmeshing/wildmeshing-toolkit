

#include "common.h"

tetwild::Vector3r tetwild::to_rational(const Vector3d& p0){
    Vector3r p(p0[0], p0[1], p0[2]);
    return p;
}

tetwild::Vector3d tetwild::to_double(const Vector3r& p0){
    Vector3d p(p0[0].to_double(), p0[1].to_double(), p0[2].to_double());
    return p;
}