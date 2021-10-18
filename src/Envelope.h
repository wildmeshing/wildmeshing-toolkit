//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_ENVELOPE_H
#define WILDMESHING_TOOLKIT_ENVELOPE_H
#include "common.h"

namespace wmtk{
    class Envelope{
        double get_point_dist(const Vector3f& p);
        double get_segment_dist(const Vector3f& p1, const Vector3f& p2);
        double get_triangle_dist(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3);

        void sample_a_segment(
                //input:
                const Vector3f& p1, const Vector3f& p2,
                //output:
                std::vector<Vector3f>& ps);

        void sample_a_triangle(
                //input:
                const Vector3f& p1, const Vector3f& p2, const Vector3f& p3,
                //output:
                std::vector<Vector3f>& ps);
    };
}
#endif //WILDMESHING_TOOLKIT_ENVELOPE_H
