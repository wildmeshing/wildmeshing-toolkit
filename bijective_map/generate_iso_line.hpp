#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace Eigen;

struct Intersection
{
    int fid;
    Vector3d barycentric;
    Vector2d point;
};

std::vector<Intersection> computeIntersections(const MatrixXd& uv, const MatrixXi& F, double x0)
{
    std::vector<Intersection> intersections;

    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v0 = F(i, j);
            int v1 = F(i, (j + 1) % 3);
            int v2 = F(i, (j + 2) % 3);

            Vector2d p0 = uv.row(v0);
            Vector2d p1 = uv.row(v1);

            if ((p0.x() <= x0 && p1.x() >= x0) || (p0.x() >= x0 && p1.x() <= x0)) {
                double t = (x0 - p0.x()) / (p1.x() - p0.x());
                double y = p0.y() + t * (p1.y() - p0.y());

                Vector3d barycentric;
                barycentric[j] = 1 - t;
                barycentric[(j + 1) % 3] = t;
                barycentric[(j + 2) % 3] = 0;

                Vector2d intersectionPoint(x0, y);

                intersections.push_back({i, barycentric, intersectionPoint});
            }
        }
    }

    std::sort(
        intersections.begin(),
        intersections.end(),
        [](const Intersection& a, const Intersection& b) { return a.point.y() < b.point.y(); });

    // some sanity checks
    if (intersections.size() % 2 != 0) {
        std::cerr << "Error: odd number of intersections!" << std::endl;
    }

    for (int i = 0; i < intersections.size() - 2; i += 2) {
        if (intersections[i + 1].fid != intersections[i].fid) {
            if (intersections[i + 2].fid == intersections[i].fid) {
                std::swap(intersections[i + 1], intersections[i + 2]);
            } else {
                std::cerr << "Error: intersections are not ordered correctly!" << std::endl;
            }
        }
    }

    return intersections;
}
