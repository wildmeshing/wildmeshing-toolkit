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

std::vector<Intersection>
computeIsoLineIntersectionsX(const MatrixXd& uv, const MatrixXi& F, double x0)
{
    double turb_eps = 1e-6;
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
                if (t == 0 || t == 1) {
                    return computeIsoLineIntersectionsX(uv, F, x0 + turb_eps);
                }
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

    for (int i = 0; i < int(intersections.size()) - 2; i += 2) {
        if (intersections[i + 1].fid != intersections[i].fid) {
            if (intersections[i + 2].fid == intersections[i].fid) {
                std::swap(intersections[i + 1], intersections[i + 2]);
            } else {
                std::cerr << "Error: intersections are not ordered correctly!" << std::endl;
                throw std::runtime_error("Error: intersections are not ordered correctly!");
            }
        }
    }

    return intersections;
}

std::vector<Intersection>
computeIsoLineIntersectionsY(const Eigen::MatrixXd& uv, const Eigen::MatrixXi& F, double y0)
{
    double turb_eps = 1e-6;
    std::vector<Intersection> intersections;

    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v0 = F(i, j);
            int v1 = F(i, (j + 1) % 3);
            int v2 = F(i, (j + 2) % 3);

            Eigen::Vector2d p0 = uv.row(v0);
            Eigen::Vector2d p1 = uv.row(v1);

            if ((p0.y() <= y0 && p1.y() >= y0) || (p0.y() >= y0 && p1.y() <= y0)) {
                double t = (y0 - p0.y()) / (p1.y() - p0.y());
                double x = p0.x() + t * (p1.x() - p0.x());

                Eigen::Vector3d barycentric;
                barycentric[j] = 1 - t;
                barycentric[(j + 1) % 3] = t;
                barycentric[(j + 2) % 3] = 0;
                if (t == 0 || t == 1) {
                    return computeIsoLineIntersectionsY(uv, F, y0 + turb_eps);
                }

                Eigen::Vector2d intersectionPoint(x, y0);

                intersections.push_back({i, barycentric, intersectionPoint});
            }
        }
    }

    std::sort(
        intersections.begin(),
        intersections.end(),
        [](const Intersection& a, const Intersection& b) { return a.point.x() < b.point.x(); });

    // some sanity checks
    if (intersections.size() % 2 != 0) {
        std::cerr << "Error: odd number of intersections!" << std::endl;
    }

    for (int i = 0; i < int(intersections.size()) - 2; i += 2) {
        if (intersections[i + 1].fid != intersections[i].fid) {
            if (intersections[i + 2].fid == intersections[i].fid) {
                std::swap(intersections[i + 1], intersections[i + 2]);
            } else {
                std::cerr << "Error: intersections are not ordered correctly!" << std::endl;
                throw std::runtime_error("Error: intersections are not ordered correctly!");
            }
        }
    }

    return intersections;
}
