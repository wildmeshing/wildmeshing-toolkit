#include "Manifold-extraction.hpp"

namespace wmtk {
    auto pntgen2d(const size_t nb_points, const double range) -> std::vector<wmtk::Point2D>{
        std::vector<Point2D> points(nb_points);
        //auto points = std::make_unique<std::vector<Point2D>>(nb_points);
        std::random_device rd{};
        std::mt19937 gen(rd());
        // seed of random gen is 10  //std::mt19937 gen(10);
        std::uniform_real_distribution<double> dis(0.0, range);
        for (size_t i = 0; i < nb_points; ++i) {
            // generate 3 random doubles between 0 and the given range
            for (auto j = 0; j < 2; j++) {
                points[i][j] = dis(gen);  //(std::rand() % 10 + 1);
            }
        }
        return points;
    }

    auto pntgen3d(size_t nb_points, double range) -> std::vector<wmtk::Point3D>{
        std::vector<Point3D> points(nb_points);
        std::random_device rd{};
        std::mt19937 gen(rd());
        // seed of random gen is 10  //std::mt19937 gen(10);
        std::uniform_real_distribution<double> dis(0.0, range);
        for (size_t i = 0; i < nb_points; ++i) {
            // generate 3 random doubles between 0 and the given range
            for (auto j = 0; j < 3; j++) {
                points[i][j] = dis(gen);  //(std::rand() % 10 + 1);
            }
        }
        return points;
    }

    auto tagassign(size_t nb_triangles) -> std::vector<size_t>{
        std::vector<size_t> tagass;
        std::srand(10);
        for (size_t i = 0 ; i < nb_triangles; i++){
            size_t tag = std::rand() % 2;
            if (tag == 1) tagass.push_back(i);
        }
        return tagass;
    }
}