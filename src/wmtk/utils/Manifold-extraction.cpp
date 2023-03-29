#include "Manifold-extraction.hpp"

namespace wmtk {
    auto pntgen() -> const std::vector<wmtk::Point2D>&{

        static size_t nb_points = 10;
        static std::vector<Point2D> points(nb_points);
        
        // seed of random gen is 10 
        std::mt19937 gen(10);
        std::uniform_real_distribution<double> dis(0.0, 10.0);

        for (size_t i = 0; i < nb_points; ++i) {
            // generate 2 random integers between 1 and 10
            for (auto j = 0; j < 2; j++) {
                points[i][j] = dis(gen);  //(std::rand() % 10 + 1);
            }
        }
        return points;
    }

    auto tagassign(std::vector<Triangle> triangles)
    -> std::map<size_t, size_t>&{
        static std::map<size_t, size_t> tagass;
        return tagass;
    }
}