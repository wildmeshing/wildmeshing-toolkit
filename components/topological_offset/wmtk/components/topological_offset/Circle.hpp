#pragma once
#include <fstream>
#include <queue>
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {
class Circle
{
private:
    Vector2d m_c;
    double m_r;
    static void
    fit_square(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3, Vector2d& c, double& l);

public:
    Circle(const Vector2d& c, const double r)
        : m_c(c)
        , m_r(r)
    {}

    Circle(const TopoOffsetTriMesh& mesh, const size_t f_id);

    double radius() const { return m_r; }
    Vector2d center() const { return m_c; }
    void refine(std::queue<Circle>& q) const;
    bool overlaps_tri(const TopoOffsetTriMesh& mesh, const size_t f_id) const;

    // public: // debugging
    //     static void write_to_txt(const std::string& path, const std::vector<Circle>& circles)
    //     {
    //         std::ofstream file(path);
    //         if (!file.is_open()) {
    //             logger().error("Failed to open file: {}", path);
    //             return;
    //         }

    //         for (const Circle& c : circles) {
    //             file << c.center()(0) << ", " << c.center()(1) << ", " << c.radius() << "\n";
    //         }

    //         file.close();
    //         logger().info("Saved {} circles to {}", circles.size(), path);
    //     }
};


} // namespace wmtk::components::topological_offset