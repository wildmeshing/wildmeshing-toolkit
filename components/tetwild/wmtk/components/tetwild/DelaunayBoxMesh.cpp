#include <wmtk/utils/DelaunayBoxMesh.hpp>
#include <wmtk/utils/Rational.hpp>
#include "TetWildMesh.h"

#include <bitset>
#include <wmtk/utils/Delaunay.hpp>
#include "wmtk/TetMesh.h"
#include "wmtk/TetMeshCutTable.hpp"
#include "wmtk/utils/GeoUtils.h"
#include "wmtk/utils/InsertTriangleUtils.hpp"
#include "wmtk/utils/Logger.hpp"

#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/threading/task_group.hpp>

#include <random>
#include <unordered_set>

namespace wmtk::components::tetwild {

void TetWildMesh::init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices)
{
    ///points for delaunay
    std::vector<wmtk::delaunay::Point3D> points(vertices.size());
    // add points from surface
    for (int i = 0; i < vertices.size(); i++) {
        for (int j = 0; j < 3; j++) points[i][j] = vertices[i][j];
    }

    // The box is grown from the ORIGINAL input bbox (m_tet_params.min/max), not from `vertices`,
    // which are the SIMPLIFIED surface -- so it is deliberately larger than the point set
    // being triangulated. m_tet_params.box_min/box_max then hold the padded box, which is what
    // bbox-face tagging compares vertex coordinates against.
    std::vector<wmtk::delaunay::Tetrahedron> tets;
    Vector3d box_min, box_max;
    wmtk::utils::delaunay_box_mesh(
        *m_envelope,
        m_tet_params.min,
        m_tet_params.max,
        m_params.diag_l,
        points,
        tets,
        box_min,
        box_max);

    m_tet_params.box_min = box_min;
    m_tet_params.box_max = box_max;

    // conn
    init(points.size(), tets);
    logger().info("init finished");
    // attr
    m_vertex_attribute.resize(points.size());
    m_tet_attribute.resize(tets.size());
    m_face_attribute.resize(tets.size() * 4);
    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = Vector3r(points[i][0], points[i][1], points[i][2]);
        m_vertex_attribute[i].m_posf = Vector3d(points[i][0], points[i][1], points[i][2]);
    }
    logger().info("attribute vectors created");
}

} // namespace wmtk::components::tetwild