#include "Optimization.hpp"
#include <wmtk/SimplicialComplex.hpp>

using namespace wmtk;


Optimization::Optimization(
    FunctionInterface& function_interface,
    const TriMesh& m,
    const InvariantCollection& invariants,
    const bool second_order,
    const bool line_search)
    : m_function_interface(function_interface)
    , m_mesh(m)
    , m_invariants(invariants)
    , m_second_order(second_order)
    , m_line_search(line_search)
{}

Optimization::Optimization(
    const Tuple& tuple,
    Accessor<double>& accessor,
    const function::DifferentiableFunction& function,
    const TriMesh& m,
    const InvariantCollection& invariants,
    const bool second_order,
    const bool line_search)
    : m_function_interface(tuple, accessor, function)
    , m_mesh(m)
    , m_invariants(invariants)
    , m_second_order(second_order)
    , m_line_search(line_search)
{}

void Optimization::optimize2d(const Eigen::Vector2d& current_pos)
{
    Eigen::Vector2d new_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d search_direction = Eigen::Vector2d::Zero();
    if (m_second_order) {
        // solve optimization problem
        search_direction = -(m_function_interface.hess()).ldlt().solve(m_function_interface.grad());
    } else {
        // solve optimization problem
        search_direction = -m_function_interface.grad();
    }
    new_pos = current_pos + search_direction;
    m_function_interface.store(new_pos);
    double step_size = 1.0;
    // get the modified primitives and convert to tuple manully
    auto one_ring = SimplicialComplex::vertex_one_ring(m_mesh, m_function_interface.m_tuple);
    std::vector<Tuple> modified_faces;
    for (const auto& s : one_ring) {
        modified_faces.emplace_back(s.tuple());
    }
    while (m_line_search && !m_invariants.after(PrimitiveType::Face, modified_faces)) {
        step_size /= 2;
        new_pos = current_pos + search_direction * step_size;
        m_function_interface.store(new_pos);
    }
}