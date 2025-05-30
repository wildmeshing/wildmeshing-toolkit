#include "Embedding.hpp"

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include "SubMesh.hpp"

namespace wmtk::submesh {

Embedding::Embedding(const std::shared_ptr<Mesh>& mesh)
    : m_mesh(mesh)
{
    m_mesh->m_embedding = this;

    m_tag_attribute_name[PrimitiveType::Vertex] = "WMTK_submesh_tag_v";
    m_tag_attribute_name[PrimitiveType::Edge] = "WMTK_submesh_tag_e";
    m_tag_attribute_name[PrimitiveType::Triangle] = "WMTK_submesh_tag_f";
    m_tag_attribute_name[PrimitiveType::Tetrahedron] = "WMTK_submesh_tag_t";

    Mesh& m = *m_mesh;

    // register tag attributes
    for (const PrimitiveType& pt : utils::primitive_below(m.top_simplex_type())) {
        if (m.has_attribute<int64_t>(m_tag_attribute_name[pt], pt)) {
            log_and_throw_error(
                "Cannot create embedding. Mesh already has an attribute with name {}",
                m_tag_attribute_name[pt]);
        }

        m_tag_handle[pt] = m.register_attribute_typed<int64_t>(m_tag_attribute_name[pt], pt, 1);


        attribute::MeshAttributeHandle h(m, m_tag_handle.at(pt));

        // m_split_new[pt] = std::make_shared<operations::SplitNewAttributeStrategy<int64_t>>(h);
        // auto& split_new_strat = *(m_split_new[pt]);
        //
        // split_new_strat.set_strategy(operations::SplitBasicStrategy::Copy);
        // split_new_strat.set_rib_strategy(operations::SplitRibBasicStrategy::None);
        //
        // m_collapse_new[pt] =
        // std::make_shared<operations::CollapseNewAttributeStrategy<int64_t>>(h); auto&
        // collapse_new_strat = *(m_collapse_new[pt]);
        //
        // auto collapse_new_func = [](const VectorX<int64_t>& a,
        //                             const VectorX<int64_t>& b,
        //                             const std::bitset<2>&) -> VectorX<int64_t> {
        //     VectorX<int64_t> r(a.rows());
        //
        //     assert(a.rows() == b.rows());
        //     assert(a.rows() == r.rows());
        //
        //     for (int64_t i = 0; i < a.rows(); ++i) {
        //         r[i] = a[i] | b[i];
        //     }
        //
        //     return r;
        // };
        // collapse_new_strat.set_strategy(collapse_new_func);
    }


    m_substructure_predicate = [this](const simplex::Simplex& s) -> bool {
        return simplex_is_in_submesh(s);
    };

    auto update_tag_func = [this](
                               const Eigen::MatrixX<int64_t>& P,
                               const std::vector<Tuple>& tuples) -> Eigen::VectorX<int64_t> {
        // transfer from vertices (P.cols()) to top_simplex
        assert(P.rows() == 1); // rows --> attribute dimension
        // cols --> number of input simplices (vertices)

        const simplex::Simplex cell(m_mesh->top_simplex_type(), tuples[0]);

        assert(cell.primitive_type() != PrimitiveType::Vertex);

        // transfer from cell to facets
        int64_t cell_tag;
        {
            auto tag_cell_acc = tag_accessor(cell.primitive_type());
            cell_tag = tag_cell_acc.const_scalar_attribute(cell);

            auto tag_facet_acc = tag_accessor(cell.primitive_type() - 1);
            const auto facets =
                simplex::faces_single_dimension_tuples(*m_mesh, cell, cell.primitive_type() - 1);

            for (const Tuple& f : facets) {
                tag_facet_acc.scalar_attribute(f) |= cell_tag;
            }
        }

        if (cell.primitive_type() != PrimitiveType::Edge) {
            // cell is triangle or tet
            for (const PrimitiveType& pt :
                 utils::primitive_range(cell.primitive_type() - 1, PrimitiveType::Edge)) {
                auto s_acc = tag_accessor(pt);
                auto f_acc = tag_accessor(pt - 1);
                const auto simplices = simplex::faces_single_dimension(*m_mesh, cell, pt);
                for (const simplex::Simplex& s : simplices) {
                    const auto faces = simplex::faces_single_dimension_tuples(*m_mesh, s, pt - 1);
                    for (const Tuple& f : faces) {
                        f_acc.scalar_attribute(f) |= s_acc.const_scalar_attribute(s);
                    }
                }
            }
        }

        return Eigen::VectorX<int64_t>::Constant(1, cell_tag);
    };

    attribute::MeshAttributeHandle h_v(m, m_tag_handle.at(PrimitiveType::Vertex));
    attribute::MeshAttributeHandle h_c(m, m_tag_handle.at(m.top_simplex_type()));

    // m_transfer =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>>(
    //         h_c,
    //         h_v,
    //         update_tag_func);
}

std::shared_ptr<SubMesh> Embedding::add_submesh()
{
    if (m_submeshes.size() == 63) {
        log_and_throw_error("An embedding can only hold up to 63 submeshes");
    }

    std::shared_ptr<SubMesh> sub = std::make_shared<SubMesh>(*this, m_submeshes.size());
    m_submeshes.emplace_back(sub);

    return sub;
}

Mesh& Embedding::mesh()
{
    return *m_mesh;
}

const Mesh& Embedding::mesh() const
{
    return *m_mesh;
}

attribute::TypedAttributeHandle<int64_t>& Embedding::tag_handle(const PrimitiveType pt)
{
    return m_tag_handle.at(pt);
}

attribute::Accessor<int64_t> Embedding::tag_accessor(const PrimitiveType pt)
{
    return mesh().create_accessor(m_tag_handle.at(pt));
}

const attribute::Accessor<int64_t> Embedding::tag_accessor(const PrimitiveType pt) const
{
    return mesh().create_const_accessor(m_tag_handle.at(pt));
}

std::vector<Tuple> Embedding::get_all(PrimitiveType type) const
{
    return mesh().get_all(type);
}

std::vector<simplex::IdSimplex> Embedding::get_all_id_simplex(PrimitiveType type) const
{
    return mesh().get_all_id_simplex(type);
}

int64_t Embedding::top_cell_dimension() const
{
    return mesh().top_cell_dimension();
}

MeshType Embedding::mesh_type() const
{
    return MeshType::Embedding;
}

Tuple Embedding::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    return mesh().switch_tuple(tuple, type);
}

bool Embedding::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    return mesh().is_boundary(pt, tuple);
}

int64_t Embedding::id(const simplex::Simplex& s) const
{
    return mesh().id(s);
}

int64_t Embedding::id(const Tuple& tuple, PrimitiveType pt) const
{
    return mesh().id(tuple, pt);
}

std::vector<std::shared_ptr<SubMesh>> Embedding::get_child_meshes() const
{
    return m_submeshes;
}

bool Embedding::has_child_mesh() const
{
    return !m_submeshes.empty();
}

bool Embedding::simplex_is_in_submesh(const simplex::Simplex& s) const
{
    const auto acc = tag_accessor(s.primitive_type());
    return acc.const_scalar_attribute(s.tuple()) > 0;
}

void Embedding::set_split_strategies(operations::EdgeSplit& split) const
{
    // for (const auto& [pt, strat] : m_split_new) {
    //     assert(m_mesh->validate_handle(m_tag_handle.at(pt)));
    //     attribute::MeshAttributeHandle h(*m_mesh, m_tag_handle.at(pt));
    //     split.set_new_attribute_strategy(h, strat);
    // }
    //
    // split.add_transfer_strategy(m_transfer);
}

void Embedding::set_collapse_strategies(operations::EdgeCollapse& collapse) const
{
    // for (const auto& [pt, strat] : m_collapse_new) {
    //     attribute::MeshAttributeHandle h(*m_mesh, m_tag_handle.at(pt));
    //     collapse.set_new_attribute_strategy(h, strat);
    // }
}

std::function<bool(const simplex::Simplex&)> Embedding::substructure_predicate() const
{
    return m_substructure_predicate;
}

void Embedding::update_tag_attribute_handles()
{
    Mesh& m = *m_mesh;

    for (const PrimitiveType& pt : utils::primitive_below(m.top_simplex_type())) {
        if (!m.has_attribute<int64_t>(m_tag_attribute_name[pt], pt)) {
            log_and_throw_error(
                "Cannot update handles. Mesh already no attribute with name {}",
                m_tag_attribute_name[pt]);
        }
        const auto& name = m_tag_attribute_name[pt];
        m_tag_handle[pt] = m.get_attribute_handle_typed<int64_t>(name, pt);


        attribute::MeshAttributeHandle h(m, m_tag_handle.at(pt));

        // m_split_new[pt] = std::make_shared<operations::SplitNewAttributeStrategy<int64_t>>(h);
        // auto& split_new_strat = *(m_split_new[pt]);
        //
        // split_new_strat.set_strategy(operations::SplitBasicStrategy::Copy);
        // split_new_strat.set_rib_strategy(operations::SplitRibBasicStrategy::None);
        //
        // m_collapse_new[pt] =
        // std::make_shared<operations::CollapseNewAttributeStrategy<int64_t>>(h); auto&
        // collapse_new_strat = *(m_collapse_new[pt]);
        //
        // auto collapse_new_func = [](const VectorX<int64_t>& a,
        //                             const VectorX<int64_t>& b,
        //                             const std::bitset<2>&) -> VectorX<int64_t> {
        //     VectorX<int64_t> r(a.rows());
        //
        //     assert(a.rows() == b.rows());
        //     assert(a.rows() == r.rows());
        //
        //     for (int64_t i = 0; i < a.rows(); ++i) {
        //         r[i] = a[i] | b[i];
        //     }
        //
        //     return r;
        // };
        // collapse_new_strat.set_strategy(collapse_new_func);
    }


    m_substructure_predicate = [this](const simplex::Simplex& s) -> bool {
        return simplex_is_in_submesh(s);
    };

    auto update_tag_func = [this](
                               const Eigen::MatrixX<int64_t>& P,
                               const std::vector<Tuple>& tuples) -> Eigen::VectorX<int64_t> {
        // transfer from vertices (P.cols()) to top_simplex
        assert(P.rows() == 1); // rows --> attribute dimension
        // cols --> number of input simplices (vertices)

        const simplex::Simplex cell(m_mesh->top_simplex_type(), tuples[0]);

        assert(cell.primitive_type() != PrimitiveType::Vertex);

        // transfer from cell to facets
        int64_t cell_tag;
        {
            auto tag_cell_acc = tag_accessor(cell.primitive_type());
            cell_tag = tag_cell_acc.const_scalar_attribute(cell);

            auto tag_facet_acc = tag_accessor(cell.primitive_type() - 1);
            const auto facets =
                simplex::faces_single_dimension_tuples(*m_mesh, cell, cell.primitive_type() - 1);

            for (const Tuple& f : facets) {
                tag_facet_acc.scalar_attribute(f) |= cell_tag;
            }
        }

        if (cell.primitive_type() != PrimitiveType::Edge) {
            // cell is triangle or tet
            for (const PrimitiveType& pt :
                 utils::primitive_range(cell.primitive_type() - 1, PrimitiveType::Edge)) {
                auto s_acc = tag_accessor(pt);
                auto f_acc = tag_accessor(pt - 1);
                const auto simplices = simplex::faces_single_dimension(*m_mesh, cell, pt);
                for (const simplex::Simplex& s : simplices) {
                    const auto faces = simplex::faces_single_dimension_tuples(*m_mesh, s, pt - 1);
                    for (const Tuple& f : faces) {
                        f_acc.scalar_attribute(f) |= s_acc.const_scalar_attribute(s);
                    }
                }
            }
        }

        return Eigen::VectorX<int64_t>::Constant(1, cell_tag);
    };

    attribute::MeshAttributeHandle h_v(m, m_tag_handle.at(PrimitiveType::Vertex));
    attribute::MeshAttributeHandle h_c(m, m_tag_handle.at(m.top_simplex_type()));

    // m_transfer =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>>(
    //         h_c,
    //         h_v,
    //         update_tag_func);

    for (const auto& [pt, h] : m_tag_handle) {
        assert(m.validate_handle(h));
    }
}


} // namespace wmtk::submesh
