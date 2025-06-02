#include "Operation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/IdSimplexCollection.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>


// it's ugly but for teh visitor we need these included
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {


Operation::Operation(Mesh& mesh)
    : m_mesh(mesh)
    , m_invariants(mesh)
{}

Operation::~Operation() = default;

std::vector<simplex::Simplex> Operation::operator()(const simplex::Simplex& simplex)
{
    if (!mesh().is_valid(simplex)) {
        return {};
    }
    if (!before(simplex)) {
        return {};
    }

    assert(mesh().is_valid(simplex.tuple()));

    auto scope = mesh().create_scope();
    assert(simplex.primitive_type() == primitive_type());

    try {
#ifndef NDEBUG
        assert(!simplex.tuple().is_null());
        mesh().parent_scope([&]() { assert(mesh().is_valid(simplex.tuple())); });
#endif
        auto unmods = unmodified_primitives(simplex);
#ifndef NDEBUG
        for (const auto& s : unmods) {
            assert(!s.tuple().is_null());
        }
        for (const auto& s : unmods) {
            mesh().parent_scope([&]() { assert(mesh().is_valid(s.tuple())); });
        }
#endif
        auto mods = execute(simplex);
#ifndef NDEBUG
        if (!mesh().is_free()) {
            for (const auto& s : mods) {
                assert(mesh().is_valid(s.tuple()));
            }
        }
#endif

        if (!mods.empty()) { // success should be marked here
            // apply_attribute_transfer(mods);
            if (after(unmods, mods)) {
                return mods; // scope destructor is called
            }
        }
    } catch (const std::exception& e) {
        scope.mark_failed();
        throw e;
    }
    scope.mark_failed();
    return {}; // scope destructor is called
}

bool Operation::before(const simplex::Simplex& simplex) const
{
    if (mesh().is_removed(simplex.tuple()) || !mesh().is_valid(simplex)) {
        return false;
    }

    if (!m_invariants.before(simplex)) {
        return false;
    }


    return true;
}

bool Operation::after(
    const std::vector<simplex::Simplex>& unmods,
    const std::vector<simplex::Simplex>& mods) const
{
    return m_invariants.directly_modified_after(unmods, mods);
}

void Operation::reserve_enough_simplices()
{
    // by default assume we'll at most create N * capacity
    constexpr static int64_t default_preallocation_size = 3;

    auto run = [&](auto&& m) {
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(m)>>) {
            auto cap = m.m_attribute_manager.m_capacities;
            for (auto& v : cap) {
                v *= default_preallocation_size;
            }
            m.reserve_more_attributes(cap);
        }
    };
    run(mesh());
}

} // namespace wmtk::operations
