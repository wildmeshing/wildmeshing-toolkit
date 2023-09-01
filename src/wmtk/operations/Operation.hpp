#pragma once
#include <string>
#include <type_traits>
#include <vector>
#include <wmtk/Accessor.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk {
class Mesh;
class InvariantCollection;

namespace operations {
template <typename T>
struct OperationSettings
{
};

class Operation
{
public:
    // main entry point of the operator by the scheduler
    bool operator()();
    virtual std::string name() const = 0;


    virtual ~Operation();

    virtual std::vector<double> priority() const { return {0}; }


    // TODO: spaceship things?
    bool operator<(const Operation& o) const;
    bool operator==(const Operation& o) const;

protected:
    virtual bool execute() = 0;
    // does invariant pre-checks
    virtual bool before() const;
    // does invariant pre-checks
    virtual bool after() const;

    void update_cell_hashes(const std::vector<Tuple>& cells);

    Tuple resurrect_tuple(const Tuple& tuple) const;

    virtual Mesh& base_mesh() const = 0;
    virtual Accessor<long>& hash_accessor() = 0;
    // implements a quiet const_cast to extract a const hash_accessor from the non-const one
    const Accessor<long>& hash_accessor() const;


    static Accessor<long> get_hash_accessor(Mesh& m);
};

} // namespace operations
} // namespace wmtk
