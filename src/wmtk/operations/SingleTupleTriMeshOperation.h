#pragma once
#include <wmtk/TriMeshOperation.h>
namespace wmtk {
class SingleTupleTriMeshOperation : public TriMeshOperation
{
public:
    virtual ~SingleTupleTriMeshOperation();
    bool operator()(TriMesh& m, const Tuple& t) override;
    operator bool() const { return m_return_tuple_opt.local().has_value(); }

    void set_return_tuple(const TriMeshTuple& t);
    void mark_failed();
    void reset();

    std::optional<TriMeshTuple> get_return_tuple_opt() const { return m_return_tuple_opt.local(); }
    // virtual std::vector<TriMeshTuple> modified_edges(const TriMesh& m) const = 0;
    // virtual std::vector<TriMeshTuple> modified_vertices(const TriMesh& m) const = 0;

private:
    mutable tbb::enumerable_thread_specific<std::optional<TriMeshTuple>> m_return_tuple_opt;
};
} // namespace wmtk
