


template <typename T>
class UpdateStrategy
{
public:
    // spine simplices = simplices that are subsets of the original split edge
    // rib simplices are the simplices orothogal to the input edge


    // provide an entry point for the new simplex that happen orthogonal to the input edge
    // (including the new vertex)
    virtual void update_split_rib_simplex(const Simplex& original_edge, const Simplex& new_simplex)
    {
        auto acc = mesh.create_attribute(m_attribute_handle);

        if (std::is_same_v<double, T> && target.primitive_type() == Vertex) {
            // take the average of the two options
            average_update(acc, original_edge, new_simplex);
        } else {
            // just default copy the value from the original simplex
            copy_update(acc, original_edge, new_simplex);
        }
    }

    // provide an entry point for any simplex that comes from an old mesh that was split in half (or
    // by some weighted ratio ;-)
    virtual void update_split_spine_simplex(
        const PrimitiveType pt,
        const Simplex& original_simplex,
        const std::array<Simplex, 2>& splitted_simplices)
    {
        auto acc = mesh.create_attribute(m_attribute_handle);
        for (const auto& s : splitted_simplices) {
            copy_update(acc, original_simplex, s);
        }
    }

    // provide an entry point for any simplex and hte two simplices that it came from (assuming a
    // semantic where it comes from its "ears"
    virtual void update_collapsed_simplex(
        const std::array<Simplex, 2>& original_ear_simplices,
        const Simplex& collapsed)
    {
        if (std::is_same_v<double, T> && target.primitive_type() == Vertex) {
            average_update(original_ear_simplices, collapsed);
        } else {
            // navigate to the correct ear
            // for vertex it's just the input edge's vertex
            // for edge on tri it's the SE
            copy_update(original_ear_simplices[0], collapsed);
        }
    }


    // default copy implementation using checkpoints
    void copy_update(const Accessor<T>& acc, const Simplex& original_edge, const Simplex& target)
    {
        acc.vector_attribute(target.tuple()) = m_mesh.previous_checkpoint([&]() {
            // get the two opposing vertices
            const Tuple t = original_edge.tuple();
            return acc.vector_attribute(t);
        })();
    }

    // helper to enable vertex updates (where we got a single simplex as input)
    void average_update(const Accessor<T>& acc, const Simplex& original_edge, const Simplex& target)
    {
        assert(std::is_same_v<T, double>);
        assert(target.primitive_type() == Vertex);
        const Tuple t = original_edge.tuple();
        const Tuple ot = m_mesh.switch_vertex(t);
        average_update(acc, std::array<Simplex, 2>{{vertex(t), vertex(ot)}}, target);
    }
    // default average implementation using checkpoints
    void average_update(
        const Accessor<T>& acc,
        const std::array<Simplex, 2>& original_simplices,
        const Simplex& target)
    {
        const auto& [a, b] = original_simplices;

        acc.vector_attribute(target.tuple()) = m_mesh.previous_checkpoint([&]() {
            return .5 * (acc.vector_attribute(a.tuple()) + acc.vector_attribute(b.tuple()));
        })();
    }


private:
    const Mesh& m_mesh;
    const MeshAttributeHandle<T> m_attribute_handle;
};
} // namespace wmtk::attribute::update_strategies

