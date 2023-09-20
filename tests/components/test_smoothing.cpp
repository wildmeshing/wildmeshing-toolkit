
TEST_CASE("smoothing_using_differentiable_energy")
{
    TriMesh mesh = ten_triangles_with_position();
    OperationSettings<VertexSmoothUsingDifferentiableEnergy> op_settings;
    op_settings.smooth_settings.position =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    op_settings.smooth_settings.smooth_boundary = false;

    auto all_vertices = mesh.get_all(PrimitiveType::Vertex);
    for (const Tuple& v : all_vertices) {
    }
}