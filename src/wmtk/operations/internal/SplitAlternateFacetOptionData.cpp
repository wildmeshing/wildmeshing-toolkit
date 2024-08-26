
wmtk::autogen::Dart dart = sd.dart_from_tuple(t);

const PrimitiveType boundary_type = mesh_pt - 1;
auto get_ear_max_subdart_size = [&](int8_t action) {
    const int8_t orientation = sd.product(action, dart.local_orientation());
    const int8_t simplex = sd.simplex_index(orientation, boundary_type);
};

const int8_t left_ear_max_subdart_size = get_ear_max_subdart_size(left_ear_action(mesh_pt));
const int8_t right_ear_max_subdart_size = get_ear_max_subdart_size(right_ear_action(mesh_pt));


else if (int8_t preserved_subdarts_to_face = 0; preserved_subdarts_to_face > 0)
{ // TODO: make this do something

    new_global_cid = alts[0];
}
else
{
    new_global_cid = alts[1];
}

