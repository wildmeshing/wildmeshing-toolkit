wmtk_add_application(tetwild_msh_converter_app
    tetwild_msh_converter_main.cpp
    tetwild_msh_converter_spec.hpp
    )

target_link_libraries(tetwild_msh_converter_app PRIVATE
wmtk::input)