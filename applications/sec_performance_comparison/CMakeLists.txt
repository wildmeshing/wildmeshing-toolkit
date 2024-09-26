include(wmtk_add_application)
wmtk_add_application(sec_performance_comparison_app
    sec_performance_comparison_main.cpp
    )


# marching requires the input component and the marching component
target_link_libraries(sec_performance_comparison_app PRIVATE
wmtk::input
wmtk::tetwild_simplification
wmtk::shortestedge_collapse
wmtk::output 
)