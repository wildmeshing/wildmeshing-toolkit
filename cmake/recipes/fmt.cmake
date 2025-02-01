# fmt (https://github.com/fmtlib/fmt)
# License: mit

if(TARGET fmt::fmt)
    return()
endif()


include(CPM)
CPMAddPackage("gh:fmtlib/fmt#11.0.2")

set_target_properties(fmt PROPERTIES FOLDER third_party)
