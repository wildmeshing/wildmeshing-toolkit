#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Accessor.hpp>

namespace wmtk{

    enum class SimplexFlag: char {
        Active = 0x1,
    };

    template <SimplexFlag Flag>
    bool is_flag_active(char value) {
        return (value & static_cast<char>(Flag)) != 0;
    }

    template <SimplexFlag Flag>
    void activate_flag(char &c) {
        c |= static_cast<char>(Flag);
    }
    template <SimplexFlag Flag>
    void deactivate_flag(char &c) {
            c = c - (c & static_cast<char>(Flag));
    }
    template <SimplexFlag Flag>
    char with_flag_active(char c) {
        char cc = c;
        activate_flag<Flag>(cc);
        return cc;
    }
    template <SimplexFlag Flag>
    char with_flag_inactive(char c) {
        char cc = c;
        deactivate_flag<Flag>(cc);
        return cc;
    }

}
