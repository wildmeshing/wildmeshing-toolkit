#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
}

namespace wmtk::simplex::utils {
class SimplexComparisons
{
public:
    /* @brief checks if simplex objects represent the same simplex
     * @param m the mesh these simplices belong to
     * @param s0 the first simplex compared
     * @param s1 the second simplex compared
     * @return true if two simplices are the same
     * */
    static bool equal(const Mesh& m, const Simplex& s0, const Simplex& s1);

    /* @brief checks if simplex objects represent the same simplex
     * @param m the mesh these simplices belong to
     * @param a the first simplex compared's tuple
     * @param a_pt the first simplex compared's primitive type
     * @param b the second simplex compared's tuple
     * @param b_pt the second simplex compared's primitive_type
     * @return true if two simplices are the same
     * */
    static bool
    equal(const Mesh& m, const Tuple& a, PrimitiveType a_pt, const Tuple& b, PrimitiveType b_pt);

    /* @brief checks if simplex objects of the same dimension represent the same simplex
     * @param m the mesh these simplices belong to
     * @param primitive_type the primitive type of the two simplices
     * @param a the first simplex compared's tuple
     * @param b the second simplex compared's tuple
     * @return true if two simplices are the same
     * */
    static bool equal(const Mesh& m, PrimitiveType primitive_type, const Tuple& a, const Tuple& b);


    /* @brief checks if simplex objects are less than one another
     *
     * uses lexicgoraphic order of (primtiive type, id)
     *
     * @param m the mesh these simplices belong to
     * @param s0 the first simplex compared
     * @param s1 the second simplex compared
     * @return true if two simplices are the same
     * */
    static bool less(const Mesh& m, const Simplex& s0, const Simplex& s1);
    /* @brief checks if simplex objects are less than one another
     *
     * uses lexicgoraphic order of (primtiive type, id)
     *
     * @param m the mesh these simplices belong to
     * @param a the first simplex compared's tuple
     * @param a_pt the first simplex compared's primitive type
     * @param b the second simplex compared's tuple
     * @param b_pt the second simplex compared's primitive_type
     * @return true if two simplices are the same
     * */
    static bool
    less(const Mesh& m, const Tuple& a, PrimitiveType a_pt, const Tuple& b, PrimitiveType b_pt);
    /* @brief checks if simplex objects are less than one another
     *
     * uses lexicgoraphic order of (primtiive type, id)
     *
     * @param m the mesh these simplices belong to
     * @param primitive_type the primitive type of the two simplices
     * @param a the first simplex compared's tuple
     * @param b the second simplex compared's tuple
     * @return true if two simplices are the same
     * */
    static bool less(const Mesh& m, PrimitiveType primitive_type, const Tuple& a, const Tuple& b);
};
} // namespace wmtk::simplex::utils
