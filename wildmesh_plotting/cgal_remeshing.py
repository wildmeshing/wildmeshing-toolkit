
from __future__ import print_function
from audioop import avg

from CGAL import CGAL_Polygon_mesh_processing


from CGAL.CGAL_Polyhedron_3 import Polyhedron_3

from CGAL.CGAL_Kernel import Point_3

import argparse
import os
import igl
import sys
from timeit import default_timer as timer

input = sys.argv[1]
output = sys.argv[2]

target = sys.argv[3]


def get_poly(input, target):
    v, f = igl.read_triangle_mesh(input)
    print("Before # vertices :", len(v))
    print("Before # tris :", len(f))
    avgedgelen = igl.avg_edge_length(v, f)
    print("Before average len : ", avgedgelen)
    P = Polyhedron_3()
    for face in f:
        a = Point_3(v[face[0]][0], v[face[0]][1], v[face[0]][2])
        b = Point_3(v[face[1]][0], v[face[1]][1], v[face[1]][2])
        c = Point_3(v[face[2]][0], v[face[2]][1], v[face[2]][2])
        P.make_triangle(a, b, c)

    return P, avgedgelen


def test_meshing_functions(input, output, target):
    print("Testing cgal meshing functions...")

# // isotropic_remeshing (4.8)
    P, avg_len = get_poly(input, target)
    hlist = []
    for hh in P.halfedges():
        hlist.append(hh)
    flist = []
    for fh in P.facets():
        flist.append(fh)
    targetlen = avg_len * float(target)
    print("traget avg eedge len is ", targetlen)
    start = timer()
    CGAL_Polygon_mesh_processing.isotropic_remeshing(
        flist, float(targetlen), P, 2, hlist, False)
    end = timer()
    filename = input.split("/")
    filename = filename[-1].split(".")[0]
    print("remeshing use : ", end - start, " sec")
    # P.write_to_file(str(output)+str(targetlen)+".off")
    output = filename+"_cgal_remesh_"+str(targetlen)+".off"
    P.write_to_file(output)
    v, f = igl.read_triangle_mesh(output)
    newavglen = igl.avg_edge_length(v, f)
    print("After # vertices :", len(v))
    print("After # tris :", len(f))
    print("After avg len is ", newavglen)


test_meshing_functions(input, output, target)
