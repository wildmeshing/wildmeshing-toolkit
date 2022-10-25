#include "IncrementalTetWild.h"
#include <BSP.h>
#include <embed.h>


// we have the vertices and triangles
// to generate what we need for volumemesher
// coords, ncoords, tri_idx, ntri_idx

void volumemesher_main(std::vector<Eigen::Vector3d> verts,
    std::vector<std::array<size_t, 3>> tris,
    double* coords_A, * coords_B = NULL,
    uint32_t ncoords_A = 0, ncoords_B = 0,
    uint32_t* tri_idx_A = NULL, * tri_idx_B = NULL,
    uint32_t ntriidx_A = 0, ntriidx_B = 0){

    bool triangulate = false;
    bool verbose = false;
    bool logging = false;
    bool surfmesh = false;
    bool blackfaces = false;
    char bool_opcode = '0';

    // convert vertices and triangles into usable type
    ncoords_A = verts.size();
    ntriidx_A = tris.size();

    *coords_A = (double*)malloc(sizeof(double) * 3 * ncoords_A);
    *tri_idx_A = (uint32_t*)malloc(sizeof(uint32_t) * 3 * ntriidx_A);

    for(int i=0;i<ncoords_A;i++){
        (*ncoords_A) + (i*3) = verts[i][0];
        (*ncoords_A) + (i*3+1) = verts[i][1];
        (*ncoords_A) + (i*3+2) = verts[i][2];
    }

    for(int i=0;i<ntriidx_A;i++){
        (*tri_idx_A) + (i*3) = tris[i][0];
        (*tri_idx_A) + (i*3+1) = tris[i][1];
        (*tri_idx_A) + (i*3+2) = tris[i][2];
    }

    // do complex
    complex = makePolyhedralMesh(
        "A", coords_A, ncoords_A, tri_idx_A, ntriidx_A,
        "B", coords_B, ncoords_B, tri_idx_B, ntriidx_B,
        bool_opcode, true, verbose, logging
        );

    // extract tet mesh from complex
    



    return;
}

    



