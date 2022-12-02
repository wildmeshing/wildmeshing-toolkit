#include "IncrementalTetWild.h"
#include <igl/predicates/ear_clipping.h>


std::vector<std::array<size_t, 3>> tetwild::TetWild::triangulate_polygon_face(std::vector<Vector3r> points){
        
        std::vector<Vector2r> points2d(points.size());
        for (int i=0;i<points.size();i++) points2d.push_back(Vector2r(0,0));
        bool colinear = true;
        // project to z,x,y plane in order
        for (int k=0;k<3;k++){
            colinear = true;
            for (int i=0;i<points.size();i++){
                points2d[i][k % 3] = points[i][k % 3]*1000;
                points2d[i][(k + 1) % 3] = points[i][(k + 1) % 3]*1000;
            }
            for (int i=0;i<points.size()-2;i++){
                auto a = points2d[i]-points2d[i+1];
                auto b = points2d[i+1]-points2d[i+2];
                if (abs(a[0]*b[1]-a[1]*b[0])>1e-8){
                    colinear = false;
                    break;
                }
            }
            if(!colinear){
                break;
            }
        }
        assert(colinear==false);

        // ear clipping
        Eigen::MatrixXd p2d(points2d.size(), 2);
        Eigen::VectorXi rt(points2d.size(), 1);

        for (int i=0;i<points2d.size();i++){
            p2d(i, 0) = points2d[i][0].to_double();
            p2d(i, 1) = points2d[i][1].to_double();
            rt(i, 0) = 0;
        }


        Eigen::VectorXi I;
        Eigen::MatrixXi eF;
        Eigen::MatrixXd nP;

        igl::predicates::ear_clipping(p2d, rt, I, eF, nP);

        std::vector<std::array<size_t, 3>> triangulated_faces;
        std::cout<<"triangulated_local_index"<<std::endl;
        for (int i=0; i<eF.rows(); i++){
            triangulated_faces.push_back({eF(i,0), eF(i,1), eF(i,2)});
            std::cout<<eF(i,0)<<" "<<eF(i,1)<<" "<<eF(i,2)<<std::endl;
        }

        return triangulated_faces;
    }

// we have the vertices and triangles
// to generate what we need for volumemesher
// coords, ncoords, tri_idx, ntri_idx

// embed input surface on generated back ground mesh

void tetwild::TetWild::insertion_by_volumeremesher(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        std::vector<Vector3r> &v_rational,
        std::vector<std::array<size_t, 3>> &facets_after,
        std::vector<bool> &is_v_on_input,
        std::vector<std::array<size_t, 4>> &tets_after){

    std::cout<<"vertices size: "<<vertices.size()<<std::endl;
    std::cout<<"faces size: "<<faces.size()<<std::endl;
    // generate background mesh
    init_from_delaunay_box_mesh(vertices);

    // prepare tet vertices and tet index info 
    
    auto tet_vers = get_vertices();
    auto tets = get_tets();
    std::vector<double> tet_ver_coord(3*tet_vers.size());
    std::vector<uint32_t> tet_index(4*tets.size());
    std::cout<<"tetver size: "<<tet_vers.size()<<std::endl;
    std::cout<<"tet size: "<<tets.size()<<std::endl;
    // tet_ver_coord.reserve(3 * tet_vers.size());
    // tet_index.reserve(4 * tets.size());

    for (int i=0; i < tet_vers.size(); ++i){
        tet_ver_coord[3 * i] = m_vertex_attribute[i].m_posf[0];
        tet_ver_coord[3 * i + 1] = m_vertex_attribute[i].m_posf[1];
        tet_ver_coord[3 * i + 2] = m_vertex_attribute[i].m_posf[2];
    }

    for (int i=0; i < tets.size(); ++i){
        auto tet_vids = oriented_tet_vids(tets[i]);
        tet_index[4 * i] = tet_vids[0];
        tet_index[4 * i + 1] = tet_vids[1];
        tet_index[4 * i + 2] = tet_vids[2];
        tet_index[4 * i + 3] = tet_vids[3];
    }

    // prepare input surfaces info
    std::vector<double> tri_ver_coord(3 * vertices.size());
    std::vector<uint32_t> tri_index(3 * faces.size());
    // tri_ver_coord.reserve(3 * vertices.size());
    // tri_index.reserve(3 * faces.size());

    for (int i=0; i < vertices.size(); ++i){
        tri_ver_coord[3 * i] = vertices[i][0];
        tri_ver_coord[3 * i + 1] = vertices[i][1];
        tri_ver_coord[3 * i + 2] = vertices[i][2];
    }

    for (int i=0; i < faces.size(); ++i){
        tri_index[3 * i] = faces[i][0];
        tri_index[3 * i + 1] = faces[i][1];
        tri_index[3 * i + 2] = faces[i][2];
    }

    std::cout<<tri_ver_coord.size()<<std::endl;
    std::cout<<tri_index.size()<<std::endl;
    std::cout<<tet_ver_coord.size()<<std::endl;
    std::cout<<tet_index.size()<<std::endl;

    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;

    // volumeremesher embed
    vol_rem::embed_tri_in_poly_mesh(
        tri_ver_coord,
        tri_index, 
        tet_ver_coord, 
        tet_index, 
        embedded_vertices, 
        embedded_facets, 
        embedded_cells, 
        embedded_facets_on_input,
        true);

    v_rational.reserve(embedded_vertices.size()/3);

    for (int i=0; i<embedded_vertices.size()/3; i++){
        v_rational.push_back(Vector3r(
            embedded_vertices[3 * i].get_d(),
            embedded_vertices[3 * i + 1].get_d(),
            embedded_vertices[3 * i + 2].get_d()
        ));
    }

    std::vector<std::vector<size_t>> polygon_faces;
    int polycnt = 0;
    for (int i=0;i<embedded_facets.size(); i++){
        int polysize = embedded_facets[i];
        std::vector<size_t> polygon;
        for (int j = i+1; j<= i+ polysize; j++){
            polygon.push_back(embedded_facets[j]);
        }
        polycnt++;
        polygon_faces.push_back(polygon);
        i+=polysize;
    }

    std::cout<<"polycnt: "<<polycnt<<std::endl;
    std::cout<<"polyfaces.size: "<<polygon_faces.size()<<std::endl;

    std::vector<bool> polygon_faces_on_input_surface(polygon_faces.size());
    for(int i=0;i<polygon_faces.size();i++){
        polygon_faces_on_input_surface[i] = false;
    }
    for (int i=0;i<embedded_facets_on_input.size();i++){
        polygon_faces_on_input_surface[embedded_facets_on_input[i]] = true;
    }

    // triangulate polygon faces
    for (int i=0;i<polygon_faces.size();i++){
        std::vector<std::array<size_t, 3>> clipped_indices;
        std::vector<Vector3r> poly_coordinates;
        std::vector<size_t> polygon_face = polygon_faces[i];
        assert(polygon_face.size()>=3);
        if (polygon_face.size() == 3) continue;
        std::cout<<std::endl<<"polyface: ";
        for (int j=0; j<polygon_face.size(); j++){
            std::cout<<polygon_face[j]<<" ";
        }
        std::cout<<std::endl<<"coords: "<<std::endl;
        for (int j=0; j<polygon_faces[i].size(); j++){
            poly_coordinates.push_back(v_rational[polygon_face[j]]);
            std::cout<<v_rational[polygon_face[j]][0]<<" "<<v_rational[polygon_face[j]][1]<<" "<<v_rational[polygon_face[j]][2]<<std::endl;
        }
        std::cout<<std::endl;
        
        clipped_indices = triangulate_polygon_face(poly_coordinates);
        std::cout<<"clipped indices size: "<<clipped_indices.size()<<std::endl;
        for (int j=0;j<clipped_indices.size(); j++){
            // need to map oldface index to new face indices
            std::array<size_t, 3> triangle_face = {polygon_face[clipped_indices[j][0]], polygon_face[clipped_indices[j][1]], polygon_face[clipped_indices[j][2]]};

            std::cout<<triangle_face[0]<<" "<<triangle_face[1]<<" "<<triangle_face[2]<<std::endl;
        }
    }

    




    // tetrahedralize cells

    // v_rational.reserve(embedded_vertices.size()/3);
    // facets_after.reserve(embedded_facets.size()/4);

    // std::cout<<"embedded vertices num: "<<embedded_vertices.size()/3<<std::endl;
    // std::cout<<"embedded facets num: "<<embedded_facets.size()/4<<std::endl;


    // for (int i=0;i<embedded_vertices.size()/3;i++){
    //     v_rational.push_back(Vector3r(
    //         embedded_vertices[3 * i].get_d(),
    //         embedded_vertices[3 * i + 1].get_d(), 
    //         embedded_vertices[3 * i + 2].get_d()
    //         ));

    //     // v_rational[i][1] = embedded_vertices[3 * i + 1].get_d();
    //     // v_rational[i][2] = embedded_vertices[3 * i + 2].get_d();
    // }

    // // check if all generated facets are triangles

    // for (int i=0;i<embedded_facets.size(); i++){
    //     if (embedded_facets[i]!=3){
    //         std::cout<< "is not triangle"<<std::endl;
    //     }
    //     i+=embedded_facets[i];
    // }

    // for (int i=0;i<embedded_facets.size()/4;i++){
    //     std::array<size_t, 3> f = {embedded_facets[i+1], embedded_facets[i+2], embedded_facets[i+3]};
    //     facets_after.push_back(f);
    //     i+=3;
    // }

    // int cnt = 0;

    // for (int i=0;i<embedded_cells.size();i++){
    //     int cellsize = embedded_cells[i];

    //     if (cellsize == 4){
    //         // already a tet

    //         // get all vertices
    //         std::vector<size_t> vs;
    //         for (int j=1;j<=4;j++){
    //             vs.push_back(facets_after[embedded_cells[i+j]][0]);
    //             vs.push_back(facets_after[embedded_cells[i+j]][1]);
    //             vs.push_back(facets_after[embedded_cells[i+j]][2]);
    //         }
    //         wmtk::vector_unique(vs);

    //         // debug code 
    //         if(vs.size() !=4){
    //             for (int k=0;k<vs.size();k++){
    //                 std::cout<<vs[k]<<" ";
    //             }
    //             std::cout<<std::endl;

    //             std::cout<<"xxxx"<<std::endl;

    //             for (int k=i+1;k<=i+4;k++){
    //                 std::cout<<embedded_cells[k] << ": " <<facets_after[embedded_cells[k]][0]<< " "<<facets_after[embedded_cells[k]][1]<< " "<<facets_after[embedded_cells[k]][2]<< std::endl;
    //             }

    //             std::cout<<"yyyy"<<std::endl;
    //             cnt++;
    //         }

    //         if (cnt == 3){
    //             assert(vs.size() == 4);
    //         }
            

    //         // add tet
    //         std::array<size_t, 4> t = {vs[0], vs[1], vs[2], vs[3]};
    //         tets_after.emplace_back(t);
    //         i+=4;
    //     }
    //     else{
    //         //compute the barycenter and split the cell
    //         Vector3r centriod(0,0,0);

    //         std::vector<size_t> vs;
    //         for (int j = 1; j <= cellsize; j++){
    //             vs.push_back(facets_after[embedded_cells[i+j]][0]);
    //             vs.push_back(facets_after[embedded_cells[i+j]][1]);
    //             vs.push_back(facets_after[embedded_cells[i+j]][2]);
    //         }
    //         wmtk::vector_unique(vs);

    //         // debug code 
    //         if(true){
    //             for (int k=0;k<vs.size();k++){
    //                 std::cout<<vs[k]<<" ";
    //             }
    //             std::cout<<std::endl;

    //             std::cout<<"xxxx"<<std::endl;

    //             for (int k=i+1;k<=i+cellsize;k++){
    //                 std::cout<<embedded_cells[k] << ": " <<facets_after[embedded_cells[k]][0]<< " "<<facets_after[embedded_cells[k]][1]<< " "<<facets_after[embedded_cells[k]][2]<< std::endl;
    //             }

    //             std::cout<<"yyyy"<<std::endl;
    //             cnt++;
    //         }

    //         if (cnt >=10){
    //             break;
    //         }

    //         for (int j=0;j<vs.size();j++){
    //             centriod = centriod + v_rational[vs[j]];
    //         }
    //         centriod = centriod/cellsize;

    //         //tetrahedralize the cell
    //         int new_v_index = v_rational.size();
    //         v_rational.emplace_back(centriod);
    //         for (int j = 1; j <= cellsize; j++){
    //             std::array<size_t, 4> t = {
    //                 new_v_index,
    //                 facets_after[embedded_cells[i+j]][0], 
    //                 facets_after[embedded_cells[i+j]][1], 
    //                 facets_after[embedded_cells[i+j]][2]
    //                 };
    //             tets_after.emplace_back(t);
    //         }
    //         i += cellsize;
    //     }
        
    // }

    // is_v_on_input.reserve(v_rational.size());
    // for (int i=0;i<v_rational.size();i++){
    //     is_v_on_input.push_back(false);
    // }

    // for (int i=0;i<embedded_facets_on_input.size();i++){
    //     is_v_on_input[facets_after[embedded_facets_on_input[i]][0]] = true;
    //     is_v_on_input[facets_after[embedded_facets_on_input[i]][1]] = true;
    //     is_v_on_input[facets_after[embedded_facets_on_input[i]][2]] = true;
    // }

    // std::cout<<"vertices num after tetrahedralize: "<<v_rational.size()<<std::endl;
    // std::cout<<"tets num after tetrahedralize: "<<tets_after.size()<<std::endl;

    // wmtk::vector_unique(tets_after);
    // std::cout<<"unique tets num after tetrahedralize: "<<tets_after.size()<<std::endl;

}

void tetwild::TetWild::init_from_Volumeremesher(
        std::vector<Vector3r> &v_rational,
        std::vector<std::array<size_t, 3>> &facets,
        std::vector<bool> &is_v_on_input,
        std::vector<std::array<size_t, 4>> &tets){

    init(v_rational.size(), tets);
    assert(check_mesh_connectivity_validity());
    


    // m_vertex_attribute.m_attributes.resize(v_rational.size());
    // m_tet_attribute.m_attributes.resize(tets.size());
    // m_face_attribute.m_attributes.resize(tets.size() * 4);

    // for (int i = 0; i < vert_capacity(); i++){
    //     m_vertex_attribute[i].m_pos = v_rational[i];
    //     m_vertex_attribute[i].m_posf = Vector3d(v_after_insertion[i][0], v_after_insertion[i][1], v_after_insertion[i][2]);
    // }

    // // track faces
    // for (int i = 0; i < is_f_surface.size(); i++){
    //     m_vertex_attribute[f_after_insertion[is_f_surface[i]][0]].m_is_on_surface = true;
    //     m_vertex_attribute[f_after_insertion[is_f_surface[i]][1]].m_is_on_surface = true;
    //     m_vertex_attribute[f_after_insertion[is_f_surface[i]][2]].m_is_on_surface = true;
    // }

    // auto faces = get_faces();
    // for (auto f : faces){
    //     auto v1 = f.vid(*this);
    //     auto v2 = f.switch_vertex(*this).vid(*this);
    //     auto v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
    //     if(m_vertex_attribute[v1].m_is_on_surface && m_vertex_attribute[v1].m_is_on_surface && m_vertex_attribute[v1].m_is_on_surface){
    //         m_face_attribute[f.fid(*this)].m_is_surface_fs = 1;
    //     }
    // }

    // // track bounding box

    // for (int i = 0; i < faces.size(); i++) {
    //     auto vs = get_face_vertices(faces[i]);
    //     std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
    //     int on_bbox = -1;
    //     for (int k = 0; k < 3; k++) {
    //         if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
    //             m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k] &&
    //             m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_min[k]) {
    //             on_bbox = k * 2;
    //             break;
    //         }
    //         if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
    //             m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k] &&
    //             m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_max[k]) {
    //             on_bbox = k * 2 + 1;
    //             break;
    //         }
    //     }
    //     if (on_bbox < 0) continue;
    //     auto fid = faces[i].fid(*this);
    //     m_face_attribute[fid].m_is_bbox_fs = on_bbox;
        
    //     for (size_t vid : vids) {
    //         m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
    //     }
    // }


}
    