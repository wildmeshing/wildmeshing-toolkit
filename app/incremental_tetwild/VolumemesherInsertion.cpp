#include "IncrementalTetWild.h"
#include <igl/predicates/ear_clipping.h>


std::vector<std::array<size_t, 3>> tetwild::TetWild::triangulate_polygon_face(std::vector<Vector3r> points){
        
        std::vector<Vector2r> points2d;
        // std::cout<<points.size()<<std::endl;
        for (int i=0;i<points.size();i++) points2d.push_back(Vector2r(0,0));
        bool colinear = true;
        // project to z,x,y plane in order
        for (int k=0;k<3;k++){
            colinear = true;
            for (int i=0;i<points.size();i++){
                points2d[i][0] = points[i][k % 3]*1000;
                points2d[i][1] = points[i][(k + 1) % 3]*1000;
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
                // std::cout<<"zxy?: "<<k<<std::endl;  
                break;
            }
        }
        assert(colinear==false);

        // ear clipping
        Eigen::MatrixXd p2d(points2d.size(), 2);
        Eigen::VectorXi rt(points2d.size(), 1);

        // std::cout<<points2d.size()<<std::endl;

        for (int i=0;i<points2d.size();i++){
            p2d(i, 0) = points2d[i][0].to_double();
            p2d(i, 1) = points2d[i][1].to_double();
            rt(i, 0) = 0;
        }
        // std::cout<<"2d coords: "<<std::endl;
        // for (int i=0;i<p2d.rows();i++){
        //     std::cout<<p2d(i, 0)<<" "<<p2d(i, 1)<<std::endl;
        // }


        Eigen::VectorXi I;
        Eigen::MatrixXi eF;
        Eigen::MatrixXd nP;

        igl::predicates::ear_clipping(p2d, rt, I, eF, nP);

        std::vector<std::array<size_t, 3>> triangulated_faces;
        // std::cout<<"triangulated_local_index"<<std::endl;

        if (eF.rows() != p2d.rows()-2){
            // deal with inversed orientation
            Eigen::MatrixXd p2d_inv(points2d.size(), 2);
            Eigen::VectorXi I_inv;
            Eigen::MatrixXi eF_inv;
            Eigen::MatrixXd nP_inv;
            for (int i=0;i<p2d.rows();i++){
                p2d_inv(i,0) = p2d(p2d.rows()-i-1,0);
                p2d_inv(i,1) = p2d(p2d.rows()-i-1,1);
            }
            igl::predicates::ear_clipping(p2d_inv, rt, I_inv, eF_inv, nP_inv);

            for (int i=0; i<eF_inv.rows(); i++){
                triangulated_faces.push_back({p2d.rows()-1-eF_inv(i,0), p2d.rows()-1-eF_inv(i,1), p2d.rows()-1-eF_inv(i,2)});
                // std::cout<<p2d.rows()-1-eF_inv(i,0)<<" "<<p2d.rows()-1-eF_inv(i,1)<<" "<<p2d.rows()-1-eF_inv(i,2)<<std::endl;
            }

            return triangulated_faces;
        }

        
        for (int i=0; i<eF.rows(); i++){
            triangulated_faces.push_back({eF(i,0), eF(i,1), eF(i,2)});
            // std::cout<<eF(i,0)<<" "<<eF(i,1)<<" "<<eF(i,2)<<std::endl;
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

//     Eigen::MatrixXd p2d(7, 2);
// //     p2d<<15.0485, 36.0371,
// // 15.1971, 36.0141,
// // 9.05103, 33.9813,
// // 9.17559, 34.6561,
// // 9.24641, 35.0398,
// // 11.6235, 35.4484,
// // 12.9259, 35.6722;

//     p2d<<12.9259, 35.6722,
//             11.6235, 35.4484,
//             9.24641, 35.0398,
//             9.17559, 34.6561,
//             9.05103, 33.9813,
//             15.1971, 36.0141,
//             15.0485, 36.0371;

//     Eigen::VectorXi rt(7, 1);
//     rt<<0,0,0,0,0,0,0;

//     Eigen::VectorXi I;
//     Eigen::MatrixXi eF;
//     Eigen::MatrixXd nP;

//     igl::predicates::ear_clipping(p2d, rt, I, eF, nP);

//     std::vector<std::array<size_t, 3>> triangulated_faces;
//     std::cout<<"triangulated_local_index_test: -------------"<<std::endl;
//     for (int i=0; i<eF.rows(); i++){
//         triangulated_faces.push_back({eF(i,0), eF(i,1), eF(i,2)});
//         std::cout<<eF(i,0)<<" "<<eF(i,1)<<" "<<eF(i,2)<<std::endl;
//     }

//     return ;

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

    // v_rational.reserve(embedded_vertices.size()/3);

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

    std::vector<std::array<size_t,3>> triangulated_faces;
    std::vector<bool> triangulated_faces_on_input;
    std::vector<std::vector<size_t>> map_poly_to_tri_face(polygon_faces.size());

    // triangulate polygon faces
    for (int i=0;i<polygon_faces.size();i++){
        // already clipped in other polygon
        if (map_poly_to_tri_face[i].size()!=0) continue;

        // new polygon face to clip
        std::vector<std::array<size_t, 3>> clipped_indices;
        std::vector<Vector3r> poly_coordinates;
        std::vector<size_t> polygon_face = polygon_faces[i];
        assert(polygon_face.size()>=3);

        if (polygon_face.size() == 3) {
            // already a triangle
            std::array<size_t, 3> triangle_face = {polygon_face[0], polygon_face[1], polygon_face[2]};
            int idx = triangulated_faces.size();
            triangulated_faces.push_back(triangle_face);
            if(polygon_faces_on_input_surface[i]){
                triangulated_faces_on_input.push_back(true);
            }
            else{
                triangulated_faces_on_input.push_back(false);
            }
            map_poly_to_tri_face[i].push_back(idx);
        }
        else{
            // std::cout<<std::endl<<"polyface: ";
            // for (int j=0; j<polygon_face.size(); j++){
                // std::cout<<polygon_face[j]<<" ";
            // }
            // std::cout<<std::endl<<"coords: "<<std::endl;
            for (int j=0; j<polygon_faces[i].size(); j++){
                poly_coordinates.push_back(v_rational[polygon_face[j]]);
                // std::cout<<v_rational[polygon_face[j]][0]<<" "<<v_rational[polygon_face[j]][1]<<" "<<v_rational[polygon_face[j]][2]<<std::endl;
            }
            // std::cout<<std::endl;
            
            clipped_indices = triangulate_polygon_face(poly_coordinates);
            // std::cout<<"clipped indices size: "<<clipped_indices.size()<<std::endl;
            for (int j=0;j<clipped_indices.size(); j++){
                // need to map oldface index to new face indices
                std::array<size_t, 3> triangle_face = {polygon_face[clipped_indices[j][0]], polygon_face[clipped_indices[j][1]], polygon_face[clipped_indices[j][2]]};
                // std::cout<<triangle_face[0]<<" "<<triangle_face[1]<<" "<<triangle_face[2]<<std::endl;
                int idx = triangulated_faces.size();
                triangulated_faces.push_back(triangle_face);

                // track input faces
                if(polygon_faces_on_input_surface[i]){
                    triangulated_faces_on_input.push_back(true);
                }
                else{
                    triangulated_faces_on_input.push_back(false);
                }
                map_poly_to_tri_face[i].push_back(idx);
            }
        }
        
    }

    std::cout<<triangulated_faces.size()<<std::endl;
    int sum = 0;
    for (int i=0;i<map_poly_to_tri_face.size();i++){
        sum += map_poly_to_tri_face[i].size();
    }
    std::cout<<sum<<std::endl;
    std::cout<<"finish triangulation"<<std::endl;

    std::cout<<"vertice before tetra num: "<<v_rational.size()<<std::endl;

    // tetrahedralize cells
    std::vector<std::vector<size_t>> polygon_cells;
    std::vector<std::array<size_t, 4>> tets_final;
    for (int i=0;i<embedded_cells.size();i++){
        std::vector<size_t> polygon_cell;
        int cellsize = embedded_cells[i];
        for (int j=1; j<=cellsize; j++){
            polygon_cell.push_back(embedded_cells[i+j]);
        }
        polygon_cells.push_back(polygon_cell);
        i+=cellsize;
    }

    std::cout<<"polygon cells num: "<<polygon_cells.size()<<std::endl;

    int was_tet_cnt = 0;
    for (int i=0;i<polygon_cells.size();i++){
        
        auto polygon_cell = polygon_cells[i];

        // get polygon vertices
        std::vector<size_t> polygon_vertices;
        for (auto f: polygon_cell){
            for (auto v: polygon_faces[f]){
                polygon_vertices.push_back(v);
            }
        }
        wmtk::vector_unique(polygon_vertices);
        
        // compute number of triangle faces
        int num_faces = 0;
        for (auto f: polygon_cell){
            num_faces += map_poly_to_tri_face[f].size();
        }

        // polygon already a tet
        if (num_faces == 4){
            was_tet_cnt++;
            assert(polygon_vertices.size()==4);
            std::array<size_t, 4> tetra = {polygon_vertices[0], polygon_vertices[1],polygon_vertices[2],polygon_vertices[3]};
            tets_final.push_back(tetra);
            continue;
        }

        // compute centroid
        Vector3r centroid(0,0,0);
        for (auto v: polygon_vertices){
            centroid = centroid + v_rational[v];
        }
        centroid = centroid/polygon_vertices.size();

        // trahedralize
        size_t centroid_idx = v_rational.size();
        v_rational.push_back(centroid);

        for (auto f: polygon_cell){
            for( auto t: map_poly_to_tri_face[f]){
                std::array<size_t, 4> tetra = {centroid_idx, triangulated_faces[t][0], triangulated_faces[t][1], triangulated_faces[t][2]};
                tets_final.push_back(tetra);

                //debug code
                if(tets_final.size()==91){
                    std::cout<<"debug tet 91: ----------------"<<std::endl;
                    std::cout<<"polygon index: "<<i<<std::endl;
                    std::cout<<"polygon faces: "<<std::endl;
                    for (auto ff: polygon_cell){
                        std::cout<<ff<<" ";
                    }
                    std::cout<<std::endl;
                    std::cout<<std::endl;
                    
                    for (auto ff: polygon_cell){
                        for (int k=0; k<polygon_faces[ff].size();k++){
                            std::cout<<polygon_faces[ff][k]<<" ";
                        }
                        std::cout<<std::endl;
                        std::cout<<std::endl;
                    }
                    std::cout<<"triangulated_faces: "<<std::endl;
                    for (auto ff: polygon_cell){
                        for (auto tt:map_poly_to_tri_face[ff]){
                            std::cout<<triangulated_faces[tt][0]<<" "<<triangulated_faces[tt][1]<<" "<<triangulated_faces[tt][2]<<std::endl;
                        }
                        std::cout<<std::endl;
                    } 

                    std::cout<<"end debug ---------------------"<<std::endl;
                }
            }
        }
    }

    std::cout<<"polygon was tet num: "<<was_tet_cnt<<std::endl;
    std::cout<<"vertices final num: "<<v_rational.size()<<std::endl;
    std::cout<<"tets final num: "<<tets_final.size()<<std::endl;

    facets_after = triangulated_faces;
    tets_after = tets_final;

    // track vertices on input
    is_v_on_input.reserve(v_rational.size());
    for (int i=0;i<v_rational.size();i++) is_v_on_input[i] = false;
    for(int i=0;i<triangulated_faces.size();i++){
        if(triangulated_faces_on_input[i]){
            is_v_on_input[triangulated_faces[i][0]] = true;
            is_v_on_input[triangulated_faces[i][1]] = true;
            is_v_on_input[triangulated_faces[i][2]] = true;
        }
    }

    //check 4277

    for (int i=0;i<embedded_facets.size();i++){
        if(embedded_facets[i]==4277) std::cout<<"found 4277"<<std::endl;
    }

    for (int i=4277;i<4279;i++){
        std::cout<<v_rational[i][0]<<" "<<v_rational[i][1]<<" "<<v_rational[i][2]<<std::endl;
    }

    //check tet 90 91 93 94
    std::cout<<"tet 90: "<<tets_final[90][0]<<" "<<tets_final[90][1]<<" "<<tets_final[90][2]<<" "<<tets_final[90][3]<<std::endl;
    std::cout<<"tet 91: "<<tets_final[91][0]<<" "<<tets_final[91][1]<<" "<<tets_final[91][2]<<" "<<tets_final[91][3]<<std::endl;
    std::cout<<"tet 93: "<<tets_final[92][0]<<" "<<tets_final[92][1]<<" "<<tets_final[92][2]<<" "<<tets_final[92][3]<<std::endl;
    std::cout<<"tet 93: "<<tets_final[93][0]<<" "<<tets_final[93][1]<<" "<<tets_final[93][2]<<" "<<tets_final[93][3]<<std::endl;
    std::cout<<"tet 94: "<<tets_final[94][0]<<" "<<tets_final[94][1]<<" "<<tets_final[94][2]<<" "<<tets_final[94][3]<<std::endl;

}

void tetwild::TetWild::init_from_Volumeremesher(
        std::vector<Vector3r> &v_rational,
        std::vector<std::array<size_t, 3>> &facets,
        std::vector<bool> &is_v_on_input,
        std::vector<std::array<size_t, 4>> &tets){

    init_with_isolated_vertices(v_rational.size(), tets);
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
    