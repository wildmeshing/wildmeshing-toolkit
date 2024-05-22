#pragma once
#include <igl/parallel_for.h>
#include <igl/unproject_onto_mesh.h>

#include <igl/ambient_occlusion.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>

int color_map[20][3] = {
    {243, 156, 18}, // 橙黄
    {192, 57, 43}, // 砖红
    {155, 89, 182}, // 紫罗兰
    {52, 152, 219}, // 亮蓝
    {41, 128, 185}, // 中蓝
    {39, 174, 96}, // 翠绿
    {22, 160, 133}, // 青绿
    {241, 196, 15}, // 金黄
    {230, 126, 34}, // 胡萝卜橙
    {231, 76, 60}, // 西瓜红
    {236, 240, 241}, // 亮灰
    {149, 165, 166}, // 石板灰
    {171, 183, 183}, // 淡灰
    {211, 84, 0}, // 暗橙
    {192, 57, 43}, // 深红
    {189, 195, 199}, // 银灰
    {127, 140, 141}, // 灰石
    {210, 180, 140}, // 茶色
    {255, 228, 181}, // 米色
    {255, 160, 122} // 浅橙红
};

typedef std::tuple<Eigen::Matrix4f, Eigen::Matrix4f, Eigen::Vector4f> camera_info;

std::tuple<std::vector<int>, std::vector<Eigen::Vector3d>>
get_pt_mat(camera_info cam, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, int W, int H)
{
    auto view = std::get<0>(cam);
    auto proj = std::get<1>(cam);
    auto vp = std::get<2>(cam);
    std::cout << "view: " << view << std::endl;
    std::cout << "proj: " << proj << std::endl;
    std::cout << "vp: " << vp << std::endl;
    std::vector<int> fids(W * H, -1);
    std::vector<Eigen::Vector3d> bcs(W * H);

    igl::parallel_for(W * H, [&](int id) {
        int i = id / W;
        int j = id % W;
        int fid;
        Eigen::Vector3f bc;
        double x = vp(2) / (double)W * (j + 0.5);
        double y = vp(3) / (double)H * (H - i - 0.5);
        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view, proj, vp, V, F, fid, bc)) {
            fids[id] = fid;
            bcs[id] = bc.cast<double>();
        }
    });

    return std::make_tuple(fids, bcs);
};


void addShading(
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
    const Eigen::MatrixXd& V3d,
    const Eigen::MatrixXi& F,
    const std::vector<int>& fids,
    const std::vector<Eigen::Vector3d>& bcs,
    const Eigen::Matrix4f& view,
    bool flatShading = false)
{
    auto view_double = view.cast<double>();
    // Compute normals (per face)
    Eigen::MatrixXd normals, pvNormals;
    igl::per_face_normals(V3d, F, Eigen::RowVector3d(1.0, 1.0, 1.0), normals);
    igl::per_vertex_normals(V3d, F, pvNormals);

    Eigen::MatrixXd pvNormals4 = Eigen::MatrixXd::Zero(pvNormals.rows(), 4);
    pvNormals4.block(0, 0, pvNormals.rows(), 3) = pvNormals;
    Eigen::MatrixXd normals4 = Eigen::MatrixXd::Zero(normals.rows(), 4);
    normals4.block(0, 0, normals.rows(), 3) = normals;

    Eigen::VectorXd ao;
    igl::ambient_occlusion(V3d, F, V3d, pvNormals, 500, ao);

    // Normal transformation matrix
    Eigen::Matrix3d normTrans = view_double.block<3, 3>(0, 0).inverse().transpose();

    Eigen::Vector3d lightEye(0.0, 0.3, 0.0);

    std::cout << "R.rows(): " << R.rows() << std::endl;
    std::cout << "R.cols(): " << R.cols() << std::endl;

    igl::parallel_for(
        fids.size(),
        [&](int id)
        // for (int id = 0; id < fids.size(); ++id)
        {
            int i = id % R.rows();
            int j = R.cols() - 1 - id / R.rows();

            int fid = fids[id];
            Eigen::RowVector3d bc = bcs[id];


            if (fid > -1) {
                Eigen::Vector3f diff;
                diff << R(i, j) / 255.0, G(i, j) / 255.0, B(i, j) / 255.0;
                Eigen::Vector3f amb = 0.2 * diff;
                Eigen::Vector3f spec =
                    Eigen::Vector3f::Constant(0.3f) + (0.1f * (diff.array() - 0.3f)).matrix();
                double aoFactor =
                    ao(F(fid, 0)) * bc(0) + ao(F(fid, 1)) * bc(1) + ao(F(fid, 2)) * bc(2);

                Eigen::Vector3d pos = V3d.row(F(fid, 0)) * bc(0) + V3d.row(F(fid, 1)) * bc(1) +
                                      V3d.row(F(fid, 2)) * bc(2);
                Eigen::Vector4d pos4(1.0, 1.0, 1.0, 1.0);
                pos4.head<3>() = pos;
                Eigen::Vector3d posEye = (view_double * pos4).head<3>();
                Eigen::Vector4d norm4;
                if (flatShading) {
                    norm4 = normals4.row(fid);
                } else {
                    norm4 = pvNormals4.row(F(fid, 0)) * bc(0) + pvNormals4.row(F(fid, 1)) * bc(1) +
                            pvNormals4.row(F(fid, 2)) * bc(2);
                }
                Eigen::Vector3d normEye = (normTrans * norm4.head<3>()).normalized();

                // Diff color
                Eigen::Vector3d vecToLightEye = lightEye - posEye;
                Eigen::Vector3d dirToLightEye = vecToLightEye.normalized();
                double clampedDotProd = std::max(dirToLightEye.dot(normEye), 0.0);
                Eigen::Vector3f colorDiff = clampedDotProd * diff;

                // Spec color
                Eigen::Vector3d projToNorm = -dirToLightEye.dot(normEye) * normEye;
                Eigen::Vector3d reflEye = projToNorm - (-dirToLightEye - projToNorm);
                Eigen::Vector3d surfToViewEye = -posEye.normalized();
                clampedDotProd = std::max(0.0, reflEye.dot(surfToViewEye));
                double specFactor = pow(clampedDotProd, 35);
                Eigen::Vector3f colorSpec = specFactor * spec;

                Eigen::Vector3f colorNew = amb + 1.2 * colorDiff + colorSpec;
                for (int k = 0; k < 3; ++k) {
                    colorNew[k] = std::max(0.0f, std::min(1.0f, colorNew[k]));
                }

                R(i, j) = int(double(colorNew[0]) * (0.5 + (1 - aoFactor) * 0.5) * 255);
                G(i, j) = int(double(colorNew[1]) * (0.5 + (1 - aoFactor) * 0.5) * 255);
                B(i, j) = int(double(colorNew[2]) * (0.5 + (1 - aoFactor) * 0.5) * 255);
            }
            // }
        });
}
